#!/usr/bin/env python2
# Generic Imports
from __future__ import print_function

import threading

# Stretch Imports
import hello_helpers.hello_misc as hm
import numpy as np

# ROS Stuff
import rospy
import stretch_funmap.navigate as nv
from sensor_msgs.msg import JointState

# Messages
from std_msgs.msg import Bool, String


class stretch_with_stretch(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.move_base = nv.MoveBase(self)

        # Rate for rospy.Rate() called in main
        self.rate = 20

        # Subscribers
        self.joint_states_subscriber = rospy.Subscriber(
            "/stretch/joint_states", JointState, self.joint_state_callback
        )
        self.select_exercise_subscriber = rospy.Subscriber(
            "/sws_select_exercise", String, self.select_exercise_callback
        )

        # Publishers
        self.notify_publisher = rospy.Publisher("/sws_notify", Bool, queue_size=1)
        self.sws_ready_publisher = rospy.Publisher("/sws_ready", Bool, queue_size=1)
        self.wrist_contact_publisher = rospy.Publisher(
            "/sws_contact_detected", Bool, queue_size=1
        )
        self.sws_start_exercise_publisher = rospy.Publisher(
            "/sws_start_exercise", String, queue_size=1
        )
        self.sws_stop_exercise_publisher = rospy.Publisher(
            "/sws_stop_exercise", Bool, queue_size=1
        )

        # Joint State Inits
        self.joint_states = None

        self.lift_position = None
        self.lift_effort = None

        self.wrist_extension = None
        self.wrist_extension_effort = None

        self.wrist_yaw = None
        self.wrist_yaw_effort = None

        # Threading for callbacks
        self.joint_states_lock = threading.Lock()

        # Internal variables
        self.lift_effort_max = 50  # N?
        self.lift_effort_min = 35  # N?
        self.wrist_extension_max = 10  # N?
        self.wrist_extension_min = -10  # N?
        self.wrist_yaw_effort_contact_threshold = 0.35  # N

        # Exercise Init
        self.current_exercise = None

        # Exercise Configuration
        self.calibration_pose = {
            "joint_lift": 0.75,  # m
            "wrist_extension": 0.05,  # m
            "joint_wrist_yaw": 0.0,  # rad
            "joint_gripper_finger_left": 0.0,
        }
        self.calibration_xya = None
        self.exercise_radius = 0.635  # m (average human arm length)

    def joint_state_callback(self, joint_states):
        # Update Joint State
        with self.joint_states_lock:
            self.joint_states = joint_states

        # Unpack Joint State
        lift_position, lift_velocity, lift_effort = hm.get_lift_state(joint_states)
        wrist_position, wrist_velocity, wrist_effort = hm.get_wrist_state(joint_states)

        self.wrist_extension_effort = joint_states.effort[
            joint_states.name.index("wrist_extension")
        ]
        self.lift_effort = joint_states.effort[joint_states.name.index("joint_lift")]
        self.wrist_yaw_effort = joint_states.effort[
            joint_states.name.index("joint_wrist_yaw")
        ]

        # Store necessary items
        self.lift_position = lift_position
        self.wrist_position = wrist_position

    def select_exercise_callback(self, data):
        self.current_exercise = data.data

    def check_for_wrist_contact(self, publish=False):
        bool_wrist_contact = False
        if self.wrist_yaw_effort is not None:

            # rospy.loginfo("wrist_yaw_effort: {}".format(self.wrist_yaw_effort))
            if abs(self.wrist_yaw_effort) > self.wrist_yaw_effort_contact_threshold:
                bool_wrist_contact = True

            # rospy.loginfo("lift_effort: {}".format(self.lift_effort))
            # if (
            #     self.lift_effort < self.lift_effort_min
            #     or self.lift_effort > self.lift_effort_max
            # ):
            #     bool_wrist_contact = True

            # rospy.loginfo("wrist_extension_effort: {}".format(self.wrist_extension_effort))
            # if (
            #     self.wrist_extension_effort < self.wrist_extension_min
            #     or self.wrist_extension_effort > self.wrist_extension_max
            # ):
            #     bool_wrist_contact = True

        if publish:
            self.wrist_contact_publisher.publish(bool_wrist_contact)

        return bool_wrist_contact

    def wait_for_calibration_handshake(self):
        rate = rospy.Rate(self.rate)

        self.move_to_pose(self.calibration_pose)
        rospy.sleep(2)  # give robot time to settle

        rospy.loginfo("*" * 40)
        rospy.loginfo("Shake hands with the robot to start the game!")
        rospy.loginfo("*" * 40)

        while not rospy.is_shutdown():
            self.sws_ready_publisher.publish(False)
            if self.check_for_wrist_contact(publish=False):
                break
            rate.sleep()

        # set x, y, a here in case we move the robot before calibration
        self.calibration_xya, _ = self.get_robot_floor_pose_xya()

    def wait_for_exercise_start(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.sws_ready_publisher.publish(True)
            if self.current_exercise is not None:
                break
            rate.sleep()

        if rospy.is_shutdown():
            return

        rospy.loginfo("Exercise {} selected".format(self.current_exercise))

        rospy.loginfo("Start repositioning the robot...")

        current_xya, _ = self.get_robot_floor_pose_xya()

        # reposition wrist
        wrist_extension = self.calibration_pose["wrist_extension"]
        self.move_to_pose({"wrist_extension": wrist_extension}, async=False)

        # turn back
        delta_a = self.calibration_xya[2] - current_xya[2]
        at_goal = self.move_base.turn(delta_a, publish_visualizations=False)

        # move base
        delta_x = self.calibration_xya[0] - current_xya[0]
        if self.current_exercise == "A":  # go to the right
            delta_x += self.exercise_radius / 2
        elif self.current_exercise == "B":  # go to the left
            delta_x -= self.exercise_radius / 2
        at_goal = self.move_base.forward(delta_x, detect_obstacles=False)

        if self.current_exercise == "C":
            # for debugging
            return

        # turn
        if self.current_exercise == "A":  # go to the right
            delta_a = np.deg2rad(30.0)
        elif self.current_exercise == "B":  # go to the left
            delta_a = np.deg2rad(-30.0)
        at_goal = self.move_base.turn(delta_a, publish_visualizations=False)

        rospy.loginfo("Start repositioning the robot... done!")

        # start exercise
        rospy.loginfo("Exercise {} starting...".format(self.current_exercise))
        self.sws_start_exercise_publisher.publish(self.current_exercise)

    def wait_for_exercise_stop(self):
        if self.current_exercise == "C":
            # for debugging
            return

        rate = rospy.Rate(self.rate)

        extra, duration = 3, 10
        start_time = rospy.Time.now().secs
        delay_time = (
            start_time + extra
        )  # give a couple extra seconds for the startup sound
        stop_time = delay_time + duration + extra
        while not rospy.is_shutdown():
            self.sws_ready_publisher.publish(False)
            self.check_for_wrist_contact(publish=True)

            now = rospy.Time.now().secs
            if now > stop_time:
                break
            elif now > delay_time:
                # movement
                pos = self.wrist_position + 0.01
                self.move_to_pose({"wrist_extension": pos}, async=True)

            rate.sleep()

        if rospy.is_shutdown():
            return

        # stop exercise
        rospy.loginfo("Exercise {} complete!".format(self.current_exercise))
        self.sws_stop_exercise_publisher.publish(True)
        self.current_exercise = None

        self.notify_publisher.publish(True)  # notify

        rospy.loginfo("Start repositioning the robot...")

        current_xya, _ = self.get_robot_floor_pose_xya()

        # reposition wrist
        wrist_extension = self.calibration_pose["wrist_extension"]
        self.move_to_pose({"wrist_extension": wrist_extension}, async=False)

        # turn back
        delta_a = self.calibration_xya[2] - current_xya[2]
        at_goal = self.move_base.turn(delta_a, publish_visualizations=False)

        rospy.loginfo("Start repositioning the robot... done!")

    def exercise_forward_kinematics(self, theta_degrees):
        # returns the x (base travel) and y (arm extension) given target reach angle theta

        # Shorthand
        r = self.exercise_radius
        t = np.radians(theta_degrees)

        # FK
        x = r * np.sin(t)
        y = r * (1 - np.cos(t))
        return x, y

    def main(self):
        hm.HelloNode.main(
            self,
            "stretch_with_stretch_node",
            "node_namespace",
            wait_for_first_pointcloud=False,
        )

        # print("go")
        # angle = hm.angle_diff_rad(0, np.deg2rad(10))
        # at_goal = self.move_base.turn(angle, publish_visualizations=False)
        # at_goal = self.move_base.forward(-0.5, detect_obstacles=False)
        # print(at_goal)
        # rospy.sleep(1.0)

        rate = rospy.Rate(self.rate)

        # wait for initialization to complete
        while self.joint_states is None:
            rate.sleep()

        # move to calibration pose (early)
        self.move_to_pose(self.calibration_pose)

        rospy.loginfo("Waiting 5 more seconds...")
        rospy.sleep(5)  # give ros nodes time to initialize

        rospy.loginfo("Initialization completed.")
        self.notify_publisher.publish(True)  # notify

        # wait for calibration to complete
        self.wait_for_calibration_handshake()

        rospy.loginfo("Calibration completed.")
        self.notify_publisher.publish(True)  # notify

        while not rospy.is_shutdown():
            # wait for an exercise to be selected
            self.wait_for_exercise_start()

            # wait for an exercise to be completed
            self.wait_for_exercise_stop()


if __name__ == "__main__":
    rospy.loginfo("stretch_with_stretch::__init__()")
    node = stretch_with_stretch()
    node.main()
