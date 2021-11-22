#!/usr/bin/env python2
# Generic Imports
from __future__ import print_function

import threading

# Stretch Imports
import hello_helpers.hello_misc as hm
import numpy as np
import stretch_gestures

# ROS Stuff
import rospy
import stretch_funmap.navigate as nv
from sensor_msgs.msg import JointState

# Messages
from std_msgs.msg import Bool, String, UInt8, Int8, Float32


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
        self.nod_head_subscriber = rospy.Subscriber(
            "/sws_nod_head", UInt8, self.nod_head_callback
        )

        self.unique_color_subscriber = rospy.Subscriber(
            "/speech/n_unique_colors", Int8, self.unique_colors_callback
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

        self.speech_recognition_publisher = rospy.Publisher(
            "/speech/start_recording", Float32, queue_size=1
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
        self.pre_calibration_pose = {
            "joint_lift": 0.7,  # m
            "wrist_extension": 0.05,  # m
            "joint_wrist_yaw": 0.0,  # rad
            "joint_gripper_finger_left": 0.5,
            "joint_head_pan": np.deg2rad(-90.0),  # rad
            "joint_head_tilt": -0.2,  # rad
        }
        self.post_calibration_pose = self.pre_calibration_pose.copy()
        self.post_calibration_pose["joint_gripper_finger_left"] = 0.05
        self.calibration_xya = None
        self.exercise_radius = 0.635  # m (average human arm length)

        # Gestures
        self.stretch_gestures = stretch_gestures.StretchGestures(self.move_to_pose)

        # callback data
        self.n_unique_colors = None

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

    def nod_head_callback(self, data):
        self.stretch_gestures.nod(data.data)

    def unique_colors_callback(self, data):
        self.n_unique_colors = data.data

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

    def goto_exercise_position(self):
        rospy.loginfo("Repositioning for exercise {}...".format(self.current_exercise))

        current_xya, _ = self.get_robot_floor_pose_xya()

        # get offsets from calibration xya
        delta_x = self.calibration_xya[0] - current_xya[0]
        delta_a = self.calibration_xya[2] - current_xya[2]

        # get lift height
        joint_lift = 0.8  # m
        if self.current_exercise == "C":
            joint_lift = self.post_calibration_pose["joint_lift"]

        # get target position
        target_x = 0.0
        if self.current_exercise == "A":  # go to the right
            target_x = self.exercise_radius / 2
        elif self.current_exercise == "B":  # go to the left
            target_x = -1 * self.exercise_radius / 2

        # get target angle
        target_a = None
        if self.current_exercise == "A":  # go to the right
            target_a = np.deg2rad(30.0)
        elif self.current_exercise == "B":  # go to the left
            target_a = np.deg2rad(-30.0)

        # turn back to calibration angle
        at_goal = self.move_base.turn(delta_a, publish_visualizations=False)

        # move base
        at_goal = self.move_base.forward(delta_x + target_x, detect_obstacles=False)

        # turn to exercise angle
        if target_a is not None:
            at_goal = self.move_base.turn(target_a, publish_visualizations=False)

        # reposition wrist
        wrist_extension = self.post_calibration_pose["wrist_extension"]
        if self.current_exercise != "C":
            wrist_extension += 0.5  # exted the arm
        self.move_to_pose(
            {
                "joint_lift": joint_lift,
                "wrist_extension": wrist_extension,
            },
            async=False,
        )

        rospy.loginfo(
            "Repositioning for exercise {}... done!".format(self.current_exercise)
        )

    def goto_rest_position(self):
        rospy.loginfo("Returning to rest position...")

        current_xya, _ = self.get_robot_floor_pose_xya()

        # get offset from calibration xya
        delta_a = self.calibration_xya[2] - current_xya[2]

        # reposition wrist
        wrist_extension = self.post_calibration_pose["wrist_extension"]
        self.move_to_pose({"wrist_extension": wrist_extension}, async=False)

        # turn back to calibration angle
        at_goal = self.move_base.turn(delta_a, publish_visualizations=False)

        rospy.loginfo("Returning to rest position... done!")

    def wait_for_calibration_handshake(self):
        rate = rospy.Rate(self.rate)

        self.move_to_pose(self.pre_calibration_pose)
        rospy.sleep(2)  # give robot time to settle

        rospy.loginfo("*" * 40)
        rospy.loginfo("Give the robot the ball to start the game!")
        rospy.loginfo("*" * 40)

        while not rospy.is_shutdown():
            self.sws_ready_publisher.publish(False)
            if self.check_for_wrist_contact(publish=False):
                break
            rate.sleep()

        if rospy.is_shutdown():
            return

        self.move_to_pose(self.post_calibration_pose)
        rospy.sleep(2)  # give robot time to settle

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

        # reposition robot
        self.goto_exercise_position()

        # exercise C returns the robot to the calbiration xya
        if self.current_exercise == "C":
            self.current_exercise = None
            return

        # start exercise
        rospy.loginfo("Exercise {} starting...".format(self.current_exercise))
        self.sws_start_exercise_publisher.publish(self.current_exercise)

    def wait_for_exercise_stop(self):
        # exercise is already over (e.g., exercise C is a "rest")
        if self.current_exercise is None:
            return

        rate = rospy.Rate(self.rate)

        extra, duration = 3, 8
        start_time = rospy.Time.now().secs
        # give a couple extra seconds for the startup sound
        delay_time = start_time + extra
        stop_time = delay_time + duration + extra

        # start audio recording
        self.speech_recognition_publisher.publish(duration + duration)

        while not rospy.is_shutdown():
            self.sws_ready_publisher.publish(False)
            self.check_for_wrist_contact(publish=True)

            now = rospy.Time.now().secs
            if now > stop_time:
                break
            elif now > delay_time:
                # movement
                pos = self.wrist_position - 0.01
                self.move_to_pose({"wrist_extension": pos}, async=True)

            rate.sleep()

        if rospy.is_shutdown():
            return

        # stop exercise
        self.current_exercise = None

        self.notify_publisher.publish(True)  # notify

        rospy.loginfo("Exercise {} complete!".format(self.current_exercise))
        self.sws_stop_exercise_publisher.publish(True)

        rospy.sleep(2)  # wait for nod

        self.goto_rest_position()

    def main(self):
        hm.HelloNode.main(
            self,
            "stretch_with_stretch_node",
            "node_namespace",
            wait_for_first_pointcloud=False,
        )

        rate = rospy.Rate(self.rate)

        # wait for initialization to complete
        while self.joint_states is None:
            rate.sleep()

        # move to calibration pose (early)
        self.move_to_pose(self.pre_calibration_pose)

        stop_wait_time = rospy.Time.now() + rospy.Duration.from_sec(10.0)
        while self.check_for_wrist_contact():
            rospy.sleep(1)
            if rospy.Time.now() > stop_wait_time:
                print("Warning: waited too long :(")
                break

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
