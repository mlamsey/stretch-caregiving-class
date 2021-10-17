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
        self.notify_publisher = rospy.Publisher(
            "/sws_notify", Bool, queue_size=1
        )
        self.sws_ready_publisher = rospy.Publisher(
            "/sws_ready", Bool, queue_size=1
        )
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

        self.arm_effort = None

        self.wrist_position = None
        self.wrist_yaw_effort = None

        # Threading for callbacks
        self.joint_states_lock = threading.Lock()

        # Internal variables
        self.wrist_yaw_effort_contact_threshold = 0.35  # N
        self.lift_contact_upper_threshold = 53  # N?
        self.lift_contact_lower_threshold = 35  # N?
        self.arm_contact_upper_threshold = 10  # N?
        self.arm_contact_lower_threshold = -10  # N?

        # Exercise Init
        self.current_exercise = None

        # Exercise Configuration
        self.calibration_lift_height = 0.75  # m
        self.calibration_arm_extension = 0.05  # m
        self.exercise_radius = 0.635  # m (average human arm length)


    def joint_state_callback(self, joint_states):
        # Update Joint State
        with self.joint_states_lock:
            self.joint_states = joint_states

        # Unpack Joint State
        lift_position, lift_velocity, lift_effort = hm.get_lift_state(joint_states)
        wrist_position, wrist_velocity, wrist_effort = hm.get_wrist_state(joint_states)

        self.wrist_yaw_effort = joint_states.effort[
            joint_states.name.index("joint_wrist_yaw")
        ]
        self.lift_effort = joint_states.effort[joint_states.name.index("joint_lift")]
        self.arm_effort = joint_states.effort[
            joint_states.name.index("wrist_extension")
        ]

        # Store necessary items
        self.lift_position = lift_position
        self.wrist_position = wrist_position
    
    def select_exercise_callback(self, data):
        self.current_exercise = data.data

    def check_for_wrist_contact(self, publish=False):
        if self.wrist_yaw_effort is not None:
            # rospy.loginfo("Current Wrist Yaw Effort: %f" % self.wrist_yaw_effort)

            bool_wrist_contact = False

            if abs(self.wrist_yaw_effort) > self.wrist_yaw_effort_contact_threshold:
                bool_wrist_contact = True

            if (
                self.lift_effort < self.lift_contact_lower_threshold
                or self.lift_effort > self.lift_contact_upper_threshold
            ):
                bool_wrist_contact = True

            if (
                self.arm_effort < self.arm_contact_lower_threshold
                or self.arm_effort > self.arm_contact_upper_threshold
            ):
                bool_wrist_contact = True

            if publish:
                self.wrist_contact_publisher.publish(bool_wrist_contact)

            return bool_wrist_contact

        return False
    
    def wait_for_calibration_handshake(self):
        rate = rospy.Rate(self.rate)

        calibration_pose = {
            "joint_lift": self.calibration_lift_height,
            "wrist_extension": self.calibration_arm_extension,
        }
        self.move_to_pose(calibration_pose)
        rospy.sleep(2)  # give robot time to settle

        rospy.loginfo("*" * 40)
        rospy.loginfo("Shake hands with the robot to start the game!")
        rospy.loginfo("*" * 40)

        while not rospy.is_shutdown():
            self.sws_ready_publisher.publish(False)
            if self.check_for_wrist_contact(publish=False):
                break
            rate.sleep()

        rospy.loginfo("Calibration handshake completed!")
    
    def wait_for_exercise_start(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.sws_ready_publisher.publish(True)
            if self.current_exercise is not None:
                break
            rate.sleep()
        
        # reposition robot
        # TODO: move robot

        # start exercise
        self.sws_start_exercise_publisher.publish(self.current_exercise)

    def wait_for_exercise_stop(self):
        rate = rospy.Rate(self.rate)

        duration = 15
        stop_time = rospy.Time.now().secs + duration
        while not rospy.is_shutdown():
            self.check_for_wrist_contact(publish=True)
            if rospy.Time.now().secs > stop_time:
                break
            rate.sleep()
        
        # stop exercise
        self.sws_stop_exercise_publisher.publish(True)
        self.current_exercise = None

        # TODO: change this to a different sound
        self.notify_publisher.publish(True)  # notify

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

        rate = rospy.Rate(self.rate)

        # wait for initialization to complete
        while self.joint_states is None:
            rate.sleep()

        # activate hold pose
        self.move_to_pose(
            {"joint_lift": self.lift_position, "wrist_extension": self.wrist_position}
        )

        rospy.loginfo("Waiting 5 more seconds...")
        rospy.sleep(5)  # give ros nodes time to initialize

        rospy.loginfo("Initialization Completed.")
        self.notify_publisher.publish(True)  # notify

        # wait for calibration to complete
        self.wait_for_calibration_handshake()

        while not rospy.is_shutdown():
            # wait for an exercise to be selected
            self.wait_for_exercise_start()

            # wait for an exercise to be completed
            self.wait_for_exercise_stop()


if __name__ == "__main__":
    rospy.loginfo("stretch_with_stretch::__init__()")
    node = stretch_with_stretch()
    node.main()
