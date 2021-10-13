#!/usr/bin/env python2
# Generic Imports
from __future__ import print_function
import threading
import numpy as np

# ROS Stuff
import rospy

# Messages
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

# Stretch Imports
import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv

class stretch_with_stretch(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

        # Rate for rospy.Rate() called in main
        self.rate = 20

        # Subscribers
        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_state_callback)

        # Publishers
        self.wrist_contact_publisher = rospy.Publisher('/wrist_contact_detected', Bool, queue_size=1)

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
        self.wrist_yaw_effort_contact_threshold = 0.35 # N
        self.lift_contact_upper_threshold = 52 # N?
        self.lift_contact_lower_threshold = 36 # N?
        self.arm_contact_upper_threshold = 10 # N?
        self.arm_contact_lower_threshold = -10 # N?

        # Exercise Configuration
        self.calibration_lift_height = 0.75 # m
        self.calibration_arm_extension = 0 # m
        self.exercise_radius = 0.635 # m (average human arm length)

    def joint_state_callback(self, joint_states):
        # Update Joint State
        with self.joint_states_lock:
            self.joint_states = joint_states
        
        # Unpack Joint State
        lift_position, lift_velocity, lift_effort = hm.get_lift_state(joint_states)
        wrist_position, wrist_velocity, wrist_effort = hm.get_wrist_state(joint_states)

        self.wrist_yaw_effort = joint_states.effort[joint_states.name.index('joint_wrist_yaw')]
        self.lift_effort = joint_states.effort[joint_states.name.index('joint_lift')]
        self.arm_effort = joint_states.effort[joint_states.name.index('wrist_extension')]

        # Store necessary items
        self.lift_position = lift_position
        self.wrist_position = wrist_position

    def check_for_wrist_contact(self):
        if self.wrist_yaw_effort is not None:
            # rospy.loginfo("Current Wrist Yaw Effort: %f" % self.wrist_yaw_effort)

            bool_wrist_contact = False

            if abs(self.wrist_yaw_effort) > self.wrist_yaw_effort_contact_threshold:
                bool_wrist_contact = True

            if self.lift_effort < self.lift_contact_lower_threshold or self.lift_effort > self.lift_contact_upper_threshold:
                bool_wrist_contact = True

            if self.arm_effort < self.arm_contact_lower_threshold or self.arm_effort > self.arm_contact_upper_threshold:
                bool_wrist_contact = True

            self.wrist_contact_publisher.publish(bool_wrist_contact)

    def wait_for_calibration_handshake(self):
        rate = rospy.Rate(self.rate)

        calibration_pose = {"joint_lift": self.calibration_lift_height, "wrist_extension": self.calibration_arm_extension}
        self.move_to_pose(calibration_pose)
        while not rospy.is_shutdown():
            if self.wrist_yaw_effort is not None:
                self.check_for_wrist_contact()

            rate.sleep()

        rospy.loginfo("Calibration Handshake Completed.")

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
        hm.HelloNode.main(self, 'stretch_with_stretch_node', 'node_namespace', wait_for_first_pointcloud=False)
        rate = rospy.Rate(self.rate)

        while self.joint_states is None:
            pass

        self.move_to_pose({"joint_lift": self.lift_position, "wrist_extension": self.wrist_position})

        while not rospy.is_shutdown():
            if self.wrist_yaw_effort is not None:
                self.check_for_wrist_contact()

            rate.sleep()

if __name__ == '__main__':
    rospy.loginfo("stretch_with_stretch::__init__()")
    node = stretch_with_stretch()
    node.main()