#!/usr/bin/env python2
# Generic Imports
from __future__ import print_function
import threading

# ROS Stuff
import rospy

# Messages
from sensor_msgs.msg import JointState

# Stretch Imports
import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv

class stretch_with_stretch(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

        # Rate for rospy.Rate() called in main
        self.rate = 1

        # Subscribers
        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_state_callback)

        # Joint State Inits
        self.joint_states = None
        self.lift_position = None
        self.wrist_position = None

        # Threading for callbacks
        self.joint_states_lock = threading.Lock()

    def joint_state_callback(self, joint_states):
        # Update Joint State
        with self.joint_states_lock:
            self.joint_states = joint_states
        
        # Unpack Joint State
        lift_position, lift_velocity, lift_effort = hm.get_lift_state(joint_states)
        wrist_position, wrist_velocity, wrist_effort = hm.get_wrist_state(joint_states)

        # Store necessary items
        self.lift_position = lift_position
        self.wrist_position = wrist_position

    def main(self):
        hm.HelloNode.main(self, 'stretch_with_stretch_node', 'node_namespace', wait_for_first_pointcloud=False)
        rate = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            if self.wrist_position is not None:
                rospy.loginfo("Current wrist extension: %f" % self.wrist_position)

            rate.sleep()

if __name__ == '__main__':
    rospy.loginfo("stretch_with_stretch::__init__()")
    node = stretch_with_stretch()
    node.main()