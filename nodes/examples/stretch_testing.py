#!/usr/bin/env python2
# Generic Imports
from __future__ import print_function

# ROS Stuff
import rospy

# Stretch Imports
import hello_helpers.hello_misc as hm

class stretch_testing(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(self, 'stretch_test_node', 'node_namespace', wait_for_first_pointcloud=False)
        self.move_to_pose({"joint_lift": 0.6})
        rospy.sleep(1)
        self.move_to_pose({"joint_lift": 0.4})
        rospy.sleep(1)
        rospy.loginfo("Test Complete!")

if __name__ == '__main__':
    print("stretch_testing::__init__()")
    node = stretch_testing()
    node.main()