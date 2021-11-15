#!/usr/bin/env python2
from __future__ import print_function

import rospy


class basic_ros_node:
    def __init__(self):
        self.some_parameter = 2

    def main(self):
        while not rospy.is_shutdown():
            rospy.loginfo("basic_ros_node::main()")
            print("My test parameter value is ", self.some_parameter)
            rospy.sleep(1)  # sleep for 1 second


if __name__ == "__main__":
    print("basic_ros_node_class::__init__()")
    instance_of_this_node = basic_ros_node()
    instance_of_this_node.main()
