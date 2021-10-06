#!/usr/bin/env python2
from __future__ import print_function

# ROS
import rospy

# Messages
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

class stretch_imu_monitor:
    def __init__(self):
        rospy.init_node('stretch_imu_monitor', anonymous=True)
        # Misc
        self.rate = 1

        # Subscribers
        self.imu_subscriber = rospy.Subscriber('/imu_wrist', Imu, self.imu_callback)

        # Data Storage
        self.imu_x = None
        self.imu_y = None
        self.imu_z = None

    def imu_callback(self, data):
        linear_acceleration = data.linear_acceleration  # Vector3
        self.imu_x = linear_acceleration.x
        self.imu_y = linear_acceleration.y
        self.imu_z = linear_acceleration.z

    def main(self):
        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            if self.imu_x is not None:
                rospy.loginfo("IMU X: %f" % self.imu_x)

            rate.sleep()

if __name__ == '__main__':
    node = stretch_imu_monitor()
    node.main()