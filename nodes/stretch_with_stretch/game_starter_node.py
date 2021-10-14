#!/usr/bin/env python2
from __future__ import print_function

# ROS
import rospy

# Messages
from std_msgs.msg import Bool

class GameStarterNode:
    def __init__(self):
        rospy.init_node("game_starter_node", anonymous=True)

        # Publishers
        self.start_game_publisher = rospy.Publisher('/game_state/start_game', Bool, queue_size=1)

        # Subscribers
        self.robot_calibrated_subscriber = rospy.Subscriber('/game_state/robot_calibrated', Bool, self.robot_calibrated_callback)
        
        # State
        self.robot_calibrated = False

    def robot_calibrated_callback(self, data):
        self.robot_calibrated = True

    def main(self):
        # wait for handshake
        while not self.robot_calibrated:
            pass

        # launch
        rospy.loginfo("AUTOMATICALLY LAUNCHING GAME")
        self.start_game_publisher.publish(True)
        rospy.spin()

if __name__ == '__main__':
    node = GameStarterNode()
    node.main()