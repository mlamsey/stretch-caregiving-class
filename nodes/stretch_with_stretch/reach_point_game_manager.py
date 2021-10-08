#!/usr/bin/env python2
# Generic Imports
from __future__ import print_function
import ros

# ROS
import rospy

# Messages
from std_msgs.msg import Bool

class reach_point_game_manager:
    def __init__(self):
        rospy.init_node('reach_point_game_manager', anonymous=True)
        self.rate = 10

        self.wrist_contact = False
        self.wrist_contact_subscriber = rospy.Subscriber('/wrist_contact_detected', Bool, self.wrist_contact_callback)

        self.point_scored_publisher = rospy.Publisher('/game_state/point_scored', Bool, queue_size=1)
        self.start_game_publisher = rospy.Publisher('/game_state/start_game', Bool, queue_size=1)

        self.game_score = 0

    def wrist_contact_callback(self, data):
        self.wrist_contact = data.data

    def point_scored_action(self):
        rospy.loginfo("Point Scored!")
        self.game_score += 1
        self.point_scored_publisher.publish(True)
        rospy.sleep(0.5)

    def main(self):
        rate = rospy.Rate(self.rate)

        rospy.sleep(3)
        self.start_game_publisher.publish(True)

        # Game state
        score_changed = False

        while not rospy.is_shutdown():
            if self.wrist_contact:
                self.point_scored_action()
                score_changed = True

            if score_changed:
                rospy.loginfo(self.game_score)
                score_changed = False

            rate.sleep()

if __name__ == '__main__':
    node = reach_point_game_manager()
    node.main()