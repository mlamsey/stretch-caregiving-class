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

        # Subscribers
        self.wrist_contact_subscriber = rospy.Subscriber('/wrist_contact_detected', Bool, self.wrist_contact_callback)
        self.start_game_subscriber = rospy.Subscriber('/game_state/start_game', Bool, self.start_game_callback)

        # Publishers
        self.point_scored_publisher = rospy.Publisher('/game_state/point_scored', Bool, queue_size=1)

        # State
        self.wrist_contact = False
        self.start_game = False
        self.game_score = 0

    def wrist_contact_callback(self, data):
        self.wrist_contact = data.data

    def start_game_callback(self, data):
        self.start_game = data.data

    def point_scored_action(self):
        rospy.loginfo("Point Scored!")
        self.game_score += 1
        self.point_scored_publisher.publish(True)
        rospy.sleep(0.75)

    def play_game(self):
        rate = rospy.Rate(self.rate)

        # Game configuration
        game_time = 30 # sec
        game_end_time = rospy.Time.now().secs + game_time

        # Game state
        score_changed = False

        # Run game
        rospy.sleep(1.5) # wait for startup sound
        while not rospy.is_shutdown() and rospy.Time.now().secs < game_end_time:
            if self.wrist_contact:
                self.point_scored_action()
                score_changed = True

            if score_changed:
                rospy.loginfo(self.game_score)
                score_changed = False

            rate.sleep()

        rospy.loginfo("Game Complete!")
        rospy.loginfo("Final Score: %i" % self.game_score)
        # TODO: game over sound

        # Reset game
        self.game_score = 0


    def main(self):
        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            if self.start_game:
                self.play_game()
                self.start_game = False

            rate.sleep()

if __name__ == '__main__':
    node = reach_point_game_manager()
    node.main()