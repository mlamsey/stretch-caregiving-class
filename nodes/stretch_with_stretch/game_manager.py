#!/usr/bin/env python2
# Generic Imports
from __future__ import print_function

# ROS
import rospy

# import filewritetest
from game_logger import GameLogger, LogState

# Messages
from std_msgs.msg import Bool, String, UInt8


class GameManager:
    def __init__(self):
        rospy.init_node("reach_point_game_manager", anonymous=True)
        self.rate = 10

        # Subscribers
        self.start_exercise_subscriber = rospy.Subscriber(
            "/sws_start_exercise", String, self.start_exercise_callback
        )
        self.stop_exercise_subscriber = rospy.Subscriber(
            "/sws_stop_exercise", Bool, self.stop_exercise_callback
        )

        self.wrist_contact_subscriber = rospy.Subscriber(
            "/sws_contact_detected", Bool, self.wrist_contact_callback
        )

        # Publishers
        self.point_scored_publisher = rospy.Publisher(
            "/sws_point_scored", Bool, queue_size=1
        )
        self.nod_head_publisher = rospy.Publisher(
            "/sws_nod_head", UInt8, queue_size=1
        )
        self.announce_score_publisher = rospy.Publisher(
            "/sws_announce_score", UInt8, queue_size=1
        )

        # State
        self.current_exercise = None
        self.contact_detected = False
        self.score = 0

        # Logger
        self.logger = GameLogger()

    def point_scored(self):
        rospy.loginfo("Point Scored!")
        self.score += 1
        self.point_scored_publisher.publish(True)
        self.logger.add_line(self.current_exercise, LogState.CONTACT_DETECTED)
        rospy.loginfo("Exercise score: {}".format(self.score))
        rospy.sleep(0.75)

    def start_exercise_callback(self, data):
        self.current_exercise = data.data
        self.score = 0

    def stop_exercise_callback(self, data):
        self.current_exercise = None
        rospy.loginfo("Exercise done: {} points scored!".format(self.score))
        self.announce_score_publisher.publish(self.score)
        if self.score > 5:
            self.nod_head_publisher.publish(5)
        elif self.score > 0:
            self.nod_head_publisher.publish(2)
        self.score = 0

    def wrist_contact_callback(self, data):
        self.contact_detected = data.data

    def main(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.current_exercise is not None:
                if self.contact_detected:
                    self.point_scored()
            rate.sleep()


if __name__ == "__main__":
    node = GameManager()
    node.main()
