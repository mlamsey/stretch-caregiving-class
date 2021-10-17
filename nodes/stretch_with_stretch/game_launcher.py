#!/usr/bin/env python2
from __future__ import print_function

# ROS
import rospy

# Messages
from std_msgs.msg import Bool, String

from menu import get_user_input


class GameLauncher:
    def __init__(self):
        rospy.init_node("game_starter_node", anonymous=True)

        # Subscribers
        self.sws_ready_subscriber = rospy.Subscriber(
            "/sws_ready", Bool, self.sws_ready_callback
        )
        self.sws_ready = False

        # Publishers
        self.select_exercise_publisher = rospy.Publisher(
            "/sws_select_exercise", String, queue_size=1
        )

    def sws_ready_callback(self, data):
        if data.data:
            self.sws_ready = True

    def main(self):
        # wait for stretch with stretch to be ready
        while not rospy.is_shutdown():
            if self.sws_ready:
                break

        if not rospy.is_shutdown():
            selection = get_user_input()
            rospy.loginfo("Exercise {} is ready to begin.".format(selection))
            self.select_exercise_publisher.publish(selection)


if __name__ == "__main__":
    node = GameLauncher()
    node.main()
