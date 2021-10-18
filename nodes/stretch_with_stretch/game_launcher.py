#!/usr/bin/env python2
from __future__ import print_function

# ROS
import rospy
from menu import get_user_input

# Messages
from std_msgs.msg import Bool, String


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
            if not self.sws_ready:
                continue

            selection = get_user_input()
            if selection is None:
                break

            rospy.loginfo("Exercise {} is ready to begin.".format(selection))
            self.select_exercise_publisher.publish(selection)

            rospy.sleep(5)  # wait for exercies to end
            self.sws_ready = False


if __name__ == "__main__":
    node = GameLauncher()
    node.main()
