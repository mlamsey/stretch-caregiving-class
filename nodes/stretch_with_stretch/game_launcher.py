#!/usr/bin/env python2
from __future__ import print_function

# ROS
import rospy

# Messages
from std_msgs.msg import Bool

from menu import get_user_input


class GameLauncher:
    def __init__(self):
        rospy.init_node("game_starter_node", anonymous=True)

        # Publishers
        self.start_game_publisher = rospy.Publisher(
            "/game_state/start_game", Bool, queue_size=1
        )

    def main(self):
        while not rospy.is_shutdown():
            gameChoice = get_user_input()
            if gameChoice == 1:
                rospy.loginfo("Exercise A is ready to begin.")
                self.start_game_publisher.publish(True)
                break
            else:
                rospy.loginfo("Exercise is not yet implemented.")


if __name__ == "__main__":
    node = GameLauncher()
    node.main()
