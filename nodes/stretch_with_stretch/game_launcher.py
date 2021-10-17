#!/usr/bin/env python2
from __future__ import print_function

# import exercise menu selection
import menu_select

# ROS
import rospy

# Messages
from std_msgs.msg import Bool


class GameLauncher:
    def __init__(self):
        rospy.init_node("game_starter_node", anonymous=True)

        # Publishers
        self.start_game_publisher = rospy.Publisher(
            "/game_state/start_game", Bool, queue_size=1
        )

        # Subscribers
        self.robot_calibrated_subscriber = rospy.Subscriber(
            "/game_state/robot_calibrated", Bool, self.robot_calibrated_callback
        )

        # State
        self.robot_calibrated = False

    def robot_calibrated_callback(self, data):
        self.robot_calibrated = True

    def main(self):
        while not rospy.is_shutdown():
            if self.robot_calibrated:
                break

        while not rospy.is_shutdown():
            gameChoice = menu_select.get_user_input()
            if gameChoice == 1:
                rospy.loginfo("Exercise A is ready to begin.")
                self.start_game_publisher.publish(True)
                break
            else:
                rospy.loginfo("Exercise is not yet implemented.")


if __name__ == "__main__":
    node = GameLauncher()
    node.main()
