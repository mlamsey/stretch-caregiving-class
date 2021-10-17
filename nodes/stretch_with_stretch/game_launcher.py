#!/usr/bin/env python2
from __future__ import print_function

# ROS
import rospy

# Messages
from std_msgs.msg import Bool


# print exercise selection menu
def print_menu():
    rospy.loginfo(" ")
    rospy.loginfo("======== EXERCISE SELECTION MENU ========")
    rospy.loginfo(" ")
    rospy.loginfo("Please select an exercise.")
    rospy.loginfo("(A) Exercise A     (B) Exercise B     (C) Exercise C")
    rospy.loginfo(" ")


# defining menu selection
def menu_selection(ex_select):
    ex_select = ex_select.upper()

    if len(ex_select) > 1:
        rospy.loginfo("Input too long!")
        return None

    # select exercise
    if ex_select == "A":
        rospy.loginfo("Exercise A Selected")
        return 1
    elif ex_select == "B":
        rospy.loginfo("Exercise B Selected")
        return 2
    elif ex_select == "C":
        rospy.loginfo("Exercise C Selected")
        return 3
    else:
        rospy.loginfo("Invalid exercise, please try again.")
        return None


def get_user_input():
    input_confirmed = False
    while not input_confirmed:
        # print("Test")
        print_menu()
        user_input = raw_input("Please enter a selection.      ")
        user_input = user_input.replace(" ", "")
        decision = raw_input(
            "You have selected %s. Is this correct? Press y/Y to proceed.      "
            % user_input
        )
        if decision.upper() == "Y":
            gameChoice = menu_selection(user_input)
            if gameChoice is not None:
                input_confirmed = True
    return gameChoice


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
