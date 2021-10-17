# rospy
import rospy

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
        print_menu()
        user_input = raw_input("Please enter a selection.      ")
        user_input = user_input.replace(" ",'')
        decision = raw_input("You have selected %s. Is this correct? Press y/Y to proceed.      " % user_input)
        if decision.upper() == "Y":
            gameChoice = menu_selection(user_input)
            if gameChoice is not None:
                input_confirmed = True
    return gameChoice

if __name__ == '__main__':
    gameChoice = get_user_input()
    print(gameChoice)

########## COPY AND PASTE BELOW INTO game_starter_node.py TO UPDATE FOR THIS FILE ##########


#!/usr/bin/env python2
from __future__ import print_function

# ROS
import rospy


# import exercise menu selection
import menu_select


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

        # launch menu select
        if self.robot_calibrated:
            gameChoice = menu_select.get_user_input()
            if gameChoice == 1:
                rospy.loginfo("Exercise 1 is ready to begin.")
                #launch game
                rospy.loginfo("AUTOMATICALLY LAUNCHING GAME")
                self.start_game_publisher.publish(True)
                rospy.spin()
            else:
                gameChoice = menu_select.get_user_input()

if __name__ == '__main__':
    node = GameStarterNode()
    node.main()