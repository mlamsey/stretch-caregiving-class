# from _typeshed import Self
import rospy

# class menudraft:
    # def __init__(self):
       #  rospy.init_node('menudraft', anonymous=True)
       # self.rate = 10

def print_menu():
    rospy.loginfo(" ")
    rospy.loginfo("===== EXERCISE SELECTION MENU =====")
    rospy.loginfo(" ")
    rospy.loginfo("Please select an exercise.")
    rospy.loginfo("(A) Exercise 1    (B) Exercise 2    (C) Exercise 3")
    #rospy.loginfo("(Q) to quit")
    rospy.loginfo(" ")


def parse_menu_selection(user_selection_string):
    user_selection_string = user_selection_string.upper()

    if len(user_selection_string) > 1:
        rospy.loginfo("Input too long!")
        return None

    # Select Exercise
    if user_selection_string == 'A':
        rospy.loginfo("Exercise 1 Selected")
        return 1
    elif user_selection_string == 'B':
        rospy.loginfo("Exercise 2 Selected")
        return 2
    elif user_selection_string == 'C':
        rospy.loginfo("Exercise 3 Selected")
        return 3
    #elif user_selection_string == 'Q':
        #rospy.loginfo("Quitting...")
        #return -1
    else:
        rospy.loginfo("Invalid exercise, please try again.")
        return None


def get_user_input():
    input_confirmed = False
    while not input_confirmed:
        print_menu()
        user_input = raw_input("Please enter a selection.  ")
        #rospy.sleep(5)
        user_input = user_input.replace(" ","")
        decision = raw_input("You have entered %s, enter y/Y to proceed.  " % user_input)
        #rospy.sleep(5)
        if decision.upper() == 'Y':
            gameChoice = parse_menu_selection(user_input)
            if gameChoice is not None:
                input_confirmed = True
    return gameChoice

if __name__ == '__main__':
    gameChoice = get_user_input()
    print(gameChoice)

    # def main(self):
       # rospy.spin()
       
#if __name__ == '__main__':
 #   node = menudraft()
  #  node.main()