import rospy

menus = ["MAIN", "EXERCISE"]

# print main menu
def print_main_menu():
    rospy.loginfo(" ")
    rospy.loginfo("======== Stretch with Stretch MENU ========")
    rospy.loginfo(" ")
    rospy.loginfo("Please select an action.")
    rospy.loginfo("(M) Select Exercise    (J) Load JSON    (Q) Quit")
    rospy.loginfo(" ")


# print exercise selection menu
def print_exercise_menu():
    rospy.loginfo(" ")
    rospy.loginfo("======== EXERCISE SELECTION MENU ========")
    rospy.loginfo(" ")
    rospy.loginfo("Please select an exercise.")
    rospy.loginfo("(A) Sit-Reach R     (B) Sit-Kick R     (C) Reach-Hold R")
    rospy.loginfo(" ")


# defining menu selection
def main_menu_selection(menu_select):
    menu_select = menu_select.upper()

    if len(menu_select) > 1:
        rospy.loginfo("Input too long!")
        return None
    
    if menu_select not in ["M", "J", "Q"]:
        rospy.loginfo("Selection {} not in menu.".format(menu_select))
        return None

    rospy.loginfo("{} selected.".format(menu_select))
    return menu_select


def exercise_menu_selection(ex_select):
    ex_select = ex_select.upper()

    if len(ex_select) > 1:
        rospy.loginfo("Input too long!")
        return None

    if ex_select not in ["A", "B", "C"]:
        rospy.loginfo("Exercise {} is not yet implemented".format(ex_select))
        return None

    rospy.loginfo("Exercise {} selected".format(ex_select))
    return ex_select


def get_user_input_with_confirmation(menu_name):
    menu_name = menu_name.upper()
    if menu_name not in menus:
        rospy.loginfo("menu::get_user_input_with_confirmation: invalid menu name.")
        return None

    if rospy.is_shutdown():
        return None

    input_confirmed = False
    while not input_confirmed:
        # init
        choice = None

        # print menu
        if menu_name == "MAIN":
            print_main_menu()
        elif menu_name == "EXERCISE":
            print_exercise_menu()

        # get input
        msg = "Please enter a selection.      "
        user_input = raw_input(msg).upper()
        user_input = user_input.replace(" ", "")

        msg = "You have selected {}\n".format(user_input)
        msg += "Is this correct? Press y/Y to proceed.      "
        # decision = raw_input(msg)
        decision = "Y"  # for faster debugging

        if decision.upper() == "Y":
            if menu_name == "MAIN":
                choice = main_menu_selection(user_input)
            elif menu_name == "EXERCISE":
                choice = exercise_menu_selection(user_input)

            if choice is not None:
                input_confirmed = True

    return choice
