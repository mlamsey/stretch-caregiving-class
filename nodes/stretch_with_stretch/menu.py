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

    if ex_select not in ["A", "B", "C"]:
        rospy.loginfo("Exercise {} is not yet implemented".format(ex_select))
        return None

    rospy.loginfo("Exercise {} selected".format(ex_select))
    return ex_select


def get_user_input():
    if rospy.is_shutdown():
        return None

    input_confirmed = False
    while not input_confirmed:
        print_menu()
        msg = "Please enter a selection.      "
        user_input = raw_input(msg)
        user_input = user_input.replace(" ", "")
        if user_input.upper() == "Q":
            return None

        msg = "You have selected {}\n".format(user_input)
        msg += "Is this correct? Press y/Y to proceed.      "
        # decision = raw_input(msg)
        decision = "Y"  # for faster debugging
        if decision.upper() == "Q":
            return None
        elif decision.upper() == "Y":
            gameChoice = menu_selection(user_input)
            if gameChoice is not None:
                input_confirmed = True

    return gameChoice
