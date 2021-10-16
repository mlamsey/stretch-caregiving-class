def print_menu():
    print(" ")
    print("===== EXERCISE SELECTION MENU =====")
    print(" ")
    print("Please select an exercise.")
    print("(A) Exercise 1    (B) Exercise 2    (C) Exercise 3")
    #print("(Q) to quit")
    print(" ")


def parse_menu_selection(user_selection_string):
    user_selection_string = user_selection_string.upper()

    if len(user_selection_string) > 1:
        print("Input too long!")
        return None

    # Select Exercise
    if user_selection_string == 'A':
        print("Exercise 1 Selected")
        return 1
    elif user_selection_string == 'B':
        print("Exercise 2 Selected")
        return 2
    elif user_selection_string == 'C':
        print("Exercise 3 Selected")
        return 3
    #elif user_selection_string == 'Q':
        #print("Quitting...")
        #return -1
    else:
        print("Invalid exercise, please try again.")
        return None


def get_user_input():
    input_confirmed = False
    while not input_confirmed:
        print_menu()
        user_input = input("Please enter a selection.  ")
        user_input = user_input.replace(" ","")
        decision = input("You have entered %s, enter y/Y to proceed.  " % user_input)
        if decision.upper() == 'Y':
            gameChoice = parse_menu_selection(user_input)
            if gameChoice is not None:
                input_confirmed = True
    return gameChoice