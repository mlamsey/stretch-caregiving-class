import json
from struct import pack
import tkinter as tk
from tkinter import *
from exercise_interface import get_exercise_specification
import sys
###############################################################
#File I/O
def readFile(fileName):
    with open(fileName) as d:
        json.load(d)
        # data = d.readlines()
    # exerciseList = [json.loads(x) for x in data]
    

def writeFile(fileName, exercise_dict):
    with open(fileName, 'w') as json_file:
        json.dump(exercise_dict, json_file, indent=2)

###############################################################
#TKinter Window Output

# window = tk.Tk('Exercise Routine')
window = tk.Tk()
window.title('Stretch with Stretch: Configure Exercise')
window.n_elements_last_added = 0
  
#Styles
window.geometry('300x600')
window['background'] = '#E0EEC6'

#ADDING A SCROLLBAR
#window.myscrollbar=tk.Scrollbar(window,orient="vertical")
#window.myscrollbar.pack(side="right",fill="y")

#Storage Dict
durEntries = []
nameEntries = []
dirEntries = []
diffEntries = []
cognEntries = []
class_data = {
    "exercises": []
    
}

#####################################
#TKinter Functions
def createRoutine():
    
    original_exercises = class_data["exercises"]

    for i in range(len(durEntries)):
        # print(nameEntries[i].get())
        # print(durEntries[i].get())
        #create new ex  
        #print(i, " :",nameEntries[i], " ",nameEntries[i].get())
        new_exercise = get_exercise_specification(
            name=nameEntries[i].get(), direction=dirEntries[i].get(), difficulty=diffEntries[i].get(), duration=float(durEntries[i].get()), cognitive=cognEntries[i].get()==1
            )
        
        #add ex to routine
        print(new_exercise)
        original_exercises.append(new_exercise)
    
    go_home = get_exercise_specification("home")
    original_exercises.append(go_home)

    class_data["exercises"] = original_exercises

    #output routine to text File
    output_path = window.output_path.get()

    if output_path == "":
        output_path = "out.txt"
    
    if output_path[-4:] != ".txt":
        output_path += ".txt"

    print("Writing file to " + output_path)
    writeFile(output_path, class_data)
    readFile(output_path)

    #clear fields
    delete_entries()
    emptyClassData()


def emptyClassData():
    class_data["exercises"] = []


def create_top_menu():
    #Create Buttons
    button_add_exercise = tk.Button(window, text="Add New Exercise", highlightbackground="#E0EEC6", command=addNewEx)
    button_add_exercise.pack()
    button_submit_routine = tk.Button(window, text="Submit Routine", highlightbackground="#E0EEC6", command=createRoutine)
    button_submit_routine.pack()
    button_remove_exercise = tk.Button(window, text="Remove Exercise", highlightbackground="#E0EEC6", command=remove_exercises)
    button_remove_exercise.pack()

    # Create file name
    path_label = tk.Label(window, text="Output File Path:", bg="#E0EEC6", font='helvetica 16')
    path_label.pack()
    window.output_path = tk.Entry(window, width=20)
    window.output_path.pack()
    window.output_path.config(highlightbackground="#E0EEC6", relief="solid", borderwidth=1, font='helvetica 16')
    
    ##INITIAL FIELDS##
    title = tk.Label(window, text="Configure Exercise", bg="#E0EEC6", font=('helvetica bold', 20))
    title.pack()
    

def addNewEx(label_bg="#E0EEC6"):
    # get current number of UI elements (used at end)
    previous_n_elements = len(window.pack_slaves())

    # Label Elements
    exercise_label = tk.Label(window, text="Exercise", bg=label_bg, font='helvetica 16')
    exercise_label.pack()

    # Exercise Type Selection Menu
    exercise_type_selection = tk.StringVar(window)
    exercise_type_selection.set("Sit and Reach") # default value
    window.menu = tk.OptionMenu(window, exercise_type_selection, "Sit and Reach", "Sit and Kick", "Stand and Reach")
    window.menu.pack()
    window.menu.config(highlightbackground='#FFFFFF', bg='#FFFFFF', font='helvetica 16')
    
    nameEntries.append(exercise_type_selection)

    # Left / Right Selector
    lr_label = tk.Label(window, text="Direction", bg=label_bg, font='helvetica 16')
    lr_label.pack()
    lr_selection=tk.StringVar(window)
    lr_selection.set("Left") #default value
    button_left = tk.Radiobutton(window, text="Left",variable=lr_selection, value="Left")
    button_left.pack()
    button_right = tk.Radiobutton(window, text="Right", variable=lr_selection, value="Right")
    button_right.pack()

    dirEntries.append(lr_selection)

    # Difficulty Selector
    difficulty_label = tk.Label(window, text="Difficulty", bg=label_bg, font='helvetica 16')
    difficulty_label.pack()
    difficulty_selection = tk.StringVar(window)
    difficulty_selection.set("Medium") #default value
    button_easy = tk.Radiobutton(window, text="Easy", variable=difficulty_selection, value="Easy", padx=11)
    button_easy.pack()
    button_medium = tk.Radiobutton(window, text="Medium", variable=difficulty_selection, value="Medium")
    button_medium.pack()
    button_hard = tk.Radiobutton(window, text="Hard", variable=difficulty_selection, value="Hard", padx=11)
    button_hard.pack()

    diffEntries.append(difficulty_selection)

    # Duration Selector
    duration_label = tk.Label(window, text="Duration (s)", bg=label_bg, font='helvetica 16')
    duration_label.pack()
    window.txt = tk.Entry(window, width=10)
    window.txt.pack()
    window.txt.config(highlightbackground=label_bg, relief="solid", borderwidth=1, font='helvetica 16')
    
    durEntries.append(window.txt)

    # Cognitive Exercise Toggle
    cognitive_selection = tk.IntVar()
    button_cognitive = tk.Checkbutton(window, text="Include Cognitive Exercise in Routine",variable=cognitive_selection)
    button_cognitive.pack()
    cognEntries.append(cognitive_selection)

    # save number of elements added to UI
    new_n_elements = len(window.pack_slaves())
    n_elements_added = new_n_elements - previous_n_elements
    window.n_elements_last_added = n_elements_added
    
def remove_exercises():
    if len(durEntries) > 1:
        del durEntries[-1]
        del nameEntries[-1]
        packList = window.pack_slaves()
        n_elements = len(packList)

        n_elements_to_remove = window.n_elements_last_added

        if n_elements_to_remove > 0:
            for x in range(n_elements_to_remove):
                i = n_elements - x - 1
                packList[i].pack_forget()

def delete_entries():
    for field in durEntries:
        field.delete(0,END)
    for field in nameEntries:
        field.set("Sit and Reach")
    

# Main
create_top_menu()
addNewEx()
window.mainloop()
