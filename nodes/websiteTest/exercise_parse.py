import json
import tkinter as tk
from tkinter import *
###############################################################
#File I/O
def readFile(fileName):
    with open(fileName) as d:
        data = d.readlines()
    exerciseList = [json.loads(x) for x in data]
    

def writeFile(fileName, exercise_dict):
    with open(fileName, 'w') as json_file:
        json.dump(exercise_dict, json_file)
###############################################################
#TKinter Window Output

window = tk.Tk('Exercise Routine')
window.title('Stretch with Stretch: Configure Exercise')
  
#Styles
window.geometry('300x600')
window['background'] = '#E0EEC6'
#ADDING A SCROLLBAR
#window.myscrollbar=tk.Scrollbar(window,orient="vertical")
#window.myscrollbar.pack(side="right",fill="y")

#Storage Dict
textEntries = []
menuEntries = []
class_data = {
    "exercises": []
    
}


#####################################
#TKinter Functions
def createRoutine():
    
    
    original_exercises = class_data["exercises"]

    for i in range(len(textEntries)):
        print(menuEntries)
        print(textEntries)
        #create new ex  
        print(i, " :",menuEntries[i], " ",menuEntries[i].get())
        new_exercise = {}
        new_exercise["name"] = menuEntries[i].get()
        new_exercise["duration"] = textEntries[i].get()
        new_exercise["actions"] = ["spin"]
        #add ex to routine
        print(new_exercise)
        original_exercises.append(new_exercise)

    class_data["exercises"] = original_exercises

    #output routine to text File
    writeFile('test.txt', class_data)
    readFile('test.txt')

    #clear fields
    delete_entries()
    emptyClassData()


def emptyClassData():
    class_data["exercises"] = []
    

def addNewEx(): 
    exlab = tk.Label(window, text="Exercise", bg="#E0EEC6", font='helvetica 16')
    exlab.pack()
    variable = tk.StringVar(window)
    variable.set("Sit and Reach (Left)") # default value
    window.menu = tk.OptionMenu(window, variable, "Sit and Reach (Left)", "Sit and Reach (Right)", "Seated Kick (Left)", "Seated Kick (Right)", "Stand and Reach (Left)", "Stand and Reach (Right)")
    window.menu.pack()
    window.menu.config(highlightbackground='#FFFFFF', bg='#FFFFFF', font='helvetica 16')
    menuEntries.append(variable)
    print(menuEntries)


    durlab = tk.Label(window, text="Duration (s)", bg="#E0EEC6", font='helvetica 16')
    durlab.pack()
    window.txt = tk.Entry(window, width=10)
    window.txt.pack()
    window.txt.config(highlightbackground='#E0EEC6', relief="solid", borderwidth=1, font='helvetica 16')
    textEntries.append(window.txt)
    print(textEntries)
    
def remove_exercises():
    #not less than 1 entry
    if len(textEntries) > 1:
        del textEntries[-1]
        del menuEntries[-1]
        packList = window.pack_slaves()
        print(packList)
        for x in range(1,5):
            packList[len(packList)-x].pack_forget()
            print(packList[len(textEntries)-x])
        print(packList)
def delete_entries():
    for field in textEntries:
        field.delete(0,END)
    for field in menuEntries:
        field.set("Sit and Reach (Left)")
    

    
#Create Buttons
bt0 = tk.Button(window, text="Add New Exercise", highlightbackground="#E0EEC6", command=addNewEx)
bt0.pack()
bt1 = tk.Button(window, text="Submit Routine", highlightbackground="#E0EEC6", command=createRoutine)
bt1.pack()
bt2 = tk.Button(window, text="Remove Exercise", highlightbackground="#E0EEC6", command=remove_exercises)
bt2.pack()
##INITIAL FIELDS##
title = tk.Label(window, text="Configure Exercise", bg="#E0EEC6", font=('helvetica bold', 20))
title.pack()

#Exercise
exlab = tk.Label(window, text="Exercise", bg="#E0EEC6", font='helvetica 16')
exlab.pack()
variable = tk.StringVar(window)
variable.set("Sit and Reach (Left)") # default value
window.menu = tk.OptionMenu(window, variable, "Sit and Reach (Left)", "Sit and Reach (Right)", "Seated Kick (Left)", "Seated Kick (Right)", "Stand and Reach (Left)", "Stand and Reach (Right)")
window.menu.pack()
window.menu.config(highlightbackground='#FFFFFF', bg='#FFFFFF', font='helvetica 16')
menuEntries.append(variable)


#Duration
durlab = tk.Label(window, text="Duration (s)", bg="#E0EEC6", font='helvetica 16')
durlab.pack()
window.txt = tk.Entry(window, width=10)
window.txt.pack()
window.txt.config(highlightbackground='#E0EEC6', relief="solid", borderwidth=1, font='helvetica 16')
textEntries.append(window.txt)


 
window.mainloop()
