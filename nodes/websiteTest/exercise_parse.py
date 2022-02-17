import json
import tkinter as tk
from tkinter import *
from exercise_interface import get_exercise_specification
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

# window = tk.Tk('Exercise Routine')
window = tk.Tk()
window.title('Stretch with Stretch: Configure Exercise')
  
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
        #print(nameEntries)
        #print(durEntries)
        #create new ex  
        #print(i, " :",nameEntries[i], " ",nameEntries[i].get())
        new_exercise = get_exercise_specification(nameEntries[i].get(), dirEntries[i].get(), diffEntries[i].get(), float(durEntries[i].get()), cognEntries[i].get()==1)
        
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
    variable.set("Sit and Reach") # default value
    window.menu = tk.OptionMenu(window, variable, "Sit and Reach", "Sit and Kick", "Stand and Reach")
    window.menu.pack()
    window.menu.config(highlightbackground='#FFFFFF', bg='#FFFFFF', font='helvetica 16')
    nameEntries.append(variable)
    print(nameEntries)

    lrlab = tk.Label(window, text="Direction", bg="#E0EEC6", font='helvetica 16')
    lrlab.pack()
    lrvar=tk.StringVar(window)
    lrvar.set("Left") #default value
    lr1 = tk.Radiobutton(window, text="Left",variable=lrvar, value="Left")
    lr1.pack()
    lr2 = tk.Radiobutton(window, text="Right", variable=lrvar, value="Right")
    lr2.pack()
    dirEntries.append(lrvar)

    difflab = tk.Label(window, text="Difficulty", bg="#E0EEC6", font='helvetica 16')
    difflab.pack()
    diffvar=tk.StringVar(window)
    diffvar.set("Medium") #default value
    diff1 = tk.Radiobutton(window, text="Easy", variable=diffvar, value="Easy", padx=11)
    diff1.pack()
    diff2 = tk.Radiobutton(window, text="Medium", variable=diffvar, value="Medium")
    diff2.pack()
    diff3 = tk.Radiobutton(window, text="Hard", variable=diffvar, value="Hard", padx=11)
    diff3.pack()
    diffEntries.append(diffvar)

    durlab = tk.Label(window, text="Duration (s)", bg="#E0EEC6", font='helvetica 16')
    durlab.pack()
    window.txt = tk.Entry(window, width=10)
    window.txt.pack()
    window.txt.config(highlightbackground='#E0EEC6', relief="solid", borderwidth=1, font='helvetica 16')
    durEntries.append(window.txt)
    print(durEntries)

    cognvar = tk.IntVar()
    bt3 = tk.Checkbutton(window, text="Include Cognitive Exercise in Routine",variable=cognvar)
    bt3.pack()
    cognEntries.append(cognvar)
    
def remove_exercises():
    #not less than 1 entry
    if len(durEntries) > 1:
        del durEntries[-1]
        del nameEntries[-1]
        packList = window.pack_slaves()
        print(packList)
        for x in range(1,5):
            packList[len(packList)-x].pack_forget()
            print(packList[len(durEntries)-x])
        print(packList)
def delete_entries():
    for field in durEntries:
        field.delete(0,END)
    for field in nameEntries:
        field.set("Sit and Reach")
    

    
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
variable.set("Sit and Reach") # default value
window.menu = tk.OptionMenu(window, variable, "Sit and Reach", "Sit and Kick", "Stand and Reach")
window.menu.pack()
window.menu.config(highlightbackground='#FFFFFF', bg='#FFFFFF', font='helvetica 16')
nameEntries.append(variable)

#L/R
lrlab = tk.Label(window, text="Direction", bg="#E0EEC6", font='helvetica 16')
lrlab.pack()
lrvar=tk.StringVar(window)
lrvar.set("Left") #default value
lr1 = tk.Radiobutton(window, text="Left",variable=lrvar, value="Left")
lr1.pack()
lr2 = tk.Radiobutton(window, text="Right", variable=lrvar, value="Right")
lr2.pack()
dirEntries.append(lrvar)

#Difficulty

difflab = tk.Label(window, text="Difficulty", bg="#E0EEC6", font='helvetica 16')
difflab.pack()
diffvar=tk.StringVar(window)
diffvar.set("Medium") #default value
diff1 = tk.Radiobutton(window, text="Easy", variable=diffvar, value="Easy", padx=11)
diff1.pack()
diff2 = tk.Radiobutton(window, text="Medium", variable=diffvar, value="Medium")
diff2.pack()
diff3 = tk.Radiobutton(window, text="Hard", variable=diffvar, value="Hard", padx=11)
diff3.pack()
diffEntries.append(diffvar)

#Duration
durlab = tk.Label(window, text="Duration (s)", bg="#E0EEC6", font='helvetica 16')
durlab.pack()
window.txt = tk.Entry(window, width=10)
window.txt.pack()
window.txt.config(highlightbackground='#E0EEC6', relief="solid", borderwidth=1, font='helvetica 16')
durEntries.append(window.txt)

#Cognitive Exercise Inclusion Checkbox
cognvar = tk.IntVar()
bt3 = tk.Checkbutton(window, text="Include Cognitive Exercise in Routine",variable=cognvar)
bt3.pack()
cognEntries.append(cognvar)
 
window.mainloop()
