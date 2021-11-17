import json
import tkinter as tk
###############################################################
#File I/O
def readFile(fileName):
    with open(fileName) as d:
        data = d.readlines()
    exerciseList = [json.loads(x) for x in data]
    print(exerciseList)

def writeFile(fileName, exercise_dict):
    with open(fileName, 'w') as json_file:
        json.dump(exercise_dict, json_file)
###############################################################
#TKinter Window Output

window = tk.Tk('Exercise Routine')
window.title('Stretch with Stretch: Configure Exercise')
  
#Styles
window.geometry('400x300')
window['background'] = '#E0EEC6'

#Initial Fields
l1 = tk.Label(window, text="Configure Exercise", bg="#E0EEC6", font=('helvetica bold', 20))
l1.pack()

l2 = tk.Label(window, text="Exercise", bg="#E0EEC6", font='helvetica 16')
l2.pack()
variable = tk.StringVar(window)
variable.set("Sit and Reach") # default value
txt2 = tk.OptionMenu(window, variable, "Sit and Reach", "Seated Kick", "Exercise 3")
txt2.pack()
txt2.config(highlightbackground='#E0EEC6', bg='#E0EEC6', font='helvetica 16')

#txt1 = tk.Entry(window, width=10)
#txt1.pack()
#txt1.config(highlightbackground='#E0EEC6', relief="solid", borderwidth=1, font='helvetica 16')

l3 = tk.Label(window, text="Duration (s)", bg="#E0EEC6", font='helvetica 16')
l3.pack()
txt2 = tk.Entry(window, width=10)
txt2.pack()
txt2.config(highlightbackground='#E0EEC6', relief="solid", borderwidth=1, font='helvetica 16')


#Storage Dict
class_data = {
    "exercises": []
    
}
#####################################
#TKinter Functions
def createRoutine():
 
    new_exercise = {}
    original_exercises = class_data["exercises"]

    print(txt1.get())
    #create new ex   
    new_exercise["name"] = txt1.get()
    new_exercise["duration"] = txt2.get()
    new_exercise["actions"] = ["spin"]
    #add ex to routine
    original_exercises.append(new_exercise)

    class_data["exercises"] = original_exercises

    #output routine to text File
    writeFile('test.txt', class_data)
    readFile('test.txt')

def addNewEx():
    res = "Welcome to " + txt1.get()
    l1.configure(text= res)

    
#Create Buttons
bt0 = tk.Button(window, text="Add New Exercise", highlightbackground="#E0EEC6", command=addNewEx)
bt0.pack()
bt1 = tk.Button(window, text="Enter", highlightbackground="#E0EEC6", command=createRoutine)
bt1.pack()


 
window.mainloop()
