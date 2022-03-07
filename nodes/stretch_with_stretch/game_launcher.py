#!/usr/bin/env python2
from __future__ import print_function

import os
import json
import rospy
from std_msgs.msg import Bool, String

import menu

# print cleanup
gamelauncher_debug = False

def launcher_loginfo(msg):
    if gamelauncher_debug:
        rospy.loginfo(msg)

class GameLauncher:
    def __init__(self):
        rospy.init_node("game_starter_node", anonymous=True)

        self.sws_ready_subscriber = rospy.Subscriber(
            "/sws_ready", Bool, self.sws_ready_callback
        )
        self.select_exercise_publisher = rospy.Publisher(
            "/sws_select_exercise", String, queue_size=1
        )
        self.say_publisher = rospy.Publisher(
            "/sws_say", String, queue_size=1
        )

        self.rate = 5
        self.sws_ready = False

        # internal
        self.exercise_dir = "/home/hello-robot/catkin_ws/src/stretch-caregiving-class/nodes/stretch_with_stretch/exercises/"

    def sws_ready_callback(self, data):
        self.sws_ready = data.data

    def main(self):
        # if self.routine is None:
        #     return

        rate = rospy.Rate(self.rate)

        # wait for stretch with stretch
        while not rospy.is_shutdown():
            if self.sws_ready:
                break
            rate.sleep()
        if rospy.is_shutdown():
            return

        # MAIN LOOP!
        while not rospy.is_shutdown():
            # reset routine
            routine = None

            # query user input
            main_in = menu.get_user_input_with_confirmation("main")
            if main_in == "M":
                ex_in = menu.get_user_input_with_confirmation("exercise")
                path = None

                # parse user selection
                if ex_in == "A":
                    path = self.exercise_dir + "A_sit_reach_right.txt"
                elif ex_in == "B":
                    path = self.exercise_dir + "B_sit_and_kick_right.txt"
                elif ex_in == "C":
                    path = self.exercise_dir + "C_sit_hold_right.txt"
                elif ex_in == "D":
                    path = self.exercise_dir + "D_sit_reach_left.txt"
                elif ex_in == "E":
                    path = self.exercise_dir + "E_stand_reach_left.txt"
                elif ex_in == "F":
                    path = self.exercise_dir + "F_sit_hold_left.txt"
                
                # load routine
                if path is not None:
                    try:
                        exercise_file = open(path, "r")
                        routine = json.load(exercise_file)
                    except IOError:
                        rospy.logerr("File path does not exist: " + path)
                    except json.JSONDecodeError:
                        rospy.logerr("File contains invalid json: " + path)

            elif main_in == "J":
                rospy.loginfo("Please enter the absolute path to a JSON exercise file:")
                path_in = raw_input()
                if os.path.exists(path_in):
                    routine = json.load(open(path_in, "r"))
                else:
                    rospy.loginfo("Invalid Entry - file does not exist!")
            elif main_in == "Q":
                rospy.loginfo("Quitting...")
                self.say_publisher.publish("thanks for playing")
                return


            # run routine
            if routine is not None:
                for exercise in routine["exercises"]:
                    # parse exercise
                    name = exercise["name"]
                    movement = exercise["movement"]
                    total_duration = sum(item["duration"] for item in movement["poses"])
                    exercise_string = json.dumps(exercise)

                    # wait for stretch with stretch
                    while not rospy.is_shutdown():
                        if self.sws_ready:
                            break
                    if rospy.is_shutdown():
                        return

                    # launch exercise
                    launcher_loginfo("Launching '{}'".format(name))
                    self.select_exercise_publisher.publish(exercise_string)
                    rospy.sleep(total_duration)


if __name__ == "__main__":
    GameLauncher().main()
