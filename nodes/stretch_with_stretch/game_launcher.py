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

        self.rate = 5
        self.sws_ready = False

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
                routine = None
                if ex_in == "A":
                    routine = json.load(open("/home/hello-robot/catkin_ws/src/stretch-caregiving-class/nodes/stretch_with_stretch/exercises/A_sit_reach_right.txt", "r"))
                elif ex_in == "B":
                    routine = json.load(open("/home/hello-robot/catkin_ws/src/stretch-caregiving-class/nodes/stretch_with_stretch/exercises/B_sit_and_kick_right.txt", "r"))
                elif ex_in == "C":
                    routine = json.load(open("/home/hello-robot/catkin_ws/src/stretch-caregiving-class/nodes/stretch_with_stretch/exercises/C_sit_hold_right.txt", "r"))
                elif ex_in == "D":
                    routine = json.load(open("/home/hello-robot/catkin_ws/src/stretch-caregiving-class/nodes/stretch_with_stretch/exercises/D_sit_reach_left.txt", "r"))
                elif ex_in == "E":
                    routine = json.load(open("/home/hello-robot/catkin_ws/src/stretch-caregiving-class/nodes/stretch_with_stretch/exercises/E_stand_reach_left.txt", "r"))
                elif ex_in == "F":
                    routine = json.load(open("/home/hello-robot/catkin_ws/src/stretch-caregiving-class/nodes/stretch_with_stretch/exercises/F_sit_hold_left.txt", "r"))
            elif main_in == "J":
                rospy.loginfo("Please enter the absolute path to a JSON exercise file:")
                path_in = raw_input()
                if os.path.exists(path_in):
                    routine = json.load(open(path_in, "r"))
                else:
                    rospy.loginfo("Invalid Entry - file does not exist!")
            elif main_in == "Q":
                rospy.loginfo("Quitting...")
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
