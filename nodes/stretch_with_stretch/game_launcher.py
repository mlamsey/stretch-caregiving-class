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
    def __init__(self, path):
        rospy.init_node("game_starter_node", anonymous=True)

        self.sws_ready_subscriber = rospy.Subscriber(
            "/sws_ready", Bool, self.sws_ready_callback
        )
        self.select_exercise_publisher = rospy.Publisher(
            "/sws_select_exercise", String, queue_size=1
        )

        self.rate = 5
        self.sws_ready = False
        if os.path.exists(path):
            self.routine = json.load(open(path, "r"))
            # print(self.routine)
            launcher_loginfo("Exercise routine:\n{}".format(json.dumps(self.routine, indent=2)))
        else:
            self.routine = None
            rospy.logwarn("Routine file {} not found in {}".format(path, os.getcwd()))

    def sws_ready_callback(self, data):
        self.sws_ready = data.data

    def main(self):
        if self.routine is None:
            return

        rate = rospy.Rate(self.rate)

        # wait for stretch with stretch
        while not rospy.is_shutdown():
            if self.sws_ready:
                break
            rate.sleep()
        if rospy.is_shutdown():
            return

        # insert
        main_in = menu.get_user_input_with_confirmation("main")
        if main_in == "M":
            ex_in = menu.get_user_input_with_confirmation("exercise")
            if ex_in == "A":
                routine = json.load(open("/home/hello-robot/catkin_ws/src/stretch-caregiving-class/nodes/websiteTest/test.txt", "r"))
                ex = routine['exercises']
                exercise_a = ex[0]

                exercise_a_string = json.dumps(exercise_a)
                self.select_exercise_publisher.publish(exercise_a_string)
        elif main_in == "J":
            rospy.logwarn("J not implemented!")
        else:
            return

        return

        for exercise in self.routine:
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
    GameLauncher("exercise-routine.json").main()
