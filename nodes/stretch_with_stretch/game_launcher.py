#!/usr/bin/env python2
from __future__ import print_function

import os
import json
import rospy
from std_msgs.msg import Bool, String


class GameLauncher:
    def __init__(self, path):
        rospy.init_node("game_starter_node", anonymous=True)

        self.sws_ready_subscriber = rospy.Subscriber(
            "/sws_ready", Bool, self.sws_ready_callback
        )
        self.select_exercise_publisher = rospy.Publisher(
            "/sws_select_exercise", String, queue_size=1
        )

        self.sws_ready = False
        self.routine = None
        if os.path.exists(path):
            self.routine = json.load(open(path, "r"))
        else:
            rospy.logwarn("Could not load routine from: {}".format(path))

    def sws_ready_callback(self, data):
        self.sws_ready = data.data

    def main(self):
        if self.routine is None:
            return

        # wait for stretch with stretch
        while not rospy.is_shutdown():
            if not self.sws_ready:
                continue
        if rospy.is_shutdown():
            return

        for exercise in self.routine:
            # parse exercise
            name = exercise["name"]
            total_duration = sum(item["duration"] for item in exercise["poses"])
            exercise_string = json.dumps(exercise)

            # wait for stretch with stretch
            while not rospy.is_shutdown():
                if not self.sws_ready:
                    continue
            if rospy.is_shutdown():
                return

            # launch exercise
            rospy.loginfo("Launching {}".format(name))
            self.select_exercise_publisher.publish(exercise_string)
            rospy.sleep(total_duration)


if __name__ == "__main__":
    GameLauncher("exercise-routine.json").main()
