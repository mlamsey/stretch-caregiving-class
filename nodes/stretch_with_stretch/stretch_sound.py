#!/usr/bin/env python2
from __future__ import division, print_function

import os

import rospy
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import Bool


class StretchSound:
    def __init__(self):
        rospy.init_node("stretch_sound", anonymous=True)

        self.point_scored_subscriber = rospy.Subscriber(
            "/game_state/point_scored", Bool, self.point_scored, queue_size=1
        )
        self.robot_initialized_subscriber = rospy.Subscriber(
            "/game_state/robot_initialized", Bool, self.welcome, queue_size=1
        )
        self.start_game_subscriber = rospy.Subscriber(
            "/game_state/start_game", Bool, self.start_exercise, queue_size=1
        )
        self.robot_initialized_subscriber = rospy.Subscriber(
            "/game_state/robot_done", Bool, self.goodbye, queue_size=1
        )

        self.base_sound_path = os.path.join(
            os.path.expanduser("~"),
            "catkin_ws",
            "src",
            "stretch-caregiving-class",
            "sounds",
        )

        # AM: why do we need to set rate here????
        self.rate = 20
        self.handle = SoundClient()

    def main(self):
        rospy.spin()

    def welcome(self, data):
        if data.data:
            path = os.path.join(self.base_sound_path, "welcome.wav")
            self.handle.playWave(path, blocking=True)

    def start_exercise(self, data):
        if data.data:
            path = os.path.join(self.base_sound_path, "3-2-1-go_75bpm.wav")
            self.handle.playWave(path, blocking=True)

    def point_scored(self, data):
        if data.data:
            path = os.path.join(self.base_sound_path, "point_scored.wav")
            self.handle.playWave(path, blocking=False)

    def goodbye(self, data):
        if data.data:
            path = os.path.join(self.base_sound_path, "goodbye.wav")
            self.handle.playWave(path, blocking=True)


if __name__ == "__main__":
    node = StretchSound()
    node.main()
