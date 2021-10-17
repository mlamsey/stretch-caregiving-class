#!/usr/bin/env python2
from __future__ import division, print_function

import os

import rospy
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import Bool, String


class StretchSound:
    def __init__(self):
        rospy.init_node("stretch_sound", anonymous=True)

        self.sws_notify_subscriber = rospy.Subscriber(
            "/sws_notify", Bool, self.notify, queue_size=1
        )
        self.start_exercise_subscriber = rospy.Subscriber(
            "/sws_start_exercise", String, self.start_exercise, queue_size=1
        )
        self.point_scored_subscriber = rospy.Subscriber(
            "/sws_point_scored", Bool, self.point_scored, queue_size=1
        )

        self.base_sound_path = os.path.join(
            os.path.expanduser("~"),
            "catkin_ws",
            "src",
            "stretch-caregiving-class",
            "sounds",
        )

        self.handle = SoundClient()

    def notify(self, data):
        path = os.path.join(self.base_sound_path, "notify.wav")
        self.handle.playWave(path, blocking=True)

    def start_exercise(self, data):
        path = os.path.join(self.base_sound_path, "3-2-1-go_75bpm.wav")
        self.handle.playWave(path, blocking=True)

    def point_scored(self, data):
        path = os.path.join(self.base_sound_path, "point_scored.wav")
        self.handle.playWave(path, blocking=False)

    def main(self):
        rospy.spin()


if __name__ == "__main__":
    node = StretchSound()
    node.main()
