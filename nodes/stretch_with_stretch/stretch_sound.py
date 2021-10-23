#!/usr/bin/env python2
from __future__ import division, print_function

import os

import rospy
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import Bool, String, UInt8


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
        self.announce_score_subscriber = rospy.Subscriber(
            "/sws_announce_score", UInt8, self.announce_score, queue_size=1
        )

        self.base_sound_path = os.path.join(
            os.path.expanduser("~"),
            "catkin_ws",
            "src",
            "stretch-caregiving-class",
            "sounds",
        )

        self.handle = SoundClient()
        self.allow_time = rospy.Time.now()

    def set_allow_time(self, delay):
        self.allow_time = rospy.Time.now() + rospy.Duration.from_sec(delay)

    def is_ready(self):
        return rospy.Time.now() >= self.allow_time

    def wait_for_ready(self):
        while not self.is_ready():
            rospy.sleep(0.1)

    def notify(self, data):
        if not self.is_ready():
            return
        path = os.path.join(self.base_sound_path, "notify.wav")
        self.handle.playWave(path, blocking=True)

    def start_exercise(self, data):
        path = os.path.join(self.base_sound_path, "3-2-1-go_75bpm.wav")
        self.handle.playWave(path, blocking=False)
        self.set_allow_time(2.0)

    def point_scored(self, data):
        if not self.is_ready():
            return
        path = os.path.join(self.base_sound_path, "point_scored.wav")
        self.handle.playWave(path, blocking=False)
        self.set_allow_time(0.75)

    def announce_score(self, data):
        fname = "{}.wav".format(data.data)
        path = os.path.join(self.base_sound_path, "tts", "v1", "en", "ie", fname)
        if os.path.exists(path):
            self.handle.playWave(path, blocking=True)
        if data.data < 10:
            return
        fname = "nice-job.wav"
        path = os.path.join(self.base_sound_path, "tts", "v1", "en", "ie", fname)
        if os.path.exists(path):
            self.handle.playWave(path, blocking=True)


    def main(self):
        rospy.spin()


if __name__ == "__main__":
    node = StretchSound()
    node.main()
