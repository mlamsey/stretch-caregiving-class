#!/usr/bin/env python2
from __future__ import print_function, division
import os

# ROS
import rospy

# Messages
from std_msgs.msg import Bool

# Sound Play
from sound_play.libsoundplay import SoundClient

class StretchSound:
    """Example code:
    http://wiki.ros.org/sound_play/Tutorials/How%20to%20Create%20a%20Sound%20Interface
    """

    def __init__(self):
        rospy.init_node("stretch_sound", anonymous=True)

        self.point_scored_subscriber = rospy.Subscriber('/game_state/point_scored', Bool, self.point_scored, queue_size=1)
        self.start_game_subscriber = rospy.Subscriber('/game_state/start_game', Bool, self.start_exercise, queue_size=1)

        self.base_sound_path = os.path.expanduser('~') + "/catkin_ws/src/stretch-caregiving-class/sounds/"

        self.rate = 1 / 5  # play sound every 5 seconds
        self.rate = 20
        self.handle = SoundClient()

    def main(self):
        rospy.spin()
        # rate = rospy.Rate(self.rate)

        # rospy.sleep(1)
        # self.start_exercise()

        # while not rospy.is_shutdown():
        #     # self.point_scored()
        #     # play sound
        #     # msg = "hi everyone"
        #     # rospy.loginfo("say {}".format(msg))
        #     # self.handle.say(msg)

        #     rate.sleep()

    def start_exercise(self, data):
        if data.data:
            start_exercise = self.base_sound_path + "3-2-1-go_75bpm.wav"
            self.handle.playWave(start_exercise,blocking=True)

    def point_scored(self, data):
        if data.data:
            point_scored = self.base_sound_path + "point_scored.wav"
            self.handle.playWave(point_scored,blocking=False)

if __name__ == "__main__":
    node = StretchSound()
    node.main()
