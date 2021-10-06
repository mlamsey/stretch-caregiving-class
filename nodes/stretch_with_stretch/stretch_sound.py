#!/usr/bin/env python2
from __future__ import print_function, division

# ROS
import rospy

# Sound Play
from sound_play.libsoundplay import SoundClient


class StretchSound:
    """Example code:
    http://wiki.ros.org/sound_play/Tutorials/How%20to%20Create%20a%20Sound%20Interface
    """

    def __init__(self):
        rospy.init_node("stretch_sound", anonymous=True)

        self.rate = 1 / 5  # play sound every 5 seconds
        self.handle = SoundClient()

    def main(self):
        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            # play sound
            msg = "hi everyone"
            rospy.loginfo("say {}".format(msg))
            self.handle.say(msg)
            self.handle.play()

            rate.sleep()


if __name__ == "__main__":
    node = StretchSound()
    node.main()
