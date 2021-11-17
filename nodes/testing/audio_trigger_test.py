#!/usr/bin/env python2

# ROS
import rospy

# Messages
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int8


class audio_trigger_test():
    def __init__(self):
        rospy.init_node('audio_trigger_test')

        # Pub/Sub
        self.start_recording_publisher = rospy.Publisher("/speech/start_recording", Float32, queue_size=1)
        self.color_string_subscriber = rospy.Subscriber("/speech/color", String, self.color_string_callback)
        self.n_unique_colors_subscriber = rospy.Subscriber("/speech/n_unique_colors", Int8)

        # State
        self.color_string = None
        self.n_unique_colors = None

        # Config
        self.recording_time = 30.
        self.rate = 10

    def n_unique_colors_callback(self, data):
        self.n_unique_colors = data.data
        rospy.loginfo("%i unique colors received." % self.n_unique_colors)

    def color_string_callback(self, data):
        self.color_string = data.data
        rospy.loginfo("String received: %s" % self.color_string)

    def main(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            raw_input("Press Enter to Query for %i seconds. " % self.recording_time)
            self.start_recording_publisher.publish(self.recording_time)
            rospy.sleep(self.recording_time + 1.)
            rate.sleep()


if __name__ == '__main__':
    node = audio_trigger_test()
    node.main()
