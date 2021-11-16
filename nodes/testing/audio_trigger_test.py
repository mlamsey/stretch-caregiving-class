#!/usr/bin/env python3

# ROS
import rospy

# Messages
from std_msgs.msg import String
from std_msgs.msg import Float32


class audio_trigger_test():
    def __init__(self):
        rospy.init_node('audio_trigger_test')

        # Pub/Sub
        self.start_recording_publisher = rospy.Publisher("/speech/start_recording", Float32, queue_size=1)
        self.color_string_subscriber = rospy.Subscriber("/speech/color", String, self.color_string_callback)

        # State
        self.data = None

        # Config
        self.recording_time = 5.
        self.rate = 10

    def color_string_callback(self, data):
        self.data = data
        print(data.data)

    def main(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            input("Press Enter to Query for %i seconds. " % self.recording_time)
            self.start_recording_publisher.publish(self.recording_time)
            rospy.sleep(self.recording_time + 1.)
            rate.sleep()


if __name__ == '__main__':
    node = audio_trigger_test()
    node.main()
