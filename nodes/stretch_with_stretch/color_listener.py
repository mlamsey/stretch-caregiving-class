#!/usr/bin/env python3

# ROS
import rospy

# Messages
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int8

# Speech Recognition
import speech_recognition as sr

# Other
from colors import colors


def parse_string_for_unique_colors(text_string):
    words = text_string.split()
    unique_words = list(set(words))
    n_unique_colors = 0

    for word in unique_words:
        if any([word is color_string for color_string in colors]) or any(
                [color_string in word for color_string in colors]):
            n_unique_colors += 1
            # rospy.loginfo("%s: It's a color!" % word)

    rospy.loginfo("%i unique colors identified!" % n_unique_colors)
    return n_unique_colors


class color_listener():
    def __init__(self):
        rospy.init_node('color_listener')
        self.recognizer = sr.Recognizer()

        # Pub/Sub
        self.color_string_publisher = rospy.Publisher("/speech/color", String, queue_size=1)
        self.n_unique_colors_publisher = rospy.Publisher("/speech/n_unique_colors", Int8, queue_size=1)
        self.start_recording_subscriber = rospy.Subscriber(
            "/speech/start_recording", Float32, self.start_recording_callback)

        # Stuff
        self.data = None

        # Config
        self.rate = 10

    def start_recording_callback(self, data):
        recording_length = data.data

        with sr.Microphone() as source:
            audio_clip = self.recognizer.record(source, duration=recording_length)
            text_string = self.predict_text(audio_clip)
            if text_string is not None:
                n_unique_colors = parse_string_for_unique_colors(text_string)
                self.n_unique_colors_publisher.publish(n_unique_colors)
                self.color_string_publisher.publish(text_string)

    def predict_text(self, audio_clip):
        rospy.loginfo("Processing Audio...")
        try:
            return self.recognizer.recognize_sphinx(audio_clip)
        except sr.UnknownValueError:
            rospy.loginfo("Sphinx could not understand audio")
            return None
        except sr.RequestError as e:
            rospy.loginfo("Sphinx error; {0}".format(e))
            return None

    def main(self):
        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    node = color_listener()
    node.main()
