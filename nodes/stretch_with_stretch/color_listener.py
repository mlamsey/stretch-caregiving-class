#!/usr/bin/env python2

# ROS
import rospy

# Speech Recognition
import speech_recognition as sr

# Messages
from std_msgs.msg import Float32, Int8

# Other
from colors import colors


def _parse_string_for_unique_colors(text_string):
    rospy.loginfo("RAW STRING: %s" % text_string)
    words = text_string.split()
    unique_words = list(set(words))
    n_unique_colors = 0

    for word in unique_words:
        if any([word is color_string for color_string in colors]) or any(
                [color_string in word for color_string in colors]):
            n_unique_colors += 1

    rospy.loginfo("%i unique colors identified!" % n_unique_colors)
    return n_unique_colors


class ColorListener():
    def __init__(self):
        rospy.init_node('color_listener')

        # Configure speech recognizer
        self.recognizer = sr.Recognizer()

        # Pub/Sub
        self.start_recording_subscriber = rospy.Subscriber(
            "/speech/start_recording", Float32, self.start_recording_callback
        )
        self.n_unique_colors_publisher = rospy.Publisher(
            "/speech/n_unique_colors", Int8, queue_size=1
        )

        # Config
        self.rate = 10

    def _predict_text(self, audio_clip):
        rospy.loginfo("Processing Audio...")
        try:
            # return self.recognizer.recognize_sphinx(audio_clip)
            return self.recognizer.recognize_google(audio_clip)
        except sr.UnknownValueError:
            rospy.loginfo("Speech recognizer could not understand audio")
            return None
        except sr.RequestError as e:
            rospy.loginfo("Speech recognition error; {0}".format(e))
            return None

    def start_recording_callback(self, data):
        recording_length = data.data
        # TODO: make a while loop that records small chunks?
        with sr.Microphone() as source:
            audio_clip = self.recognizer.record(source, duration=recording_length)
            text_string = self._predict_text(audio_clip)
            if text_string is not None:
                n_unique_colors = _parse_string_for_unique_colors(text_string)
                self.n_unique_colors_publisher.publish(n_unique_colors)

    def main(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    ColorListener().main()
