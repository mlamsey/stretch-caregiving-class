#!/usr/bin/env python3

# ROS
import rospy

# Messages
from std_msgs.msg import String
from std_msgs.msg import Float32

# Speech Recognition
import speech_recognition as sr

# Other
from colors import colors


class color_listener():
    def __init__(self):
        rospy.init_node('color_listener')
        self.recognizer = sr.Recognizer()

        # Pub/Sub
        self.color_string_publisher = rospy.Publisher("/speech/color", String, queue_size=1)
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
                self.color_string_publisher.publish(text_string)

    def predict_text(self, audio_clip):
        rospy.loginfo("Processing Audio...")
        try:
            text_string = self.recognizer.recognize_sphinx(audio_clip)
            rospy.loginfo("Text spoken: " + text_string)
            if any([text_string in substring for substring in colors]) or any([substring in text_string for substring in colors]):
                rospy.loginfo("It's a color!")
            else:
                rospy.loginfo("It's not a color!")
            return text_string
        except sr.UnknownValueError:
            rospy.loginfo("Sphinx could not understand audio")
            return None
        except sr.RequestError as e:
            rospy.loginfo("Sphinx error; {0}".format(e))
            return None

    def main(self):
        rate = rospy.Rate(self.rate)

        with sr.Microphone() as source:
            while not rospy.is_shutdown():
                rate.sleep()
                # rospy.loginfo("color_listener::main: say something!")
                # audio_clip = self.recognizer.listen(source)
                # self.predict_text(audio_clip)


if __name__ == '__main__':
    node = color_listener()
    node.main()
