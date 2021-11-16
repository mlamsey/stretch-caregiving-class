#!/usr/bin/env python3

# ROS
import rospy

# Messages
from std_msgs.msg import String

# Speech Recognition
import speech_recognition as sr

# Other
from colors import colors


class color_listener():
    def __init__(self):
        rospy.init_node('color_listener')
        self.recognizer = sr.Recognizer()
        self.color_string_publisher = rospy.Publisher("/speech/color", String, queue_size=1)

    def predict_text(self, audio_clip):
        rospy.loginfo("Processing Audio...")
        try:
            text_string = self.recognizer.recognize_sphinx(audio_clip)
            rospy.loginfo("Text spoken: " + text_string)
            if any([text_string in substring for substring in colors]) or any([substring in text_string for substring in colors]):
                rospy.loginfo("It's a color!")
            else:
                rospy.loginfo("It's not a color!")
        except sr.UnknownValueError:
            rospy.loginfo("Sphinx could not understand audio")
        except sr.RequestError as e:
            rospy.loginfo("Sphinx error; {0}".format(e))

    def main(self):
        with sr.Microphone() as source:
            while not rospy.is_shutdown():
                rospy.loginfo("color_listener::main: say something!")
                audio_clip = self.recognizer.listen(source)
                self.predict_text(audio_clip)


if __name__ == '__main__':
    node = color_listener()
    node.main()
