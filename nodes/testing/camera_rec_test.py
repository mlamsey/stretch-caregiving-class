#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def process_image(img):
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return img

class CameraRecorder:
    def __init__(self):
        # config
        self.rate = 30

        # data
        self.image_rgb = None

        # pub/sub
        self.sub_camera = rospy.Subscriber("/camera/color/image_raw", Image, callback=self.callback_get_image_rgb)

    def callback_get_image_rgb(self, data):
        self.image_rgb = data

    def main(self):
        # ros init
        rospy.init_node("camera_recorder", anonymous=True)
        rate = rospy.Rate(self.rate)
        rospy.wait_for_message("/camera/color/image_raw", Image)

        # other init
        bridge = CvBridge()
        i = 0

        while not rospy.is_shutdown():
            if self.image_rgb is not None:
                cv_image = process_image(bridge.imgmsg_to_cv2(self.image_rgb))
                i_str = str(i).zfill(5)
                cv2.imwrite("/home/hello-robot/test/" + i_str + ".png", cv_image)
                i += 1
            if i > 1000:
                break
            rate.sleep()

if __name__ == '__main__':
    CameraRecorder().main()
