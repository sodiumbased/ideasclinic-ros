#!/usr/bin/env python

# Necessary imports
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# The subscriber class definition where objects subscribes and listens to a given topic that sends an `Image`
class ImageSubscriber:

    def callback_(self, img):
        self.latest_img = img

    def display_image(self):
        try:
            cv_im = CvBridge().imgmsg_to_cv2(self.latest_img)
            cv2.imshow(self.topic, cv_im)
            cv2.waitKey(25)
        except CvBridgeError as e:
            print e

    def get_image(self):
        return CvBridge().imgmsg_to_cv2(self.latest_img)

    def __init__(self, topic):
        self.topic = topic
        rospy.Subscriber(topic, Image, callback=self.callback_)
