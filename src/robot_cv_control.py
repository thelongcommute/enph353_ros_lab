#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2 as cv

def callback(data):
	pub = ropsy.Publisher("cmd_vel" , Twist)
	rospy.init_node("image_processor_talker", anonymous=True, queue_size=1)
	rate = rospy.Rate(1)
	pub.publish() # velocity message we define
	rate.sleep()  # Does this actually do anything because the callback is in a different thread then then listener?

def image_processor_listener():
	rospy.init_node("image_processor_listener", anonymous=True)
	rospy.Subscriber("image_raw", Image, callback)
	rospy.spin()

def image_algorithm(ros_image):


if __name__ = '__main__':
	image_processor_listener()