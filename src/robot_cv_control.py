#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class robot_cv_control:
    def __init__(self):
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/robot/camera/image_raw", Image, self.callback)
        self.lastpos = 0

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            print(e)

        try:
            # image algorithm
            max_row = 20
            slice_gray = cv_image[-max_row:-1,:]
            width = slice_gray.shape[1]
            height = slice_gray.shape[0]
            # Convert to Binary image
            threshold = 100
            _, img_bin = cv.threshold(slice_gray, threshold, 255, cv.THRESH_BINARY)
            # average rows
            averaged_rows = np.zeros(width)
            for j in range(width):
                for i in range(height):
                    averaged_rows[j] = averaged_rows[j] + img_bin[i,j]
                averaged_rows[j] = averaged_rows[j] / img_bin.shape[0]
            
            # cv.imshow("Image window", img_bin)
            # cv.waitKey(0)
            
            # Find index of center of road by setting two thresholds
            off_road = 180
            on_road = 70

            left_edge = None
            right_edge = None

            # left edge of frame is already black because left edge of raod not included in frame
            if averaged_rows[0] < on_road:
                left_edge = 0
            if averaged_rows[-1] < on_road:
                right_edge = width

            for j in range(1,len(averaged_rows)-1):
                if averaged_rows[j] < on_road and averaged_rows[j-1] > on_road:
                    left_edge = j
                if averaged_rows[j] > off_road and averaged_rows[j-1] < off_road:
                    right_edge = j

            if left_edge is not None and right_edge is not None:
                center_idx = int((right_edge + left_edge) / 2)
            else:
                if self.lastpos < int(width / 2):
                    center_idx = 0
                else:
                    center_idx = width
            print("lastpos: {}".format(self.lastpos))
            self.lastpos = center_idx

            # test = cv_image[-1,:]
            # print("Max pixel value in bottom rows, ie lightest {} its value {}".format(test.argmax(), test[test.argmax()]))
            # print("Min pixel value in bottom rows, ie darkest {} its value {}".format(test.argmin(), test[test.argmin()]))
            # # print("One \"column\"{col}".format(col = cv_image[:,1,:]))
            # # print("One \"row\"{row}".format(row = cv_image[1,:,:]))
            # print("Dimensions of image {dim}".format(dim=cv_image.shape))
            # print("length of averaged_rows {l}".format(l=len(averaged_rows)))

            # print("left_edge: {left_edge}".format(left_edge=left_edge))
            # print("right_edge: {right_edge}".format(right_edge=right_edge))
            # print("one bin row: {}".format(img_bin[-1,:]))
            # print("averaged_rows: {avg}".format(avg=averaged_rows))

            # Regime 1: Center point within tolerance
            reg_thresh = 40
            move = Twist()
            
            if abs(width / 2 - center_idx) < reg_thresh:
                move.linear.x = 0.4
            # Regime 2: Center point outside tolerance
            elif (center_idx + reg_thresh) > width / 2:
                move.angular.z = -0.5
            else:
                move.angular.z = 0.5

            self.vel_pub.publish(move)
            # print("center_idx: {center_idx}".format(center_idx=center_idx))
            # print("move: {}".format(move))
        except CvBridgeError as e:
            print(e)

def main(args):
    rcc = robot_cv_control()
    rospy.init_node("robot_cv_control", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)