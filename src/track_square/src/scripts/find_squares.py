#!/usr/bin/env python

import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

bridge = CvBridge()

def main():
    rospy.Subscriber("/io/internal_camera/right_hand_camera/image_rect", Image, callback)
    rospy.spin()
    print("here")

def callback(msg):
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    cv_image = cv2.GaussianBlur(cv_image, (5,5), 0)
    value, thresh = cv2.threshold(cv_image, 80,255,cv2.THRESH_BINARY_INV)
    contours, h = cv2.findContours(thresh, 1, 2)
    print("ok")

    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
        if len(approx) == 4:
            cv2.drawContours(cv_image, [cnt],0,(0,0,255),-1)
    cv2.imshow('thresh', thresh)
    cv2.imshow('img', cv_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    cv2.imshow('Image', cv_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
if __name__ == "__main__":
    rospy.init_node('square_finder', anonymous=True)
    main()