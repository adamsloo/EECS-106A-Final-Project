#! /usr/bin/env python
# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
import intera_interface

def show_image_callback(img_data, xxx_todo_changeme):
    """The callback function to show image by using CvBridge and cv
    """
    (edge_detection, window_name) = xxx_todo_changeme
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")
    except CvBridgeError as err:
        rospy.logerr(err)
        return
    if edge_detection == True:
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        # customize the second and the third argument, minVal and maxVal
        # in function cv2.Canny if needed
        get_edge = cv2.Canny(blurred, 10, 100)
        cv_image = np.hstack([get_edge])
    edge_str = "(Edge Detection)" if edge_detection else ''
    cv_win_name = ' '.join([window_name, edge_str])
    cv2.namedWindow(cv_win_name, 0)
    # refresh the image on the screen
    cv2.imshow(cv_win_name, cv_image)
    cv2.waitKey(3)

def main(current_camera):
    """Camera Display Example

    Cognex Hand Camera Ranges
        - exposure: [0.01-100]
        - gain: [0-255]
    Head Camera Ranges:
        - exposure: [0-100], -1 for auto-exposure
        - gain: [0-79], -1 for auto-gain
    """
    rp = intera_interface.RobotParams()
    valid_cameras = rp.get_camera_names()

    print("Initializing node... ")
    rospy.init_node('camera_display', anonymous=True)
    cameras = intera_interface.Cameras()
    if not cameras.verify_camera_exists(current_camera):
        rospy.logerr("Could not detect the specified camera, exiting the example.")
        return
    rospy.loginfo("Opening camera '{0}'...".format(current_camera))
    cameras.start_streaming(current_camera)
    rectify_image = True
    use_canny_edge = False
    cameras.set_callback(current_camera, show_image_callback,
        rectify_image=rectify_image, callback_args=(use_canny_edge, current_camera))


    def clean_shutdown():
        print("Shutting down camera_display node.")
        cv2.destroyAllWindows()

    rospy.on_shutdown(clean_shutdown)
    rospy.loginfo("Camera_display node running. Ctrl-c to quit")
    rospy.spin()


if __name__ == '__main__':
    camera_to_use = input("Input camera")
    main(camera_to_use)
    camera_to_use = input("Input camera")
    main(camera_to_use)
