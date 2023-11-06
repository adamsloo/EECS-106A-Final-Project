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
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.1
# See the License for the specific language governing permissions and
# limitations under the License.

"""
SDK Joint Position Example: keyboard
"""
import argparse

import rospy
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION

from cv_bridge import CvBridge

current_ids = {}
recent_image = None
bridge = CvBridge()

#upload the photo to the server, determine text
def upload_photo(id):
    if recent_image != None:
        cv_image = bridge.imgmsg_to_cv2(recent_image, desired_encoding="passthrough")
        print(cv_image)
    
def image_callback(msg):
    global recent_image
    recent_image = msg

def marker_callback(msg):
    # Need to add a check if marker was removed from frame
    if msg.id not in current_ids.keys():
        current_ids[msg.id] = None
        print(msg.id)
        upload_photo(id)


if __name__ == '__main__':
    rospy.init_node("song_tracker")
    rospy.Subscriber("/visualization_marker", Marker, marker_callback)
    rospy.Subscriber("/io/internal_camera/right_hand_camera/image_rect", Image, image_callback)
    rospy.spin()
