#!/usr/bin/env python
"""
Starter script for 106a lab7. 
Author: Chris Correa
"""
import sys
import argparse
import numpy as np
import rospkg
import roslaunch

from paths.trajectories import LinearTrajectory, CircularTrajectory
from paths.paths import MotionPath
from paths.path_planner import PathPlanner
from controllers.controllers import ( 
    PIDJointVelocityController, 
    FeedforwardJointVelocityController
)
from utils.utils import *

from trac_ik_python.trac_ik import IK

import rospy
import tf2_ros
import intera_interface
from moveit_msgs.msg import DisplayTrajectory, RobotState
from sawyer_pykdl import sawyer_kinematics

from intera_interface import gripper as robot_gripper
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String

current_marker_names = []

class ArListener:
    def __init__(self):
        self.allow_execution = True
        self.current_marker_names = []

    # Gets list of ar markers
    def ar_marker_callback(self, data):
        if self.allow_execution:
            self.allow_execution = False
            self.current_marker_names = [marker.id for marker in data.markers]
            current_marker_names = self.current_marker_names
            print("Marker IDs: ", self.current_marker_names)
        else:
            pass

    # Controls when we take a look at what markers are in the workspace
    def timer_callback(self, event):
        self.allow_execution = True

    # Node subscribing to topic where all ar markers are published
    def listen(self):
        rospy.init_node("ar_marker_listiner")
        rospy.Timer(rospy.Duration(10), self.timer_callback)
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_marker_callback)
        rospy.spin()

    def choose_random_marker(self):
        pass


def seen_markers_publisher_node():
    rospy.init_node('seen_markers_publisher_node')
    pub = rospy.Publisher('/seen_markers', String, queue_size = 10)
    rate = rospy.Rate(1)

    listener = ArListener()
    listener.listen()

    while not rospy.is_shutdown():
        rospy.loginfo(current_marker_names)
        pub.publish(current_marker_names)
        rate.sleep()

def tuck():
        """
        Tuck the robot arm to the start position. Use with caution
        """
        if input('Would you like to tuck the arm? (y/n): ') == 'y':
            rospack = rospkg.RosPack()
            path = rospack.get_path('sawyer_full_stack')
            launch_path = path + '/launch/custom_sawyer_tuck.launch'
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
            launch.start()
        else:
            print('Canceled. Not tucking the arm.')

if __name__ == "__main__":
    #tuck()
    try:
        seen_markers_publisher_node()
    except rospy.ROSInterruptException:
        pass

