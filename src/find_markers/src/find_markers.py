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
from std_msgs.msg import String, Float32MultiArray

current_marker_names = []

# Gets list of ar markers, resets the global variables 
def ar_marker_callback(data):
    global allow_execution
    rate = rospy.Rate(5)
    if allow_execution:
        allow_execution = False
        current_marker_names = [float(marker.id) for marker in data.markers]
        message = Float32MultiArray()
        message.data = current_marker_names
        pub.publish(message)
        print("Marker IDs: ", current_marker_names)
        rate.sleep()
        allow_execution = True
    else:
        pass

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
    try:
        allow_execution = True
        rospy.init_node('ar_marker_listener', anonymous =  True)
        pub = rospy.Publisher('/seen_markers', Float32MultiArray, queue_size = 10)
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, ar_marker_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

