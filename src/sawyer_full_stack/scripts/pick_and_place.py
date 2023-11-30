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

import time


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
        #print('tuck start ####################################')
        launch.start()
        #print('tuck start finished ####################################')
    else:
        print('Canceled. Not tucking the arm.')

def lookup_tag(tag_number):
    """
    Given an AR tag number, this returns the position of the AR tag in the robot's base frame.
    You can use either this function or try starting the scripts/tag_pub.py script.  More info
    about that script is in that file.  

    Parameters
    ----------
    tag_number : int

    Returns
    -------
    3x' :obj:`numpy.ndarray`
        tag position
    """

    # TODO: initialize a tf buffer and listener as in lab 3
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    
    try:
        # TODO: lookup the transform and save it in trans
        trans = tfBuffer.lookup_transform("base", f"ar_marker_{tag_number}", rospy.Time(0), rospy.Duration(10.0))
        # The rospy.Time(0) is the latest available 
        # The rospy.Duration(10.0) is the amount of time to wait for the transform to be available before throwing an exception
    except Exception as e:
        print(e)
        print("Retrying ...")

    tag_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
    return np.array(tag_pos)

def get_trajectory(limb, kin, ik_solver, tag_pos, num_way, task, z = -0.003):
    """
    Returns an appropriate robot trajectory for the specified task.  You should 
    be implementing the path functions in paths.py and call them here
    
    Parameters
    ----------
    task : string
        name of the task.  Options: line, circle, square
    tag_pos : 3x' :obj:`numpy.ndarray`
        
    Returns
    -------
    :obj:`moveit_msgs.msg.RobotTrajectory`
    """

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    try:
        trans = tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0), rospy.Duration(10.0))
    except Exception as e:
        print(e)

    current_position = np.array([getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')])
    print("Current Position:", current_position)
    #print("TASK ", type(task))

    if task == 'line':
        target_pos = tag_pos
        target_pos[2] = z + 0.3 #linear path moves to a Z position above AR Tag. CHANGE THIS TO 0.4 IT IS SCARY!!!!!!!
        print("TARGET POSITION:", target_pos)
        trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=9)
    elif task == 'adjustment':
        target_pos = current_position
        target_pos[2] = z
        print("TARGET POSITION ADJUSTMENT:", target_pos)
        trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=3)
    elif task == 'queue':
        target_pos = tag_pos
        target_pos[2] = z
        target_pos[1] -= 0.020
        print("TARGET POSITION:", target_pos)
        trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=9)
        
    else:
        raise ValueError('task {} not recognized'.format(task))
    
    path = MotionPath(limb, kin, ik_solver, trajectory)
    return path.to_robot_trajectory(num_way, True)


def pick_and_place(marker, prev_marker, task='line', rate=200, timeout=None, num_way=50):
    # Marker should be a single ar marker string

    rospy.init_node('moveit_node')
    
    tuck()
    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper')

    # Calibrate the gripper (other commands won't work unless you do this first)
    print('Calibrating...')
    right_gripper.calibrate()
    rospy.sleep(2.0)
    
    # this is used for sending commands (velocity, torque, etc) to the robot
    ik_solver = IK("base", "right_gripper_tip")
    limb = intera_interface.Limb("right")
    kin = sawyer_kinematics("right")

    # Lookup the AR tag position.
    tag_pos = lookup_tag(marker)

    
    # This is a wrapper around MoveIt! for you to use.  We use MoveIt! to go to the start position
    # of the trajectory
    planner = PathPlanner('right_arm')
    
    # ####### PICK ########
    robot_trajectory = get_trajectory(limb, kin, ik_solver, tag_pos, num_way, task)
    # Move to the trajectory start position
    plan = planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[0].positions)
    planner.execute_plan(plan[1])

    try:
        input('Press <Enter> to execute the trajectory using MOVEIT')
    except KeyboardInterrupt:
        sys.exit()

    # Uses MoveIt! to execute the trajectory.
    print("opening right gripper...")
    right_gripper.open()
    planner.execute_plan(robot_trajectory)

    # Adjusting to get closer to the cube
    robot_trajectory = get_trajectory(limb, kin, ik_solver, tag_pos, num_way, task='adjustment')
    plan = planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[0].positions)
    planner.execute_plan(plan[1])
    planner.execute_plan(robot_trajectory)
    time.sleep(5)
    # Grab the cube
    right_gripper.close()

    tuck()

    ####### PLACE ########
    print("placing next in queue...")
    tag_pos = lookup_tag(prev_marker)
    robot_trajectory = get_trajectory(limb, kin, ik_solver, tag_pos, num_way, task='queue')
    plan = planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[0].positions)
    planner.execute_plan(plan[1])
    planner.execute_plan(robot_trajectory)
    # Place the cube
    right_gripper.open()

    print("finished...")


if __name__ == "__main__":
    # call pick from arg1 tag and place next to arg 2 tag
    pick_and_place(17, 16)
