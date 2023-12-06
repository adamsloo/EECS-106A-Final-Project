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
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_interface import Limb

from utils.utils import *

from trac_ik_python.trac_ik import IK

import rospy
import tf2_ros
import intera_interface
from moveit_msgs.msg import DisplayTrajectory, RobotState
from sawyer_pykdl import sawyer_kinematics

from intera_interface import gripper as robot_gripper

import time

from song_queue.srv import MoveCubeRequest

prev_ar_tag_position = {}

def tuck():
    try:
        limb = Limb()
        traj = MotionTrajectory(limb = limb)

        wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=0.5,
                                         max_joint_accel=0.5)
        waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

        joint_angles = limb.joint_ordered_angles()

        waypoint.set_joint_angles(joint_angles = [0, -1, 0, 1.5, 0, -0.5, 1.7])
        traj.append_waypoint(waypoint.to_msg())

        waypoint.set_joint_angles(joint_angles = [0, -1, 0, 1.5, 0, -0.5, 1.7])
        traj.append_waypoint(waypoint.to_msg())

        result = traj.send_trajectory(timeout=None)
        if result is None:
            rospy.logerr('Trajectory FAILED to send')
            return

        if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)
    except rospy.ROSInterruptException:
        rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')

def lookup_tag(tag_number1, tag_number2):
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
    global prev_ar_tag_position
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    try:
        # TODO: lookup the transform and save it in trans
        trans1 = tfBuffer.lookup_transform("base", f"ar_marker_{tag_number1}", rospy.Time(0), rospy.Duration(10.0))
        tag_pos2 = None
        print("rev ar tag")
        print(prev_ar_tag_position)
        if tag_number2 in prev_ar_tag_position.keys():
            print(" in first if statement")
            tag_pos2 = prev_ar_tag_position[tag_number2]
            print(tag_pos2)
        else:
            print(" in second if statement")
            trans2 = tfBuffer.lookup_transform("base", f"ar_marker_{tag_number2}", rospy.Time(0), rospy.Duration(10.0))
            tag_pos2 = [getattr(trans2.transform.translation, dim) for dim in ('x', 'y', 'z')]
            print(tag_pos2)

        # The rospy.Time(0) is the latest available 
        # The rospy.Duration(10.0) is the amount of time to wait for the transform to be available before throwing an exception
    except Exception as e:
        print(e)
        print("Retrying ...")

    tag_pos1 = [getattr(trans1.transform.translation, dim) for dim in ('x', 'y', 'z')]
    print("look up tag")
    print(tag_pos1)
    print(tag_pos2)
    

    return np.array(tag_pos1), np.array(tag_pos2)

def get_trajectory(limb, kin, ik_solver, tag_pos, num_way, task, ar_tag, z=-0.002, y_offset=0.0029, x_offset = 0.001 ):
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
    x_offset = 0
    y_offset = 0

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
        print("line")
        target_pos = tag_pos
        target_pos[2] = z + 0.4 #linear path moves to a Z position above AR Tag. CHANGE THIS TO 0.4 IT IS SCARY!!!!!!!
        target_pos[1] += y_offset
        print("TARGET POSITION:", target_pos)
        trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=3)
    elif task == 'adjustment':
        print("adjustment")
        target_pos = current_position
        target_pos[2] = z
        print("TARGET POSITION ADJUSTMENT:", target_pos)
        trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=3)
    elif task == 'queue':
        print("queue")
        print(tag_pos)
        target_pos = tag_pos
        target_pos[2] = z
        target_pos[1] -= 0.08 + y_offset
        print("TARGET POSITION:", target_pos)
        trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=3)
        global prev_ar_tag_position
        prev_ar_tag_position[ar_tag] = target_pos
        
    else:
        raise ValueError('task {} not recognized'.format(task))
    
    path = MotionPath(limb, kin, ik_solver, trajectory)
    return path.to_robot_trajectory(num_way, True)

def main():
    rospy.init_node('moveit_node')
    move_cube_service = rospy.Service('/move_cube', MoveCubeRequest, handle_move_cube_request)
    rospy.spin()

def handle_move_cube_request(request):
    print("in handle move cube request")
    move_cube(request.ar_tag, request.prev_ar_tag)
    return True


def move_cube(marker, prev_marker, task='line', rate=200, timeout=None, num_way=50):
    # Marker should be a single ar marker string
    tuck()
    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper')

    # Calibrate the gripper (other commands won't work unless you do this first)
    print('Calibrating...')
    right_gripper.calibrate()
    rospy.sleep(1.0)
    
    # this is used for sending commands (velocity, torque, etc) to the robot
    ik_solver = IK("base", "right_gripper_tip")
    limb = intera_interface.Limb("right")
    kin = sawyer_kinematics("right")

    # Lookup the AR tag position.
    tag_pos1, tag_pos2 = lookup_tag(marker, prev_marker)
    
    # This is a wrapper around MoveIt! for you to use.  We use MoveIt! to go to the start position
    # of the trajectory
    planner = PathPlanner('right_arm')
    
    # ####### PICK ########
    robot_trajectory = get_trajectory(limb, kin, ik_solver, tag_pos1, num_way, task, ar_tag=marker)
    # Move to the trajectory start position
    plan = planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[0].positions)
    planner.execute_plan(plan[1])

    # try:
    #     input('Press <Enter> to execute the trajectory using MOVEIT')
    # except KeyboardInterrupt:
    #     sys.exit()

    # Uses MoveIt! to execute the trajectory.
    print("opening right gripper...")
    right_gripper.open()
    planner.execute_plan(robot_trajectory)

    # Adjusting to get closer to the cube
    robot_trajectory = get_trajectory(limb, kin, ik_solver, tag_pos2, num_way, task='adjustment', ar_tag=marker)
    plan = planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[0].positions)
    planner.execute_plan(plan[1])
    planner.execute_plan(robot_trajectory)
    #rospy.sleep(1.0)
    # Grab the cube
    right_gripper.close()
    tuck()

    ####### PLACE ########
    print("placing next in queue...")
    robot_trajectory = get_trajectory(limb, kin, ik_solver, tag_pos2, num_way, task='queue', ar_tag=marker)
    plan = planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[0].positions)
    planner.execute_plan(plan[1])
    planner.execute_plan(robot_trajectory)
    right_gripper.open()

    print("finished...")


if __name__ == "__main__":
    # call pick from arg1 tag and place next to arg 2 tag
    main()
    # rospy.init_node('moveit_node')
    # move_cube(9,11)
