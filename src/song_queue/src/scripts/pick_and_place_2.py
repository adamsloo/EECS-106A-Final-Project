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

class PickAndPlace:
    def __init__(self):
        self.ik_solver = IK("base", "right_gripper_tip")
        self.limb = intera_interface.Limb("right")
        self.kin = sawyer_kinematics("right")

        self.tuck_limb = Limb()
        self.wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=0.5,
                                            max_joint_accel=0.5)

        self.prev_ar_tag_position = {}
        self.planner = PathPlanner('right_arm')
        self.right_gripper = robot_gripper.Gripper('right_gripper')

    def tuck(self):
        try:
            traj = MotionTrajectory(limb = self.tuck_limb)
            waypoint = MotionWaypoint(options = self.wpt_opts.to_msg(), limb = self.limb)
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

    def audience_tuck(self):
        try:
            traj = MotionTrajectory(limb = self.tuck_limb)

            waypoint = MotionWaypoint(options = self.wpt_opts.to_msg(), limb = self.tuck_limb)
            waypoint.set_joint_angles(joint_angles = [0, -1, 0, 1.5, 0, -2, 1.7])
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

    def lookup_tag(self, tag_number1, tag_number2):
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

        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)

        try:
            trans1 = tfBuffer.lookup_transform("base", f"ar_marker_{tag_number1}", rospy.Time(0), rospy.Duration(10.0))
            tag_pos2 = None
            if tag_number2 in self.prev_ar_tag_position.keys():
                tag_pos2 = self.prev_ar_tag_position[tag_number2]
            else:
                trans2 = tfBuffer.lookup_transform("base", f"ar_marker_{tag_number2}", rospy.Time(0), rospy.Duration(10.0))
                tag_pos2 = [getattr(trans2.transform.translation, dim) for dim in ('x', 'y', 'z')]

        except Exception as e:
            print(e)
            print("Retrying ...")

        tag_pos1 = [getattr(trans1.transform.translation, dim) for dim in ('x', 'y', 'z')]
        return np.array(tag_pos1), np.array(tag_pos2)

    def get_trajectory(self, tag_pos, num_way, task, ar_tag, z=-0.002, y_offset=0.0029, x_offset = 0.001 ):
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

        if task == 'line':
            print("line")
            target_pos = tag_pos
            target_pos[2] = z + 0.4 #linear path moves to a Z position above AR Tag. CHANGE THIS TO 0.4 IT IS SCARY!!!!!!!
            target_pos[1] += y_offset
            print("TARGET POSITION:", target_pos)
            trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=4)
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
            target_pos[2] = z + 0.4
            target_pos[1] -= 0.08 + y_offset
            print("TARGET POSITION:", target_pos)
            trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=4)
            global prev_ar_tag_position
            self.prev_ar_tag_position[ar_tag] = target_pos
        else:
            raise ValueError('task {} not recognized'.format(task))
        
        path = MotionPath(self.limb, self.kin, self.ik_solver, trajectory)
        return path.to_robot_trajectory(num_way, True)

    def get_push_trajectory(self, tag_pos, num_way, task, ar_tag, z=0.000, y_offset=0.4, x_offset = 0.001):
        # y offset represents how far away from the cube we start off as
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        try:
            trans = tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0), rospy.Duration(10.0))
        except Exception as e:
            print(e)

        current_position = np.array([getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')])
        print("Current Position:", current_position)

        if task == 'start':
            print("start")
            target_pos = tag_pos
            target_pos[2] = z + 0.3
            target_pos[1] += y_offset
            print("TARGET POSITION:", target_pos)
            trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=4)
        elif task == 'adjustment':
            print("adjustment")
            target_pos = current_position
            target_pos[2] = z
            print("TARGET POSITION ADJUSTMENT:", target_pos)
            trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=3)
        elif task == 'end':
            print("end")
            print(tag_pos)
            target_pos = tag_pos
            target_pos[2] = z
            target_pos[1] += -2 * y_offset
            print("TARGET POSITION:", target_pos)
            trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=4)
        else:
            raise ValueError('task {} not recognized'.format(task))
        
        path = MotionPath(self.limb, self.kin, self.ik_solver, trajectory)
        return path.to_robot_trajectory(num_way, True)

    def handle_move_cube_request(self, request):
        print("in handle move cube request")
        if request.prev_ar_tag == -1:
            self.move_cube_now_playing(request.ar_tag)
        else:
            self.move_cube(request.ar_tag, request.prev_ar_tag)
        
        return True

    def move_cube_now_playing(self, marker, task='line', rate=200, timeout=None, num_way=50):
        self.tuck()
        # Lookup the AR tag position of currently playing cube.
        tag_pos1 = self.lookup_tag(marker)
        
        
        # ####### POSITION ########
        # Move to the push start position behind cube
        robot_trajectory = self.get_push_trajectory(tag_pos1, num_way, task = "start", ar_tag=marker)
        plan = self.planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[0].positions)
        self.planner.execute_plan(plan[1])
        self.planner.execute_plan(robot_trajectory)

        # Moving downwards
        robot_trajectory = self.get_push_trajectory(tag_pos1, num_way, task='adjustment', ar_tag=marker)
        plan = self.planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[0].positions)
        self.planner.execute_plan(plan[1])
        self.planner.execute_plan(robot_trajectory)
    
        ####### PUSH ########
        # Pushing cube forward
        print("pushing now playing forward...")
        robot_trajectory = self.get_push_trajectory(tag_pos1, num_way, task='end', ar_tag=marker)
        plan = self.planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[0].positions)
        self.planner.execute_plan(plan[1])
        self.planner.execute_plan(robot_trajectory)

        print("finished...")
        print("tucking to see audience")
        self.audience_tuck()
        return True

    def move_cube(self, marker, prev_marker, task='line', rate=200, timeout=None, num_way=50):
        # Marker should be a single ar marker string
        self.tuck()

        # Calibrate the gripper (other commands won't work unless you do this first)
        print('Calibrating...')
        self.right_gripper.calibrate()
        
        # Lookup the AR tag position.
        tag_pos1, tag_pos2 = self.lookup_tag(marker, prev_marker)
        
        # ####### PICK ########
        robot_trajectory = self.get_trajectory(tag_pos1, num_way, task, ar_tag=marker)
        # Move to the trajectory start position
        plan = self.planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[0].positions)
        self.planner.execute_plan(plan[1])
        self.planner.execute_plan(robot_trajectory)

        print("opening right gripper...")
        self.right_gripper.open()

        # Adjusting to get closer to the cube
        robot_trajectory = self.get_trajectory(tag_pos2, num_way, task='adjustment', ar_tag=marker)
        plan = self.planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[0].positions)
        self.planner.execute_plan(plan[1])
        self.planner.execute_plan(robot_trajectory)
        self.right_gripper.close()
        self.tuck()

        ####### PLACE ########
        print("placing next in queue...")
        robot_trajectory = self.get_trajectory(tag_pos2, num_way, task='queue', ar_tag=marker)
        plan = self.planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[0].positions)
        self.planner.execute_plan(plan[1])
        self.planner.execute_plan(robot_trajectory)

        robot_trajectory = self.get_trajectory(tag_pos2, num_way, task='adjustment', ar_tag=marker)
        self.planner.execute_plan(plan[1])
        self.planner.execute_plan(robot_trajectory)
        self.right_gripper.open()

        print("finished...")
        print("tucking to see audience")
        self.audience_tuck()
        return True

def main():
    rospy.init_node('moveit_node')
    instance = PickAndPlace()
    move_cube_service = rospy.Service('/move_cube', MoveCubeRequest, instance.handle_move_cube_request)
    rospy.spin()

if __name__ == "__main__":
    main()
