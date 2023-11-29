"""
Recieves ar tracker number to move and moves it to the next spot in the line. 
Notes for implementation:
- keep track of previous ar tag to put this one behind it
"""

from re import L
import rospy
from std_msgs.msg import String, Float32
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

class MoveTag:
    def __init__(self, start_ar_tag):
        rospy.init_node('moveit_node')

        self.prev_ar_tag = start_ar_tag
        #currently hardcoded the ar tag number"
        #rospy.Subscriber('/curr_tag_processing', String, main)
        data = Float32()
        data.data = 0.0
        self.main(data)
        rospy.spin()

    def get_next_pos(self, ar_tag_pos):
        """
        Returns next position in line//figure out how much to add to whichever coord of the tag
        """
        ... 

    def tuck(self):
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
            
    def lookup_tag(self, tag_number):
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
            trans = tfBuffer.lookup_transform("base", f"ar_marker_{tag_number}", rospy.Time(0), rospy.Duration(10.0))
            # The rospy.Time(0) is the latest available 
            # The rospy.Duration(10.0) is the amount of time to wait for the transform to be available before throwing an exception
        except Exception as e:
            print(e)
            print("Retrying ...")

        tag_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
        return np.array(tag_pos)

    # def get_trajectory(self, limb, kin, ik_solver, tag_pos, num_way, task):
    #     """
    #     Returns an appropriate robot trajectory for the specified task.  You should 
    #     be implementing the path functions in paths.py and call them here
        
    #     Parameters
    #     ----------
    #     task : string
    #         name of the task.  Options: line, circle, square
    #     tag_pos : 3x' :obj:`numpy.ndarray`
            
    #     Returns
    #     -------
    #     :obj:`moveit_msgs.msg.RobotTrajectory`
    #     """

    #     tfBuffer = tf2_ros.Buffer()
    #     listener = tf2_ros.TransformListener(tfBuffer)

    #     try:
    #         trans = tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0), rospy.Duration(10.0))
    #     except Exception as e:
    #         print(e)

    #     current_position = np.array([getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')])
    #     print("Current Position:", current_position)

    #     if task == 'line':
    #         target_pos = self.get_next_pos(self.lookup_tag(self.prev_ar_tag))
    #         target_pos[2] += 0.05 #linear path moves to a Z position above AR Tag. CHANGE THIS TO 0.4 IT IS SCARY!!!!!!!
    #         print("TARGET POSITION:", target_pos)
    #         trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=9)
        
    #     path = MotionPath(limb, kin, ik_solver, trajectory)
    #     return path.to_robot_trajectory(num_way, True)


    def main(self, data):
        rospy.loginfo("Received data: %s", data.data)
        ar_tag = data.data
        
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
        tag_pos = self.lookup_tag(ar_tag)

        # Get an appropriate RobotTrajectory for the task (circular, linear, or square)
        # If the controller is a workspace controller, this should return a trajectory where the
        # positions and velocities are workspace positions and velocities.  If the controller
        # is a jointspace or torque controller, it should return a trajectory where the positions
        # and velocities are the positions and velocities of each joint.
        # TODO
        #robot_trajectory = get_trajectory(limb, kin, ik_solver, tag_pos, args)
        robot_trajectory = get_trajectory(limb, kin, ik_solver, tag_pos, num_way=50, task ='line')

        # This is a wrapper around MoveIt! for you to use.  We use MoveIt! to go to the start position
        # of the trajectory
        planner = PathPlanner('right_arm')
        
        # By publishing the trajectory to the move_group/display_planned_path topic, you should 
        # be able to view it in RViz.  You will have to click the "loop animation" setting in 
        # the planned path section of MoveIt! in the menu on the left side of the screen.
        trajectory_pub = rospy.Publisher('move_group/display_planned_path', DisplayTrajectory, queue_size=10)
        disp_traj = DisplayTrajectory()
        disp_traj.trajectory.append(robot_trajectory)
        disp_traj.trajectory_start = RobotState()
        trajectory_pub .publish(disp_traj)

        # Move to the trajectory start position
        try:
            input('Press <Enter> to execute the trajectory using MOVEIT')
        except KeyboardInterrupt:
            sys.exit()
        # Uses MoveIt! to execute the trajectory.
        print("opening right gripper...")
        right_gripper.open()
        planner.execute_plan(robot_trajectory)
        right_gripper.close()
        rospy.init_node('moveit_node')
        tuck()

if __name__ == "__main__":
    try:
        instance = MoveTag()
    except rospy.ROSInterruptException:
        pass
    