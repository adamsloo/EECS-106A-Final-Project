#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
from intera_interface import Limb
import tf2_ros

import intera_interface
import sys
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)

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

def lookup_tag(tag_number1):
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
        # The rospy.Time(0) is the latest available 
        # The rospy.Duration(10.0) is the amount of time to wait for the transform to be available before throwing an exception
    except Exception as e:
        print(e)
        print("Retrying ...")

    tag_pos1 = [getattr(trans1.transform.translation, dim) for dim in ('x', 'y', 'z')]
    return np.array(tag_pos1)


def main():
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    while not rospy.is_shutdown():
        tuck()
        tag_pos = lookup_tag(11)
        target_pos = tag_pos
        target_pos[2] = -0.05  #linear path moves to a Z position above AR Tag. CHANGE THIS TO 0.4 IT IS SCARY!!!!!!!
        print(target_pos)
        
        input('Press [ Enter ]: ')
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = "right_gripper_tip"

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"

        # position_1 = [0.879, -0.462, 0.932, 0.104, 0.426, -0.414, 0.798]
        # position_2 = [0.890, 0.107, 0.221, -0.023, 0.967, 0.008, 0.254]
        # position_3 = [0.727, 0.648, 0.928, -0.214, 0.310, 0.132, 0.917]

        desired_pos = target_pos

        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = desired_pos[0]
        request.ik_request.pose_stamped.pose.position.y = desired_pos[1]
        request.ik_request.pose_stamped.pose.position.z = desired_pos[2]       
        request.ik_request.pose_stamped.pose.orientation.x = 0
        request.ik_request.pose_stamped.pose.orientation.y = 1
        request.ik_request.pose_stamped.pose.orientation.z = 0
        request.ik_request.pose_stamped.pose.orientation.w = 0
        
        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            ###group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK
            plan = group.plan()
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            if user_input == 'y':
                group.execute(plan[1])
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

# Python's syntax for a main() method
if __name__ == '__main__':
    main()
