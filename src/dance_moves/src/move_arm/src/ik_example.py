#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys

def main():
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    while not rospy.is_shutdown():
        input('Press [ Enter ]: ')
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = "right_gripper_tip"

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"

        position_1 = [0.879, -0.462, 0.932, 0.104, 0.426, -0.414, 0.798]
        position_2 = [0.890, 0.107, 0.221, -0.023, 0.967, 0.008, 0.254]
        position_3 = [0.727, 0.648, 0.928, -0.214, 0.310, 0.132, 0.917]

        desired_pos = position_1

        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = desired_pos[0]
        request.ik_request.pose_stamped.pose.position.y = desired_pos[1]
        request.ik_request.pose_stamped.pose.position.z = desired_pos[2]       
        request.ik_request.pose_stamped.pose.orientation.x = desired_pos[3]
        request.ik_request.pose_stamped.pose.orientation.y = desired_pos[4]
        request.ik_request.pose_stamped.pose.orientation.z = desired_pos[5]
        request.ik_request.pose_stamped.pose.orientation.w = desired_pos[6]
        
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
