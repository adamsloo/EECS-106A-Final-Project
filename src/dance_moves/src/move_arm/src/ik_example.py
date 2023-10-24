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
        left_position = [0.453, -0.231, 0.98, 0.330, 0.539, -0.528, 0.567]
        right_position = [0.302, 0.46, 0.953, -0.521, 0.397, 0.084, 0.751]
        down_position = [0.860, 0.695, 0.570,-0.169, 0.760, 0.071, 0.624]
        up_position = [0.633, 0.664, 0.995,-0.143, 0.558, 0.179, 0.798]

        desired_position = left_position

        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = desired_position[0]
        request.ik_request.pose_stamped.pose.position.y = desired_position[1]
        request.ik_request.pose_stamped.pose.position.z = desired_position[2]     
        request.ik_request.pose_stamped.pose.orientation.x = desired_position[3]
        request.ik_request.pose_stamped.pose.orientation.y = desired_position[4]
        request.ik_request.pose_stamped.pose.orientation.z = desired_position[5]
        request.ik_request.pose_stamped.pose.orientation.w = desired_position[6]
        
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
