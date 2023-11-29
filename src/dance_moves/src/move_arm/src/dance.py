#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from intera_interface import gripper as robot_gripper

def main():
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    right_gripper = robot_gripper.Gripper('right_gripper')

    position_1 = [0.879, -0.462, 0.932, 0.104, 0.426, -0.414, 0.798]
    position_2 = [0.890, 0.107, 0.221, -0.023, 0.967, 0.008, 0.254]
    position_3 = [0.727, 0.648, 0.928, -0.214, 0.310, 0.132, 0.917]
    is_safe = False
    skip_step = False
    end_program = False

    while not rospy.is_shutdown() and not end_program:
        input('Press [ Enter ]: ')
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = "right_gripper_tip"

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        desired_pos = position_2
        request.ik_request.pose_stamped.pose.position.x = desired_pos[0]
        request.ik_request.pose_stamped.pose.position.y = desired_pos[1]
        request.ik_request.pose_stamped.pose.position.z = desired_pos[2]
        request.ik_request.pose_stamped.pose.orientation.x = desired_pos[3]
        request.ik_request.pose_stamped.pose.orientation.y = desired_pos[4]
        request.ik_request.pose_stamped.pose.orientation.z = desired_pos[5]
        request.ik_request.pose_stamped.pose.orientation.w = desired_pos[6]
        
        try:
            while not is_safe and not skip_step and not end_program:
                # Send the request to the service
                response = compute_ik(request)
                
                # Print the response HERE
                print(response)
                group = MoveGroupCommander("right_arm")

                # Setting position and orientation target
                group.set_pose_target(request.ik_request.pose_stamped)

                # Plan IK
                plan = group.plan()
                user_input = input("Enter 'y' if the trajectory looks safe on RVIZ.\nEnter 's' if you want to skip this step.\nEnter 'e' if you want to end the program.\nEnter any other button to replan the path.\n")
                # Execute IK if safe
                if user_input == 'y':
                    is_safe = True
                    group.execute(plan[1])
                    rospy.sleep(1.0)
                elif user_input == 's':
                    skip_step = True
                elif user_input == 'e':
                    end_program = True
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        # END OF CONFIG

        all_positions = [position_3, position_2, position_1, position_2]
        counter = 0

        while not end_program:
            is_safe = False
            skip_step = False

            input('Press [ Enter ]: ')

            desired_pos = all_positions[counter]
            counter = (counter + 1) % len(all_positions)

            # Set the desired orientation for the end effector HERE            
            request.ik_request.pose_stamped.pose.position.x = desired_pos[0]
            request.ik_request.pose_stamped.pose.position.y = desired_pos[1]
            request.ik_request.pose_stamped.pose.position.z = desired_pos[2]
            request.ik_request.pose_stamped.pose.orientation.x = desired_pos[3]
            request.ik_request.pose_stamped.pose.orientation.y = desired_pos[4]
            request.ik_request.pose_stamped.pose.orientation.z = desired_pos[5]
            request.ik_request.pose_stamped.pose.orientation.w = desired_pos[6]
            
            try:
                while not is_safe and not skip_step and not end_program:

                    # Send the request to the service
                    response = compute_ik(request)
                    
                    # Print the response HERE
                    print(response)
                    group = MoveGroupCommander("right_arm")

                    # Setting position and orientation target
                    group.set_pose_target(request.ik_request.pose_stamped)

                    # Plan IK
                    plan = group.plan()
                    user_input = input("Enter 'y' if the trajectory looks safe on RVIZ.\nEnter 's' if you want to skip this step.\nEnter 'e' if you want to end the program.\nEnter any other button to replan the path.\n")
                    
                    # Execute IK if safe
                    if user_input == 'y':
                        is_safe = True
                        group.execute(plan[1])
                        rospy.sleep(1.0)
                    elif user_input == 's':
                        skip_step = True
                    elif user_input == 'e':
                        end_program = True
            except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)

# Python's syntax for a main() method
if __name__ == '__main__':
    main()
