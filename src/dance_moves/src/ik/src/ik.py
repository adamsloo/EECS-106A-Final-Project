#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from intera_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

def ik_service_client():
    service_name = "ExternalTools/right/PositionKinematicsNode/IKService"
    ik_service_proxy = rospy.ServiceProxy(service_name, SolvePositionIK)
    ik_request = SolvePositionIKRequest()
    header = Header(stamp=rospy.Time.now(), frame_id='base')

    # Create a PoseStamped and specify header (specifying a header is very important!)
    pose_stamped = PoseStamped()
    pose_stamped.header = header

    user_input = input("Enter x y z, orientation: ")
    input_split = user_input.split(" ")
    x = float(input_split[0])
    y = float(input_split[1])
    z = float(input_split[2])
    qx = float(input_split[3])
    qy = float(input_split[4])
    qz = float(input_split[5])
    qw = float(input_split[6])
    # Set end effector position: YOUR CODE HERE

    pos = Point()
    pos.x = x
    pos.y = y
    pos.z = z
    # Set end effector quaternion: YOUR CODE HERE

    orient = Quaternion()
    orient.x = qx
    orient.y = qy  
    orient.z = qz
    orient.w = qw
    pose_stamped.pose.position = pos
    pose_stamped.pose.orientation = orient

    

    # Add desired pose for inverse kinematics
    ik_request.pose_stamp.append(pose_stamped)
    # Request inverse kinematics from base to "right_hand" link
    ik_request.tip_names.append('right_hand')

    rospy.loginfo("Running Simple IK Service Client example.")

    try:
        rospy.wait_for_service(service_name, 5.0)
        response = ik_service_proxy(ik_request)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call failed: %s" % (e,))
        return

    # Check if result valid, and type of seed ultimately used to get solution
    if (response.result_type[0] > 0):
        rospy.loginfo("SUCCESS!")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(list(zip(response.joints[0].name, response.joints[0].position)))
        rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
        rospy.loginfo("------------------")
        rospy.loginfo("Response Message:\n%s", response)
        print("Response print: \n", response)
    else:
        rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
        rospy.logerr("Result Error %d", response.result_type[0])
        return False

    return True


def main():
    rospy.init_node("ik_service_client")

    ik_service_client()

if __name__ == '__main__':
    main()
