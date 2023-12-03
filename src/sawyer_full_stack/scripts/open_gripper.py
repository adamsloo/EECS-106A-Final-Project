from intera_interface import gripper as robot_gripper
import rospy

if __name__ == "__main__":
    print("hello")
    rospy.init_node("hi")

    right_gripper = robot_gripper.Gripper('right_gripper')

    # Calibrate the gripper (other commands won't work unless you do this first)
    print('Calibrating...')
    right_gripper.calibrate()
    rospy.sleep(1)

    right_gripper.open()


