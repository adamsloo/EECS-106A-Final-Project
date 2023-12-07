ROS Package for EECS C106A Lab 7: Full Stack Robotics

to run;

source ~ee106a/sawyer_setup.bash
source devel/setup.bash

rosrun intera_interface enable_robot.py -e
rosrun intera_interface joint_trajectory_action_server.py
roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
roslaunch sawyer_full_stack sawyer_camera_track.launch

7F0MuIk5glqtowCUjbn9es

rosrun song_queue decision_maker.py
rosrun song_qpick_and_place.py
rosru




rosrun song_queue remaining_time.py
rosrun song_queue request_taker.py
rosrun song_queue decision_maker.py
rosrun song_queue pick_and_place.py
rosrun song_queue audience_controller.py
