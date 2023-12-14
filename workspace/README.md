# Final Project Set-up command

### Connect to sawyer
source ~ee106a/sawyer_setup.bash

### Enable robot
rosrun intera_interface enable_robot.py -e

### Move the robot to the right start position
roslaunch intera_examples sawyer_tuck.launch

roslaunch sawyer_full_stack custom_sawyer_tuck.launch

### Get the current position of the gripper
rosrun tf tf_echo base right_gripper_tip

### Start joint trajectory controller
rosrun intera_interface joint_trajectory_action_server.py

### Start MoveIt
roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true

### Execute function 
rosrun move_arm ik_example_1.py

rosrun move_arm main_move_it.py

### Test gripper
rosrun move_arm gripper_test.py

### Launch ar_track
roslaunch sawyer_full_stack sawyer_camera_track.launch
