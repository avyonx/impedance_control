
roslaunch franka_position_servo joint_position_controller.launch robot_ip:=172.16.0.2 load_gripper:=true

roslaunch panda_moveit_config panda_moveit.launch load_gripper:=true

rosrun manipulator_moveit_control wp_manipulator_control 

