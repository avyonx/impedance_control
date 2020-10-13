# impedance_control

## Simulation startup
```
roscore
roslaunch wp_manipulator_3rx planning_context.launch
roslaunch impedance_control impedance_control_asap_3rx.launch
roslaunch plane_detection_ros plane_detection_wall.launch
rosrun impedance_control wall_planner_node __ns:=uav 
```

## Wall planning
```
rosservice call /uav/force_filter/zero_all "{}"
rosservice call /uav/aerial_manipulator_control/start "data: true"
rostopic pub /uav/aerial_manipulator_control/pose_stamped_ref_input geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 2.0
    y: 2.0
    z: 2.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
rosservice call /uav/plan_wall_path "data: True"
rosservice call /uav/execute_wall_path "{}"
```
