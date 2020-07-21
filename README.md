# impedance_control

```
roscore
roslaunch wp_manipulator planning_context.launch
roslaunch impedance_control impedance_control_asap.launch
rosservice call /uav/force_filter/zero_all "{}"
rosservice call /uav/impedance_control/start "data: true"
rostopic pub /uav/impedance_control/pose_stamped_ref_input ...
roslaunch plane_detection_ros plane_detection_wall.launch
```