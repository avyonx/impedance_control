#!/bin/bash
namespace=${1:-uav}

echo "Calling service to zero force sensor measurements."
rosservice call /$namespace/force_filter/zero_all "{}"
sleep 2
echo "Calling service to start manipulator control."
rosservice call /$namespace/aerial_manipulator_control/start "data: true"
sleep 2
echo "Publishing takeoff pose."
#rostopic pub -1 /$namespace/aerial_manipulator_control/pose_stamped_ref_input geometry_msgs/PoseStamped "header:
#  seq: 0
#  stamp:
#    secs: 0
#    nsecs: 0
#  frame_id: ''
#pose:
#  position:
#    x: 2.0
#    y: 2.0
#    z: 2.0
#  orientation:
#    x: 0.0
#    y: 0.0
#    z: 0.0
#    w: 1.0"
rostopic pub -1 /$namespace/aerial_manipulator_control/trajectory_ref_input trajectory_msgs/JointTrajectoryPoint "positions: [1.37,2.2,2.0,0,0,0,0,0,0,0,0,0,0,0,0,1]                            
velocities: [0]
accelerations: [0]
effort: [0]
time_from_start: {secs: 0, nsecs: 0}"
# Wait to get airborne and then zero forces
sleep 8
echo "Calling service to zero force sensor measurements."
rosservice call /$namespace/force_filter/zero_all "{}"

#rosservice call /$namespace/plan_wall_path "data: True"
#rosservice call /$namespace/execute_wall_path "{}"

