#!/bin/bash

. ~/.shell_scripts.sh

[ -z "$FORCE_X" ] && FORCE_X=2.0
[ -z "$FORCE_Y" ] && FORCE_Y=0.0
[ -z "$FORCE_Z" ] && FORCE_Z=0.0

[ -z "$TARGET_X" ] && TARGET_X=0.5
[ -z "$TARGET_Y" ] && TARGET_Y=0.685
[ -z "$TARGET_Z" ] && TARGET_Z=1.17

[ -z "$HOME_X" ] && HOME_X=-0.47
[ -z "$HOME_Y" ] && HOME_Y=0.8
[ -z "$HOME_Z" ] && HOME_Z=1.17

[ -z "$CONTACT_TIME" ] && CONTACT_TIME=30.0

echo "Touching target [$TARGET_X, $TARGET_Y, $TARGET_Z] with force [$FORCE_X, $FORCE_Y, $FORCE_Z]"

rosservice call /$UAV_NAMESPACE/position_hold
echo "Starting position hold, sleeping..."
sleep 2.0

rosservice call /$UAV_NAMESPACE/impedance_control/start true
echo "Starting impedance control, sleeping..."
sleep 2.0

rostopic pub --once /$UAV_NAMESPACE/tracker/input_pose geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: $TARGET_X
    y: $TARGET_Y
    z: $TARGET_Z
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: -1.0"

echo "Waiting for ACCEPT"
waitForTracker

echo "Publishing Force [$FORCE_X, $FORCE_Y, $FORCE_Z]"
rostopic pub --once /$UAV_NAMESPACE/impedance_control/force_torque_ref_input geometry_msgs/WrenchStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
wrench:
  force:
    x: $FORCE_X
    y: $FORCE_Y
    z: $FORCE_Z
  torque:
    x: 0.0
    y: 0.0
    z: 0.0" 

echo "Sleeping $CONTACT_TIME"
sleep $CONTACT_TIME

rostopic pub --once /$UAV_NAMESPACE/impedance_control/force_torque_ref_input geometry_msgs/WrenchStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
wrench:
  force:
    x: 0.0
    y: 0.0
    z: 0.0
  torque:
    x: 0.0
    y: 0.0
    z: 0.0" 

echo "Going home - [$HOME_X, $HOME_Y, $HOME_Z]"
rostopic pub --once /$UAV_NAMESPACE/tracker/input_pose geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: $HOME_X
    y: $HOME_Y
    z: $HOME_Z
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: -1.0"
