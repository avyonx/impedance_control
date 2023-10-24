#!/bin/bash

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"

# remove the old link
rm .tmuxinator.yml

# link the session file to .tmuxinator.yml
ln session.yml .tmuxinator.yml

SETUP_NAME=$1
[ -z "$SETUP_NAME" ] && SETUP_NAME=tilt_control_optitrack_setup.sh

# start tmuxinator
tmuxinator uav_ros_indoor setup_name=$SETUP_NAME

