export UAV_NAMESPACE=red
export PIX_SYM=/dev/ttyUSB_px4:921600

# Camera parameters
export CAMERA_NAME=camera
export CAMERA_LINK=$UAV_NAMESPACE/$CAMERA_NAME

export SENSOR=t265
export ODOM_TOPIC=$CAMERA_NAME/odom/sample
export CONTROL_PARAMS=custom_config/t265_position_control.params.yaml
export RC_MAPPING=custom_config/rc_mapping_tukan.yaml
