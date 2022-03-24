export record_topic="/uav2/detection_status/image /uav2/acc_visualization /uav2/detection_status/drone /uav2/mavros/local_position/pose /uav2/mavros/local_position/velocity_local /uav2/px4_application/uav_command /uav2/reference/flat_setpoint /uav2/yolo_detector/bounding_boxes"
gnome-terminal --window -e 'bash -c "rosbag record $record_topic --bz2 -b 0; exec bash"' \

