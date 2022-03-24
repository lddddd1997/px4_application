export record_topic="/uav1/detection_status/image /uav1/detection_status/door /uav1/acc_visualization /uav1/mavros/local_position/pose /uav1/mavros/local_position/velocity_local /uav1/px4_application/uav_command /uav1/reference/boundary_conds/min_jerk /uav1/reference/flat_setpoint /uav1/waypoints_visualization /uav1/trajectory_visualization/min_jerk"
gnome-terminal --window -e 'bash -c "rosbag record $record_topic --bz2 -b 0; exec bash"' \

