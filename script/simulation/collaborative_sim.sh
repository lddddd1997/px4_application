##sitl_gazebo
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch px4 mavros_posix_sitl.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch px4_application base_uav_control.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch px4_application app_uav_collaboration.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch px4_application base_gcs_display.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch px4_application base_gcs_setting.launch; exec bash"' \