##sitl_gazebo
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS1:921600"; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch px4_application app_uav_collaboration.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_application base_gcs_control.launch; exec bash"' \
#--tab -e 'bash -c "sleep 5; roslaunch px4_application base_gcs_setting.launch; exec bash"' \