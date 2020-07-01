##sitl_gazebo
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch px4 mavros_posix_sitl.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch px4_application uav.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; rosrun px4_application gcs_display; exec bash"' \
--tab -e 'bash -c "sleep 3; rosrun px4_application gcs_setting; exec bash"' \