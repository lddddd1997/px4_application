##Task start script
export uav_id=1
gnome-terminal --window -e 'bash -c "roslaunch px4_application base_uav_mavros_sim.launch uav_id:=$uav_id; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch px4_application base_uav_control.launch uav_id:=$uav_id; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch px4_application app_uav_collaboration.launch uav_id:=$uav_id; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch px4_application base_gcs_display.launch uav_id:=$uav_id; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch px4_application base_gcs_setting.launch uav_id:=$uav_id; exec bash"' \