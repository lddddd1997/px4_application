##Task start script
export uav_id=1
gnome-terminal --window --geometry=80x24+0+0 -e 'bash -c "roslaunch px4_application base_sim_single_vehicle_ground_truth.launch uav_id:=$uav_id; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_application base_uav_control.launch uav_id:=$uav_id; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_application base_uav_estimator.launch uav_id:=$uav_id; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_application app_collaboration.launch uav_id:=$uav_id; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_application base_gcs_display.launch uav_id:=$uav_id; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_application base_gcs_setting.launch uav_id:=$uav_id; exec bash"' \