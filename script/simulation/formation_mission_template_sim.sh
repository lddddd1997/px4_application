##Task start script
gnome-terminal --window -e 'bash -c "roslaunch px4_application base_uav_formation_sim.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_application base_uav_control.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_application app_uav_mission_template.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_gcs_display.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_gcs_setting.launch uav_id:=1; exec bash"' \
gnome-terminal --window -e 'bash -c "sleep 5; roslaunch px4_application base_uav_control.launch uav_id:=2; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_application app_uav_mission_template.launch uav_id:=2; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_gcs_display.launch uav_id:=2; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_gcs_setting.launch uav_id:=2; exec bash"' \
gnome-terminal --window -e 'bash -c "sleep 5; roslaunch px4_application base_uav_control.launch uav_id:=3; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_application app_uav_mission_template.launch uav_id:=3; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_gcs_display.launch uav_id:=3; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_gcs_setting.launch uav_id:=3; exec bash"' \
gnome-terminal --window -e 'bash -c "sleep 5; roslaunch px4_application base_uav_control.launch uav_id:=4; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_application app_uav_mission_template.launch uav_id:=4; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_gcs_display.launch uav_id:=4; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_gcs_setting.launch uav_id:=4; exec bash"' \
gnome-terminal --window -e 'bash -c "sleep 5; roslaunch px4_application base_uav_control.launch uav_id:=5; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_application app_uav_mission_template.launch uav_id:=5; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_gcs_display.launch uav_id:=5; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_gcs_setting.launch uav_id:=5; exec bash"' \