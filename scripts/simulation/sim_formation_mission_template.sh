##Task start script
gnome-terminal --window --geometry=80x24+0+0 -e 'bash -c "roslaunch px4_application base_sim_multi_uav_ground_truth.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_uav_control.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_uav_estimator.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application app_mission_template.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_gcs_display.launch uav_id:=1; exec bash"' \
gnome-terminal --window --geometry=80x24+1200+0 -e 'bash -c "sleep 6; roslaunch px4_application base_uav_control.launch uav_id:=2; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_uav_estimator.launch uav_id:=2; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application app_mission_template.launch uav_id:=2; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_gcs_display.launch uav_id:=2; exec bash"' \
gnome-terminal --window --geometry=80x24+0+700 -e 'bash -c "sleep 6; roslaunch px4_application base_uav_control.launch uav_id:=3; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_uav_estimator.launch uav_id:=3; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application app_mission_template.launch uav_id:=3; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_gcs_display.launch uav_id:=3; exec bash"' \
gnome-terminal --window --geometry=80x24+1200+700 -e 'bash -c "sleep 6; roslaunch px4_application base_uav_control.launch uav_id:=4; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_uav_estimator.launch uav_id:=4; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application app_mission_template.launch uav_id:=4; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_gcs_display.launch uav_id:=4; exec bash"' \
gnome-terminal --window --geometry=80x24+600+320 -e 'bash -c "sleep 6; roslaunch px4_application base_uav_control.launch uav_id:=5; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_uav_estimator.launch uav_id:=5; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application app_mission_template.launch uav_id:=5; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_gcs_display.launch uav_id:=5; exec bash"' \