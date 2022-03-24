##Task start script
gnome-terminal --window --geometry=80x24+0+0 -e 'bash -c "roslaunch px4_application base_sim_double_uav_ground_truth.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch px4_application base_uav_control.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch px4_application app_visual_formation.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch px4_application base_traj_publisher.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch px4_application base_gcs_display.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 10; rosrun plotjuggler plotjuggler; exec bash"' \