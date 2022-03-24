gnome-terminal --window --geometry=80x24+0+0 -e 'bash -c "roslaunch px4_application base_sim_double_uav_ground_truth.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_application base_uav_control.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_application app_visual_formation.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_application base_traj_publisher.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_application det_door.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_application base_gcs_display.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun image_view image_view image:=/uav1/detection_status/image; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_application rviz_visualization.launch; exec bash"' \