##Task start script
gnome-terminal --window --geometry=80x24+0+0 -e 'bash -c "roslaunch px4_application base_sim_multi_uav_ground_truth.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch px4_application base_uav_control.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch px4_application app_visual_formation.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch px4_application base_traj_publisher.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_application det_door.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch px4_application base_gcs_display.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun image_view image_view image:=/uav1/detection_status/image; exec bash"' \
gnome-terminal --window --geometry=80x24+1200+0 -e 'bash -c "sleep 5; roslaunch px4_application base_uav_control.launch uav_id:=2; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch px4_application app_visual_formation.launch uav_id:=2; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_application det_yolo_drone.launch uav_id:=2; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch px4_application base_gcs_display.launch uav_id:=2; exec bash"' \
gnome-terminal --window --geometry=80x24+0+700 -e 'bash -c "sleep 5; roslaunch px4_application base_uav_control.launch uav_id:=3; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch px4_application app_visual_formation.launch uav_id:=3; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_application det_yolo_drone.launch uav_id:=3; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch px4_application base_gcs_display.launch uav_id:=3; exec bash"' \
gnome-terminal --window --geometry=80x24+1200+700 -e 'bash -c "sleep 5; roslaunch px4_application base_uav_control.launch uav_id:=4; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch px4_application app_visual_formation.launch uav_id:=4; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_application det_yolo_drone.launch uav_id:=4; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch px4_application base_gcs_display.launch uav_id:=4; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_application rviz_visualization.launch; exec bash"' \