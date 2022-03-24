##Task start script
# Simulation "udp://:14540@127.0.0.1:14557"
# Usb        "/dev/ttyACM0:57600"
# Nano       "/dev/ttyTHS1:921600"
# TX2        "/dev/ttyTHS2:921600"
# 无人机序号从1～5


# 根据实际摆放修改 app_visual_formation.yaml的门框num和各个door位置
# 可能需要修改status_subscriber.h中的相机偏移和door_detection.cpp中的面积比率阈值
# 可能需要修改status_subscriber.h中的门框前后点偏移和app_visual_formation.cpp中的前向路径点偏移
# 可选择性修改轨迹生成的最大速度与最大加速度
# plotjuggler采集无人机位置,速度,轨迹控制器输入,门框位姿waypoint，bound_condition

export uav_id=1
export fcu_url="/dev/ttyTHS2:921600"
gnome-terminal --window -e 'bash -c "roslaunch px4_application base_uav_mavros_connection.launch uav_id:=$uav_id fcu_url:=$fcu_url; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch px4_application base_uav_control.launch uav_id:=$uav_id; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch px4_application app_visual_formation.launch uav_id:=$uav_id; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch px4_application base_traj_publisher.launch uav_id:=$uav_id; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch px4_application det_door.launch uav_id:=$uav_id; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch px4_application base_gcs_display.launch uav_id:=$uav_id; exec bash"' \
--tab -e 'bash -c "sleep 10; rosrun plotjuggler plotjuggler; exec bash"' \
