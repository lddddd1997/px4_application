##Task start script
# Simulation "udp://:14540@127.0.0.1:14557"
# Usb        "/dev/ttyACM0:57600"
# Nano       "/dev/ttyTHS1:921600"
# TX2        "/dev/ttyTHS2:921600"
# 无人机序号从1～5
export uav_id=1
export fcu_url="/dev/ttyTHS1:921600"
gnome-terminal --window -e 'bash -c "roslaunch px4_application base_uav_mavros_connection.launch uav_id:=$uav_id fcu_url:=$fcu_url; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_uav_control.launch uav_id:=$uav_id; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application app_uav_collaboration.launch uav_id:=$uav_id; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch MonoCamera findObject.launch uav_id:=$uav_id; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_gcs_display.launch uav_id:=$uav_id; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch px4_application base_gcs_setting.launch uav_id:=$uav_id; exec bash"' \