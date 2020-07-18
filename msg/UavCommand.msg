std_msgs/Header header

float64 x
float64 y
float64 z
float32 yaw
float64 period #指令周期
bool update
string task_name

uint8 xyz_id #控制ID
uint8 PX_PY_PZ = 0
uint8 VX_VY_VZ = 1
uint8 VX_VY_PZ = 2
uint8 VX_PY_VZ = 3
uint8 PX_VY_VZ = 4
uint8 PX_PY_VZ = 5
uint8 UX_UY_UZ = 6 #不控

uint8 yaw_id #航向控制ID
uint8 NO_YAW = 0
uint8 YAW = 1
uint8 YAW_RATE = 2

uint8 frame_id #坐标系ID  PX4： NED  MAVROS： ENU
uint8 LOCAL = 0 #常用
uint8 BODY = 1 #一般用于机体系的速度控制  向前Y+ 向右X+ 向上Z+
