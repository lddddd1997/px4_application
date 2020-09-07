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

uint8 frame_id #控制指令坐标系ID  PX4(Local: NED Body: Head X+  Right Y+  Down Z+)  MAVROS(Local: ENU (Firmware 1.9.2)Body: Head X+  Left Y+  Up Z+ (Firmware 1.10.1)Body: Head Y+  Right X+  Up Z+) 注：以1.9.2为基准
uint8 LOCAL = 0 #常用
uint8 BODY = 1 #用于机体系的速度或加速度控制
