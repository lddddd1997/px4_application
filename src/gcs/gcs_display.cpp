/** 
* @file     gcs_display.cpp
* @brief    无人机状态显示
* @details  
* @note
* @author   lddddd
*           Email: lddddd1997@gmail.com
*           Github: https://github.com/lddddd1997
* @date     2020.7.21
* @version  2.0
* @par      Edit history:
*           1.0: lddddd, 2020.5.24, .
*           2.0: lddddd, 2020.7.21, 更新节点句柄与topic的命名空间.
*/

#include "gcs_display.h"

void GcsDisplay::CommandCallback(const px4_application::UavCommand::ConstPtr& _msg)
{
    this->command_reception = *_msg;
}

float GcsDisplay::GetTimePassSec(void)
{
    ros::Time current_time = ros::Time::now();
    float current_time_sec = current_time.sec - this->begin_time.sec;
    float current_time_nsec = current_time.nsec / 1e9 - this->begin_time.nsec / 1e9;
    return (current_time_sec + current_time_nsec);
}

void GcsDisplay::CommandUpdateReset(void)
{
    this->command_reception.update = false;
}

void GcsDisplay::LoopTask(void)
{
    UavStateDisplay();
    // cout << "Virtual Loop Task of Derived Class !" << endl;
}

void GcsDisplay::Initialize(void)
{
    this->begin_time = ros::Time::now();
    this->current_info.uav_status.state.mode = "UNKNOWN";
    this->command_reception.task_name = "UNKNOWN";
    this->uav_command_sub = this->nh.subscribe<px4_application::UavCommand>("px4_application/uav_command",
                                                                             10,
                                                                              &GcsDisplay::CommandCallback,
                                                                               this,
                                                                                ros::TransportHints().tcpNoDelay());
}
void GcsDisplay::UavStateDisplay(void)
{
    std::cout << "---------------------------------State Info----------------------------------" << std::endl;
    /*固定的浮点显示*/
    std::cout.setf(std::ios::fixed);
    /*setprecision(n) 设显示小数精度为n位*/
    std::cout << std::setprecision(2);
    /*左对齐*/
    std::cout.setf(std::ios::right);
    /*强制显示小数点*/
    std::cout.setf(std::ios::showpoint);
    /*强制显示符号*/
    std::cout.setf(std::ios::showpos);

    std::cout << "Time:" << std::setw(8) << GetTimePassSec() << " [s] ";

    /*是否和飞控建立起连接*/
    std::cout << (this->current_info.uav_status.state.connected ? " [ Connected ] " : " [ Unconnected ] ");
    /*是否上锁*/
    std::cout << (this->current_info.uav_status.state.armed ? " [ Armed ] " : " [ DisArmed ] ");
    std::cout << " [ " << this->current_info.uav_status.state.mode <<" ] ";
    std::string flight_state;
    switch(this->current_info.uav_status.extended_state.landed_state)
    {
        case mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED: flight_state = "UNDEFINED"; break;
        case mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND: flight_state = "ON_GROUND"; break;
        case mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR: flight_state = "IN_FLIGHT"; break;
        case mavros_msgs::ExtendedState::LANDED_STATE_TAKEOFF: flight_state = "TAKEOFF"; break;
        case mavros_msgs::ExtendedState::LANDED_STATE_LANDING: flight_state = "LANDING"; break;
        default : flight_state = "UNDEFINED"; break;
    }

    std::cout << " [ " << flight_state << " ] " << std::endl;

    std::cout << "Estimated Status:  ";
    std::cout << "Attitude   " << (this->current_info.uav_status.estimator_status.attitude_status_flag ? "[√]   " : "[X]   ")
               << "Vel Horiz Rel " << (this->current_info.uav_status.estimator_status.velocity_horiz_status_flag ? "[√]   " : "[X]   ") 
                << "Vel Verti Rel " << (this->current_info.uav_status.estimator_status.velocity_vert_status_flag ? "[√]   " : "[X]   ") << std::endl;


    std::cout << "                   ";
    std::cout << "Accel      " << (this->current_info.uav_status.estimator_status.accel_error_status_flag ? "[X]   " : "[√]   ")
               << "Pos Verti Rel " << (this->current_info.uav_status.estimator_status.pos_vert_agl_status_flag ? "[√]   " : "[X]   ") 
                << "Pos Verti Abs " << (this->current_info.uav_status.estimator_status.pos_vert_abs_status_flag ? "[√]   " : "[X]   ") << std::endl;

    std::cout << "                   ";
    std::cout << "Gps Glitch " << (this->current_info.uav_status.estimator_status.gps_glitch_status_flag ? "[√]   " : "[X]   ")
               << "Pos Horiz Rel " << (this->current_info.uav_status.estimator_status.pos_horiz_rel_status_flag ? "[√]   " : "[X]   ") 
                << "Pos Horiz Abs " << (this->current_info.uav_status.estimator_status.pos_horiz_abs_status_flag ? "[√]   " : "[X]   ") << std::endl;

    std::cout << "                   ";
    std::cout << "Const Mode " << (this->current_info.uav_status.estimator_status.const_pos_mode_status_flag ? "[√]   " : "[X]   ")
               << "pre Horiz Rel " << (this->current_info.uav_status.estimator_status.pred_pos_horiz_rel_status_flag ? "[√]   " : "[X]   ") 
                << "pre Horiz Abs " << (this->current_info.uav_status.estimator_status.pred_pos_horiz_abs_status_flag ? "[√]   " : "[X]   ") << std::endl;

    const double RAD2DEG = 57.295779513082320876846364344191;
    std::string frame_name;
    if(this->command_reception.frame_id == px4_application::UavCommand::LOCAL)
    {
        frame_name = " [LOCAL]";
    }
    else if(this->command_reception.frame_id == px4_application::UavCommand::BODY)
    {
        frame_name = "  [BODY]";
    }
    else
    {
        frame_name = " [ERROR]";
    }
    int setw_num = 9;
    std::cout << "--------------------------------Attitude Info--------------------------------" << std::endl;
    std::cout << "Attitude Angle  [R P Y] : " 
               << std::setw(setw_num) << this->current_info.uav_status.attitude_angle.x * RAD2DEG << " [ ° ] " 
                << std::setw(setw_num) << this->current_info.uav_status.attitude_angle.y * RAD2DEG << " [ ° ] "
                 << std::setw(setw_num) << this->current_info.uav_status.attitude_angle.z * RAD2DEG << " [ ° ] " << std::endl;
    std::cout << "Attitude Rate   [R P Y] : "
               << std::setw(setw_num) << this->current_info.uav_status.attitude_rate.x  * RAD2DEG << " [°/s] "
                << std::setw(setw_num) << this->current_info.uav_status.attitude_rate.y  * RAD2DEG << " [°/s] "
                 << std::setw(setw_num) << this->current_info.uav_status.attitude_rate.z  * RAD2DEG << " [°/s] " << std::endl;
    std::cout << "-------------------------------Navigation Info-------------------------------" << std::endl;
    std::cout << "LOCAL Position  [X Y Z] : "
               << std::setw(setw_num) << this->current_info.uav_status.local_position.x << " [ m ] "
                << std::setw(setw_num) << this->current_info.uav_status.local_position.y << " [ m ] "
                 << std::setw(setw_num) << this->current_info.uav_status.local_position.z << " [ m ] " << std::endl;
    std::cout << "LOCAL Velocity  [X Y Z] : "
               << std::setw(setw_num) << this->current_info.uav_status.local_velocity.x << " [m/s] "
                << std::setw(setw_num) << this->current_info.uav_status.local_velocity.y << " [m/s] "
                 << std::setw(setw_num) << this->current_info.uav_status.local_velocity.z << " [m/s] " << std::endl;
    std::cout << "BODY  Velocity  [X Y Z] : "    //指机体坐标映射到的参考系的水平方向（即参考系到机体系，只旋转了Z轴）
               << std::setw(setw_num) << this->current_info.uav_status.body_heading_velocity.x << " [m/s] "
                << std::setw(setw_num) << this->current_info.uav_status.body_heading_velocity.y << " [m/s] "
                 << std::setw(setw_num) << this->current_info.uav_status.body_heading_velocity.z << " [m/s] " << std::endl;
    std::cout << "-------------------------------Command" << frame_name << "-------------------------------" << std::endl;
    std::cout << "Period: " << this->command_reception.period  << " [s] " << " [ " << this->command_reception.task_name << " ] ";

    std::cout << (this->command_reception.update ? " [ ACTIVE ] " : "  [ DEAD ]  ");
    switch(this->command_reception.xyz_id)
    {
        case px4_application::UavCommand::PX_PY_PZ:
        {
            std::cout << " [ PX_PY_PZ ] ";
            break;
        }
        case px4_application::UavCommand::VX_VY_VZ:
        {
            std::cout << " [ VX_VY_VZ ] ";
            break;
        }
        case px4_application::UavCommand::VX_VY_PZ:
        {
            std::cout << " [ VX_VY_PZ ] ";
            break;
        }
        case px4_application::UavCommand::VX_PY_VZ:
        {
            std::cout << " [ VX_PY_VZ ] ";
            break;
        }
        case px4_application::UavCommand::PX_VY_VZ:
        {
            std::cout << " [ PX_VY_VZ ] ";
            break;
        }
        case px4_application::UavCommand::PX_PY_VZ:
        {
            std::cout << " [ PX_PY_VZ ] ";
            break;
        }
        case px4_application::UavCommand::UX_UY_UZ:
        {
            std::cout << " [ UX_UY_UZ ] ";
            break;
        }
        default:     
        {
            std::cout << " [ ERROR ] ";
            break;
        }
    }
    switch(this->command_reception.yaw_id)
    {
        case px4_application::UavCommand::NO_YAW:
        {
            std::cout << " [ NO_YAW ] ";
            break;
        }
        case px4_application::UavCommand::YAW:
        {
            std::cout << " [ YAW ] ";
            break;
        }
        case px4_application::UavCommand::YAW_RATE:
        {
            std::cout << " [ YAW_RATE ] ";
            break;
        }
        default:     
        {
            std::cout << " [ ERROR ] ";
            break;
        }
    }
    std::cout << std::endl;
    std::cout << "Command Reception : "
               << std::setw(setw_num) << this->command_reception.x << " [X]"
                << std::setw(setw_num) << this->command_reception.y << " [Y]"
                 << std::setw(setw_num) << this->command_reception.z << " [Z]"
                  << std::setw(setw_num) << this->command_reception.yaw << " [Yaw]" << std::endl;
    std::cout << std::endl;

    CommandUpdateReset();
}

GcsDisplay::GcsDisplay(const ros::NodeHandle& _nh, double _period) : RosBase(_nh, _period)
{
    Initialize();
}
GcsDisplay::~GcsDisplay()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gcs_display");
    ros::NodeHandle nh;
    GcsDisplay GcsDisplay(nh, 0.1);

    ros::spin();
    return 0;

}

