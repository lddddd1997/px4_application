/** 
* @file     gcs_display.cpp
* @brief    无人机状态显示
* @details  
* @note
* @author   lddddd
*           Email: lddddd1997@gmail.com
* @date     2020.5.24
* @version  1.0
* @par      Edit history:
*           1.0: lddddd, 2020.5.24, .
*/
#include "gcs_display.h"

// void GcsDisplay::LoopTimerCallback(const ros::TimerEvent& _event)
// {
//     UavStateDisplay();
// }

void GcsDisplay::UavStateCallback(const mavros_msgs::State::ConstPtr& _msg)
{
    current_state_uav_ = *_msg;
}

void GcsDisplay::UavPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg)
{
    local_position_uav_.x = _msg->pose.position.x;
    local_position_uav_.y = _msg->pose.position.y;
    local_position_uav_.z = _msg->pose.position.z;
}

void GcsDisplay::UavVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg)
{
    local_velocity_uav_.x = _msg->twist.linear.x;
    local_velocity_uav_.y = _msg->twist.linear.y;
    local_velocity_uav_.z = _msg->twist.linear.z;
}

void GcsDisplay::UavImuCallback(const sensor_msgs::Imu::ConstPtr& _msg)
{
    attitude_rate_uav_.x = _msg->angular_velocity.x;
    attitude_rate_uav_.y = _msg->angular_velocity.y;
    attitude_rate_uav_.z = _msg->angular_velocity.z;
    quaternion_uav_.w = _msg->orientation.w;
    quaternion_uav_.x = _msg->orientation.x;
    quaternion_uav_.y = _msg->orientation.y;
    quaternion_uav_.z = _msg->orientation.z;

    // MathUtils::Quaternion2Euler(quaternion_uav_, attitude_angle_uav_);

    tf::Quaternion quat;
    tf::quaternionMsgToTF(quaternion_uav_, quat);
    tf::Matrix3x3(quat).getRPY(attitude_angle_uav_.x, attitude_angle_uav_.y, attitude_angle_uav_.z);    //四元数转欧拉角
}

void GcsDisplay::UavCommandCallback(const px4_application::UavCommand::ConstPtr& _msg)
{
    command_reception_ = *_msg;
}

void GcsDisplay::EstimatorStatusCallback(const mavros_msgs::EstimatorStatus::ConstPtr& _msg)
{
    estimator_status_uav_ = *_msg;
}

void GcsDisplay::ExtendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& _msg)
{
    extended_state_uav_ = *_msg;
}

float GcsDisplay::GetTimePassSec(void)
{
    ros::Time current_time = ros::Time::now();
    float current_time_sec = current_time.sec - begin_time_.sec;
    float current_time_nsec = current_time.nsec / 1e9 - begin_time_.nsec / 1e9;
    return (current_time_sec + current_time_nsec);
}

void GcsDisplay::CommandUpdateReset(void)
{
    command_reception_.update = false;
}

void GcsDisplay::LoopTask(void)
{
    UavStateDisplay();
    // cout << "Virtual Loop Task of Derived Class !" << endl;
}

void GcsDisplay::Initialize(void)
{
    begin_time_ = ros::Time::now();
    current_state_uav_.mode = "UNKNOWN";
    command_reception_.task_name = "UNKNOWN";
    //loop_timer_ = nh_.createTimer(ros::Duration(loop_period_), &GcsDisplay::LoopTimerCallback, this);
    uav_state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state",
                                                        10,
                                                         &GcsDisplay::UavStateCallback,
                                                          this,
                                                           ros::TransportHints().tcpNoDelay());   //tcpNoDelay默认true降低延迟
    uav_local_position_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",
                                                                         10,
                                                                          &GcsDisplay::UavPositionCallback,
                                                                           this,
                                                                            ros::TransportHints().tcpNoDelay());
    uav_local_velocity_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local",
                                                                          10,
                                                                           &GcsDisplay::UavVelocityCallback,
                                                                            this,
                                                                             ros::TransportHints().tcpNoDelay());
    uav_imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/mavros/imu/data",
                                                    10,
                                                     &GcsDisplay::UavImuCallback,
                                                      this,
                                                       ros::TransportHints().tcpNoDelay());
    uav_estimator_sub_ = nh_.subscribe<mavros_msgs::EstimatorStatus>("/mavros/estimator_status",
                                                                      10,
                                                                       &GcsDisplay::EstimatorStatusCallback,
                                                                        this,
                                                                         ros::TransportHints().tcpNoDelay());
    uav_extended_state_sub_ = nh_.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state",
                                                                         10,
                                                                          &GcsDisplay::ExtendedStateCallback,
                                                                           this,
                                                                            ros::TransportHints().tcpNoDelay());
    uav_command_sub_ = nh_.subscribe<px4_application::UavCommand>("/px4_application/uav_command",
                                                                   10,
                                                                    &GcsDisplay::UavCommandCallback,
                                                                     this,
                                                                      ros::TransportHints().tcpNoDelay());
}
void GcsDisplay::UavStateDisplay(void)
{
    std::cout << "---------------------------------State Info----------------------------------" << std::endl;
    //固定的浮点显示
    std::cout.setf(std::ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    std::cout << std::setprecision(2);
    //左对齐
    std::cout.setf(std::ios::right);
    // 强制显示小数点
    std::cout.setf(std::ios::showpoint);
    // 强制显示符号
    std::cout.setf(std::ios::showpos);

    std::cout << "Time:" << std::setw(8) << GetTimePassSec() << " [s] ";

    //是否和飞控建立起连接
    // if (current_state_uav_.connected == true)
    // {
    //     std::cout << " [ Connected ] ";
    // }
    // else
    // {
    //     std::cout << " [ Unconnected ] ";
    // }

    std::cout << (current_state_uav_.connected ? " [ Connected ] " : " [ Unconnected ] ");
    //是否上锁
    // if (current_state_uav_.armed == true)
    // {
    //     std::cout << " [ Armed ] ";
    // }
    // else
    // {
    //     std::cout << " [ DisArmed ] ";
    // }
    std::cout << (current_state_uav_.armed ? " [ Armed ] " : " [ DisArmed ] ");
    std::cout << " [ " << current_state_uav_.mode <<" ] ";
    std::string flight_state;
    switch(extended_state_uav_.landed_state)
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
    std::cout << "Attitude   " << (estimator_status_uav_.attitude_status_flag ? "[√]   " : "[X]   ")
               << "Vel Horiz Rel " << (estimator_status_uav_.velocity_horiz_status_flag ? "[√]   " : "[X]   ") 
                << "Vel Verti Rel " << (estimator_status_uav_.velocity_vert_status_flag ? "[√]   " : "[X]   ") << std::endl;


    std::cout << "                   ";
    std::cout << "Accel      " << (estimator_status_uav_.accel_error_status_flag ? "[X]   " : "[√]   ")
               << "Pos Verti Rel " << (estimator_status_uav_.pos_vert_agl_status_flag ? "[√]   " : "[X]   ") 
                << "Pos Verti Abs " << (estimator_status_uav_.pos_vert_abs_status_flag ? "[√]   " : "[X]   ") << std::endl;

    std::cout << "                   ";
    std::cout << "Gps Glitch " << (estimator_status_uav_.gps_glitch_status_flag ? "[√]   " : "[X]   ")
               << "Pos Horiz Rel " << (estimator_status_uav_.pos_horiz_rel_status_flag ? "[√]   " : "[X]   ") 
                << "Pos Horiz Abs " << (estimator_status_uav_.pos_horiz_abs_status_flag ? "[√]   " : "[X]   ") << std::endl;

    std::cout << "                   ";
    std::cout << "Const Mode " << (estimator_status_uav_.const_pos_mode_status_flag ? "[√]   " : "[X]   ")
               << "pre Horiz Rel " << (estimator_status_uav_.pred_pos_horiz_rel_status_flag ? "[√]   " : "[X]   ") 
                << "pre Horiz Abs " << (estimator_status_uav_.pred_pos_horiz_abs_status_flag ? "[√]   " : "[X]   ") << std::endl;

    const double RAD2DEG = 57.295779513082320876846364344191;
    std::string frame_name;
    if(command_reception_.frame_id == px4_application::UavCommand::LOCAL)
    {
        frame_name = " [LOCAL]";
    }
    else if(command_reception_.frame_id == px4_application::UavCommand::BODY)
    {
        frame_name = "  [BODY]";
    }
    else
    {
        frame_name = " [ERROR]";
    }
    int setw_num = 9;
    std::cout << "--------------------------------Attitude Info--------------------------------" << std::endl;
    std::cout << "Attitude Angle  [R P Y] : " << std::setw(setw_num) << attitude_angle_uav_.x * RAD2DEG << " [ ° ] " << std::setw(setw_num) << attitude_angle_uav_.y * RAD2DEG << " [ ° ] " << std::setw(setw_num) << attitude_angle_uav_.z * RAD2DEG << " [ ° ] " << std::endl;
    std::cout << "Attitude Rate   [R P Y] : " << std::setw(setw_num) << attitude_rate_uav_.x  * RAD2DEG << " [°/s] " << std::setw(setw_num) << attitude_rate_uav_.y  * RAD2DEG << " [°/s] " << std::setw(setw_num) << attitude_rate_uav_.z  * RAD2DEG << " [°/s] " << std::endl;
    std::cout << "----------------------------Navigation Info [ENU]----------------------------" << std::endl;
    std::cout << "FCU Position    [X Y Z] : " << std::setw(setw_num) << local_position_uav_.x << " [ m ] " << std::setw(setw_num) << local_position_uav_.y << " [ m ] " << std::setw(setw_num) << local_position_uav_.z << " [ m ] " << std::endl;
    std::cout << "FCU Velocity    [X Y Z] : " << std::setw(setw_num) << local_velocity_uav_.x << " [m/s] " << std::setw(setw_num) << local_velocity_uav_.y << " [m/s] " << std::setw(setw_num) << local_velocity_uav_.z << " [m/s] " << std::endl;
    std::cout << "-------------------------------Command" << frame_name << "-------------------------------" << std::endl;
    std::cout << "Period: " << command_reception_.period  << " [s] " << " [ " << command_reception_.task_name << " ] ";
    // if (command_reception_.update == true)
    // {
    //     std::cout << " [ active ] ";
    // }
    // else
    // {
    //     std::cout << "  [ dead ]  ";
    // }
    std::cout << (command_reception_.update ? " [ ACTIVE ] " : "  [ DEAD ]  ");
    switch(command_reception_.xyz_id)
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
        default:     
        {
            std::cout << " [ ERROR ] ";
            break;
        }
    }
    switch(command_reception_.yaw_id)
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
    std::cout << "Command Reception : " << std::setw(setw_num) << command_reception_.x << " [X]" << std::setw(setw_num) << command_reception_.y << " [Y]"
               << std::setw(8) << command_reception_.z << " [Z]" << std::setw(setw_num) << command_reception_.yaw << " [Yaw]" << std::endl;
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
    ros::NodeHandle nh("~");
    GcsDisplay GcsDisplay(nh, 0.1);

    ros::spin();
    return 0;

}

