/** 
* @file     uav_control.cpp
* @brief    无人机指令控制
* @details  
* @note
* @author   lddddd
*           Email: lddddd1997@gmail.com
*           Github: https://github.com/lddddd1997
* @date     2020.7.21
* @version  2.0
* @par      Edit history:
*           1.0: lddddd, 2020.5.24, .
*           1.1: lddddd, 2020.7.06, 内部指令输入全部修改为速度输入.
*           2.0: lddddd, 2020.7.21, 更新节点句柄与topic的命名空间.
*/

#include "uav_control.h"

// void UavControl::LoopTimerCallback(const ros::TimerEvent& _event)
// {
//     CommandExecution();
// }

void UavControl::UavCommandCallback(const px4_application::UavCommand::ConstPtr& _msg)
{
    command_reception_ = *_msg;
}

void UavControl::UavPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg)
{
    local_position_uav_.x = _msg->pose.position.x;
    local_position_uav_.y = _msg->pose.position.y;
    local_position_uav_.z = _msg->pose.position.z;
}

void UavControl::LoopTask(void)
{
    CommandExecution();
    // std::cout << "Virtual Loop Task of Derived Class !" << std::endl;
}

void UavControl::Initialize(void)
{
    // loop_timer_ = nh_.createTimer(ros::Duration(loop_period_), &UavControl::LoopTimerCallback, this); //周期为0.01s
    setpoint_raw_local_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    uav_command_sub_ = nh_.subscribe<px4_application::UavCommand>("px4_application/uav_command",
                                                                   1,
                                                                    &UavControl::UavCommandCallback,
                                                                     this,
                                                                      ros::TransportHints().tcpNoDelay());
    uav_local_position_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",
                                                                         10,
                                                                          &UavControl::UavPositionCallback,
                                                                           this,
                                                                            ros::TransportHints().tcpNoDelay());
    ros::NodeHandle nh("~");
    PidParameters param;
    nh.param<float>("pid_xy/pos/kp", param.kp, 1.0);
    nh.param<float>("pid_xy/pos/ki", param.ki, 0.0);
    nh.param<float>("pid_xy/pos/kd", param.kd, 0.0);
    nh.param<float>("pid_xy/pos/ff", param.ff, 0.0);
    nh.param<float>("pid_xy/pos/error_max", param.error_max, 10.0);
    nh.param<float>("pid_xy/pos/integral_max", param.integral_max, 5.0);
    nh.param<float>("pid_xy/pos/output_max", param.output_max, 8.0);    //QGC XY速度最大期望默认10m/s
    PoseX.SetParameters(param);
    PoseY.SetParameters(param);
    nh.param<float>("pid_z/pos/kp", param.kp, 1.0);
    nh.param<float>("pid_z/pos/ki", param.ki, 0.0);
    nh.param<float>("pid_z/pos/kd", param.kd, 0.0);
    nh.param<float>("pid_z/pos/ff", param.ff, 0.0);
    nh.param<float>("pid_z/pos/error_max", param.error_max, 3.0);
    nh.param<float>("pid_z/pos/integral_max", param.integral_max, 2.0);
    nh.param<float>("pid_z/pos/output_max", param.output_max, 2.0);    //QGC Z速度最大上升默认3m/s 下降1m/s
    PoseZ.SetParameters(param);
    std::cout << "----------X Position controller parameters----------" << std::endl;
    PoseX.PrintParameters();
    std::cout << "----------Y Position controller parameters----------" << std::endl;
    PoseY.PrintParameters();
    std::cout << "----------Z Position controller parameters----------" << std::endl;
    PoseZ.PrintParameters();
}

void UavControl::CommandUpdateReset(void)
{
    command_reception_.update = false;
}

void UavControl::CommandExecution(void)
{
    // if(!command_reception_.update)    //如果指令没更新，则退出
    //     return;
    //坐标系选择
    switch(command_reception_.frame_id)
    {
        case px4_application::UavCommand::LOCAL:
        {
            command_target_uav_.coordinate_frame = 1;
            break;
        }
        case px4_application::UavCommand::BODY:
        {
            command_target_uav_.coordinate_frame = 8;
            break;
        }
        default:
        {
            command_target_uav_.coordinate_frame = 1;
            break;
        }
    }
    //航向控制选择
    switch(command_reception_.yaw_id)
    {
        case px4_application::UavCommand::NO_YAW:    //无航向指令
        {
            command_target_uav_.type_mask = 0b110000000000;

            break;
        }
        case px4_application::UavCommand::YAW:    //航向角指令
        {
            command_target_uav_.type_mask = 0b100000000000;
            command_target_uav_.yaw = command_reception_.yaw;
            break;
        }
        case px4_application::UavCommand::YAW_RATE:    //航向角速度指令
        {
            command_target_uav_.type_mask = 0b010000000000;
            command_target_uav_.yaw_rate = command_reception_.yaw;
            break;
        }
        default:
        {
            command_target_uav_.type_mask = 0b110000000000;
            break;
        }
    }
    //位置速度控制选择
    switch(command_reception_.xyz_id)    //注：建议在位置速度组合控制时，使用VX_VY_VZ模式，位置外环控制输出到速度内环
    {
        case px4_application::UavCommand::PX_PY_PZ:
        {
            command_target_uav_.type_mask |= 0b000111000111;
            command_target_uav_.velocity.x = PoseX.ControlOutput(command_reception_.x, local_position_uav_.x);
            command_target_uav_.velocity.y = PoseY.ControlOutput(command_reception_.y, local_position_uav_.y);
            command_target_uav_.velocity.z = PoseZ.ControlOutput(command_reception_.z, local_position_uav_.z);
            break;
        }
        case px4_application::UavCommand::VX_VY_VZ:
        {
            command_target_uav_.type_mask |= 0b000111000111;    // 000 111 000 111
            command_target_uav_.velocity.x = command_reception_.x;
            command_target_uav_.velocity.y = command_reception_.y;
            command_target_uav_.velocity.z = command_reception_.z;
            break;
        }
        case px4_application::UavCommand::VX_VY_PZ:
        {
            command_target_uav_.type_mask |= 0b000111000111;
            command_target_uav_.velocity.x = command_reception_.x;
            command_target_uav_.velocity.y = command_reception_.y;
            command_target_uav_.velocity.z = PoseZ.ControlOutput(command_reception_.z, local_position_uav_.z);
            break;
        }
        case px4_application::UavCommand::VX_PY_VZ:
        {
            command_target_uav_.type_mask |= 0b000111000111;
            command_target_uav_.velocity.x = command_reception_.x;
            command_target_uav_.velocity.y = PoseY.ControlOutput(command_reception_.y, local_position_uav_.y);
            command_target_uav_.velocity.z = command_reception_.z;
            break;
        }
        case px4_application::UavCommand::PX_VY_VZ:
        {
            command_target_uav_.type_mask |= 0b000111000111;
            command_target_uav_.velocity.x = PoseX.ControlOutput(command_reception_.x, local_position_uav_.x);
            command_target_uav_.velocity.y = command_reception_.y;
            command_target_uav_.velocity.z = command_reception_.z;
            break;
        }
        case px4_application::UavCommand::PX_PY_VZ:
        {
            command_target_uav_.type_mask |= 0b000111000111;
            command_target_uav_.velocity.x = PoseX.ControlOutput(command_reception_.x, local_position_uav_.x);
            command_target_uav_.velocity.y = PoseY.ControlOutput(command_reception_.y, local_position_uav_.y);
            command_target_uav_.velocity.z = command_reception_.z;
            break;
        }
        case px4_application::UavCommand::UX_UY_UZ:
        {
            command_target_uav_.type_mask &= 0x4000;    //保持
            break;
        }
        default: 
        {
            command_target_uav_.type_mask &= 0x4000;
            break;
        }
    }

    command_target_uav_.header.stamp = ros::Time::now();
    setpoint_raw_local_pub_.publish(command_target_uav_);
    CommandUpdateReset();    //update重置
}

UavControl::UavControl(const ros::NodeHandle& _nh, double _period) : RosBase(_nh, _period)
                                                                    , PoseX(PidController::NORMAL)
                                                                     , PoseY(PidController::NORMAL)
                                                                      , PoseZ(PidController::NORMAL)
{
    Initialize();
}

UavControl::~UavControl()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_control");
    ros::NodeHandle nh;
    UavControl UavControl(nh, 0.01);
    
    ros::spin();
    return 0;
}

