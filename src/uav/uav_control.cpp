/** 
* @file         uav_control.cpp
* @brief        无人机指令控制
* @author       lddddd
* @date         2020.5.24 
* @version      v1.0 
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

void UavControl::LoopTask(void)
{
    CommandExecution();
    // std::cout << "Virtual Loop Task of Derived Class !" << std::endl;
}

void UavControl::Initialize(void)
{
    // loop_timer_ = nh_.createTimer(ros::Duration(loop_period_), &UavControl::LoopTimerCallback, this); //周期为0.01s
    setpoint_raw_local_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    uav_command_sub_ = nh_.subscribe<px4_application::UavCommand>("/px4_application/uav_command",
                                                                   1,
                                                                    &UavControl::UavCommandCallback,
                                                                     this,
                                                                      ros::TransportHints().tcpNoDelay());
}

void UavControl::CommandUpdateReset(void)
{
    command_reception_.update = false;
}

void UavControl::CommandExecution(void)
{
    if(!command_reception_.update)    //如果指令没更新，则退出
        return;
    switch(command_reception_.xyz_id)
    {
        case px4_application::UavCommand::PX_PY_PZ:
        {
            command_target_uav_.type_mask = 0b000111111000;    // 000 111 111 000
            command_target_uav_.position.x = command_reception_.x;
            command_target_uav_.position.y = command_reception_.y;
            command_target_uav_.position.z = command_reception_.z;
            break;
        }
        case px4_application::UavCommand::VX_VY_VZ:
        {
            command_target_uav_.type_mask = 0b000111000111;    // 000 111 000 111
            command_target_uav_.velocity.x = command_reception_.x;
            command_target_uav_.velocity.y = command_reception_.y;
            command_target_uav_.velocity.z = command_reception_.z;
            break;
        }
        case px4_application::UavCommand::VX_VY_PZ:
        {
            command_target_uav_.type_mask = 0b000000000011;    // 000 000 000 011
            command_target_uav_.velocity.x = command_reception_.x;
            command_target_uav_.velocity.y = command_reception_.y;
            command_target_uav_.position.z = command_reception_.z;
            break;
        }
        case px4_application::UavCommand::VX_PY_VZ:   //控制不了Y位置
        {
            command_target_uav_.type_mask = 0b000000000101;    // 000 000 000 101
            command_target_uav_.velocity.x = command_reception_.x;
            command_target_uav_.position.y = command_reception_.y;
            command_target_uav_.velocity.z = command_reception_.z;
            break;
        }
        case px4_application::UavCommand::PX_VY_VZ:    //控制不了X位置
        {
            command_target_uav_.type_mask = 0b000000000110;    // 000 000 000 110
            command_target_uav_.position.x = command_reception_.x;
            command_target_uav_.velocity.y = command_reception_.y;
            command_target_uav_.velocity.z = command_reception_.z;
            break;
        }
        default: 
        {
            command_target_uav_.type_mask = 0b111111111111;
            break;
        }
    }

    switch(command_reception_.yaw_id)
    {
        case px4_application::UavCommand::NO_YAW:    //无航向指令
        {
            command_target_uav_.type_mask |= 0b110000000000;

            break;
        }
        case px4_application::UavCommand::YAW:    //航向角指令
        {
            command_target_uav_.type_mask |= 0b100000000000;
            command_target_uav_.yaw = command_reception_.yaw;
            break;
        }
        case px4_application::UavCommand::YAW_RATE:    //航向角速度指令
        {
            command_target_uav_.type_mask |= 0b010000000000;
            command_target_uav_.yaw_rate = command_reception_.yaw;
            break;
        }
        default:
        {
            command_target_uav_.type_mask |= 0b110000000000;
            break;
        }
    }

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
    command_target_uav_.header.stamp = ros::Time::now();
    setpoint_raw_local_pub_.publish(command_target_uav_);
    CommandUpdateReset();    //update重置
}

UavControl::UavControl(const ros::NodeHandle& _nh, const double _period) : RosBase(_nh, _period)
{
    Initialize();
}

UavControl::~UavControl()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_control");
    ros::NodeHandle nh("~");
    UavControl UavControl(nh, 0.01);
    
    ros::spin();
    return 0;
}

