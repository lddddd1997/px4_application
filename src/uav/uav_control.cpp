/** 
* @file     uav_control.cpp
* @brief    无人机指令控制
* @details  
* @note
* @author   lddddd
*           Email: lddddd1997@gmail.com
*           Github: https://github.com/lddddd1997
* @date     2020.9.07
* @version  2.1
* @par      Edit history:
*           1.0: lddddd, 2020.5.24, .
*           1.1: lddddd, 2020.7.06, 内部指令输入全部修改为速度输入.
*           2.0: lddddd, 2020.7.21, 更新节点句柄与topic的命名空间.
×           2.1: lddddd, 2020.9.07, 修正固件版本导致的Body heading坐标系下控制指令方向的错误(Firmware 1.9.2)Body: Head X+  Left Y+  Up Z+ (Firmware 1.10.1)Body: Head Y+  Right X+  Up Z+)注：以1.9.2为基准
*/

#include "uav_control.h"

void UavControl::CommandCallback(const px4_application::UavCommand::ConstPtr& _msg)
{
    this->command_reception = *_msg;
}

void UavControl::PositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg)
{
    this->local_position_uav.x = _msg->pose.position.x;
    this->local_position_uav.y = _msg->pose.position.y;
    this->local_position_uav.z = _msg->pose.position.z;
}

void UavControl::LoopTask(void)
{
    CommandExecution();
    // std::cout << "Virtual Loop Task of Derived Class !" << std::endl;
}

void UavControl::Initialize(void)
{
    this->setpoint_raw_local_pub = this->nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    this->uav_command_sub = this->nh.subscribe<px4_application::UavCommand>("px4_application/uav_command",
                                                                             1,
                                                                              &UavControl::CommandCallback,
                                                                               this,
                                                                                ros::TransportHints().tcpNoDelay());
    this->uav_local_position_sub = this->nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",
                                                                                   10,
                                                                                    &UavControl::PositionCallback,
                                                                                     this,
                                                                                      ros::TransportHints().tcpNoDelay());
    ros::NodeHandle nh("~");
    PidParameters param;
    nh.param<float>("pid_xy/position/kp", param.kp, 1.0);
    nh.param<float>("pid_xy/position/ki", param.ki, 0.0);
    nh.param<float>("pid_xy/position/kd", param.kd, 0.0);
    nh.param<float>("pid_xy/position/ff", param.ff, 0.0);
    nh.param<float>("pid_xy/position/error_max", param.error_max, 10.0);
    nh.param<float>("pid_xy/position/integral_max", param.integral_max, 5.0);
    nh.param<float>("pid_xy/position/output_max", param.output_max, 8.0);    //QGC XY速度最大期望默认10m/s
    this->PositionX.SetParameters(param);
    this->PositionY.SetParameters(param);
    nh.param<float>("pid_z/position/kp", param.kp, 1.0);
    nh.param<float>("pid_z/position/ki", param.ki, 0.0);
    nh.param<float>("pid_z/position/kd", param.kd, 0.0);
    nh.param<float>("pid_z/position/ff", param.ff, 0.0);
    nh.param<float>("pid_z/position/error_max", param.error_max, 3.0);
    nh.param<float>("pid_z/position/integral_max", param.integral_max, 2.0);
    nh.param<float>("pid_z/position/output_max", param.output_max, 2.0);    //QGC Z速度最大上升默认3m/s 下降1m/s
    this->PositionZ.SetParameters(param);
    std::cout << "----------X position controller parameters----------" << std::endl;
    this->PositionX.PrintParameters();
    std::cout << "----------Y position controller parameters----------" << std::endl;
    this->PositionY.PrintParameters();
    std::cout << "----------Z position controller parameters----------" << std::endl;
    this->PositionZ.PrintParameters();
}

void UavControl::CommandUpdateReset(void)
{
    this->command_reception.update = false;
}

void UavControl::CommandExecution(void)
{
    // if(!this->command_reception.update)    //如果指令没更新，则退出
    //     return;
    /*坐标系选择，下面的指令只能用于LOCAL坐标系（因为某些指令控制用到参考坐标系反馈），若用BODY坐标系，需用速度控制指令或加速度控制指令*/
    switch(this->command_reception.frame_id)
    {
        case px4_application::UavCommand::LOCAL:
        {
            this->command_target_uav.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            break;
        }
        case px4_application::UavCommand::BODY:
        {
            this->command_target_uav.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
            break;
        }
        default:
        {
            this->command_target_uav.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            break;
        }
    }
    /*航向控制选择*/
    switch(this->command_reception.yaw_id)
    {
        case px4_application::UavCommand::NO_YAW:    //无航向指令
        {
            this->command_target_uav.type_mask = 0b110000000000;

            break;
        }
        case px4_application::UavCommand::YAW:    //航向角指令
        {
            this->command_target_uav.type_mask = 0b100000000000;
            this->command_target_uav.yaw = this->command_reception.yaw;
            break;
        }
        case px4_application::UavCommand::YAW_RATE:    //航向角速度指令
        {
            this->command_target_uav.type_mask = 0b010000000000;
            this->command_target_uav.yaw_rate = this->command_reception.yaw;
            break;
        }
        default:
        {
            this->command_target_uav.type_mask = 0b110000000000;
            break;
        }
    }
    /*位置速度控制选择*/
    switch(this->command_reception.xyz_id)    //注：建议在位置速度组合控制时，使用VX_VY_VZ模式，位置外环控制输出到速度内环
    {
        case px4_application::UavCommand::PX_PY_PZ:
        {
            this->command_target_uav.type_mask |= 0b000111000111;
            this->command_target_uav.velocity.x = this->PositionX.ControlOutput(this->command_reception.x, this->local_position_uav.x);
            this->command_target_uav.velocity.y = this->PositionY.ControlOutput(this->command_reception.y, this->local_position_uav.y);
            this->command_target_uav.velocity.z = this->PositionZ.ControlOutput(this->command_reception.z, this->local_position_uav.z);
            break;
        }
        case px4_application::UavCommand::VX_VY_VZ:
        {
            this->command_target_uav.type_mask |= 0b000111000111;
            /*修正固件版本导致的Body heading坐标系下控制指令方向的错误(Firmware 1.9.2)Body: Head X+  Left Y+  Up Z+ (Firmware 1.10.1)Body: Head Y+  Right X+  Up Z+)注：以1.9.2为基准*/
            if(this->command_reception.frame_id == px4_application::UavCommand::BODY)
            {
                this->command_target_uav.velocity.x = -this->command_reception.y;
                this->command_target_uav.velocity.y = this->command_reception.x;
                this->command_target_uav.velocity.z = this->command_reception.z;
            }
            else
            {
                this->command_target_uav.velocity.x = this->command_reception.x;
                this->command_target_uav.velocity.y = this->command_reception.y;
                this->command_target_uav.velocity.z = this->command_reception.z;
            }
            break;
        }
        case px4_application::UavCommand::VX_VY_PZ:
        {
            this->command_target_uav.type_mask |= 0b000111000111;
            this->command_target_uav.velocity.x = this->command_reception.x;
            this->command_target_uav.velocity.y = this->command_reception.y;
            this->command_target_uav.velocity.z = this->PositionZ.ControlOutput(this->command_reception.z, this->local_position_uav.z);
            break;
        }
        case px4_application::UavCommand::VX_PY_VZ:
        {
            this->command_target_uav.type_mask |= 0b000111000111;
            this->command_target_uav.velocity.x = this->command_reception.x;
            this->command_target_uav.velocity.y = this->PositionY.ControlOutput(this->command_reception.y, this->local_position_uav.y);
            this->command_target_uav.velocity.z = this->command_reception.z;
            break;
        }
        case px4_application::UavCommand::PX_VY_VZ:
        {
            this->command_target_uav.type_mask |= 0b000111000111;
            this->command_target_uav.velocity.x = this->PositionX.ControlOutput(this->command_reception.x, this->local_position_uav.x);
            this->command_target_uav.velocity.y = this->command_reception.y;
            this->command_target_uav.velocity.z = this->command_reception.z;
            break;
        }
        case px4_application::UavCommand::PX_PY_VZ:
        {
            this->command_target_uav.type_mask |= 0b000111000111;
            this->command_target_uav.velocity.x = this->PositionX.ControlOutput(this->command_reception.x, this->local_position_uav.x);
            this->command_target_uav.velocity.y = this->PositionY.ControlOutput(this->command_reception.y, this->local_position_uav.y);
            this->command_target_uav.velocity.z = this->command_reception.z;
            break;
        }
        case px4_application::UavCommand::UX_UY_UZ:
        {
            this->command_target_uav.type_mask &= 0x4000;    //保持
            break;
        }
        default: 
        {
            this->command_target_uav.type_mask &= 0x4000;
            break;
        }
    }

    this->command_target_uav.header.stamp = ros::Time::now();
    this->setpoint_raw_local_pub.publish(this->command_target_uav);
    CommandUpdateReset();    //update重置
}

UavControl::UavControl(const ros::NodeHandle& _nh, double _period) : RosBase(_nh, _period)
                                                                    , PositionX(PidController::NORMAL)
                                                                     , PositionY(PidController::NORMAL)
                                                                      , PositionZ(PidController::NORMAL)
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

