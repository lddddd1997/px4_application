/** 
* @file     uav_control.cpp
* @brief    无人机指令控制
* @details  
* @note
* @author   lddddd
*           Email: lddddd1997@gmail.com
*           Github: https://github.com/lddddd1997
* @date     2021.1.06
* @version  2.2
* @par      Edit history:
*           1.0: lddddd, 2020.5.24, .
*           1.1: lddddd, 2020.7.06, 内部指令输入全部修改为速度输入.
*           2.0: lddddd, 2020.7.21, 更新节点句柄与topic的命名空间.
*           2.1: lddddd, 2020.9.07, 修正固件版本导致的Body heading坐标系下控制指令方向的错误(Firmware 1.9.2)Body: Head X+  Left Y+  Up Z+ (Firmware 1.10.1)Body: Head Y+  Right X+  Up Z+)注：以1.9.2为基准
*           2.2: lddddd, 2021.1.06, 添加终端界面的无人机id显示.
*/

#include "uav/uav_control.h"

void UavControl::StateCallback(const mavros_msgs::State::ConstPtr& _msg)
{
    this->curr_mode = _msg->mode;
}

void UavControl::CommandCallback(const px4_application::UavCommand::ConstPtr& _msg)
{
    this->command_reception = *_msg;
}

void UavControl::PositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg)
{
    this->local_position[0] = _msg->pose.position.x;
    this->local_position[1] = _msg->pose.position.y;
    this->local_position[2] = _msg->pose.position.z;
}

void UavControl::VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg)
{
    this->local_velocity[0] = _msg->twist.linear.x;
    this->local_velocity[1] = _msg->twist.linear.y;
    this->local_velocity[2] = _msg->twist.linear.z;
}

void UavControl::FlatTargetCallback(const px4_application::FlatTarget::ConstPtr& _msg)
{
    this->flat_target = *_msg;
}

void UavControl::LoopTask(void)
{
    // CommandExecution();
    // std::cout << "Virtual Loop Task of Derived Class !" << std::endl;
}

void UavControl::LoopTaskWithoutVirtual(void)
{
    CommandExecution();
    // std::cout << "Virtual Loop Task of Derived Class !" << std::endl;
}

void UavControl::Initialize(void)
{
    this->state_sub = this->nh.subscribe<mavros_msgs::State>("mavros/state",
                                                              10,
                                                               &UavControl::StateCallback,
                                                                this,
                                                                 ros::TransportHints().tcpNoDelay());
    this->setpoint_raw_local_pub = this->nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    this->uav_command_sub = this->nh.subscribe<px4_application::UavCommand>("px4_application/uav_command",
                                                                             1,
                                                                              &UavControl::CommandCallback,
                                                                               this,
                                                                                ros::TransportHints().tcpNoDelay());
    this->uav_local_position_sub = this->nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",
                                                                                   1,
                                                                                    &UavControl::PositionCallback,
                                                                                     this,
                                                                                      ros::TransportHints().tcpNoDelay());
    this->uav_local_velocity_sub = this->nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local",
                                                                                    1,
                                                                                     &UavControl::VelocityCallback,
                                                                                      this,
                                                                                       ros::TransportHints().tcpNoDelay());
    this->flat_target_sub = this->nh.subscribe<px4_application::FlatTarget>("reference/flat_setpoint",
                                                                             1,
                                                                              &UavControl::FlatTargetCallback,
                                                                               this,
                                                                                ros::TransportHints().tcpNoDelay());
    ros::NodeHandle nh("~");
    nh.param<int>("uav_id", this->own_id, 0);
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
    // std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>uav_" << this->own_id << " parameters<<<<<<<<<<<<<<<<<<<<<<<<<<<<< " << std::endl;
    // std::cout << "----------X position controller parameters----------" << std::endl;
    // this->PositionX.PrintParameters();
    // std::cout << "----------Y position controller parameters----------" << std::endl;
    // this->PositionY.PrintParameters();
    // std::cout << "----------Z position controller parameters----------" << std::endl;
    // this->PositionZ.PrintParameters();


    this->use_position_control = true;
    nh.param<bool>("use_position_control", this->use_position_control, true);
    std::cout << "use position control: " << this->use_position_control << std::endl;
    double kp_xy = 0.0;
    double kp_z = 0.0;
    double kv_xy = 0.0;
    double kv_z = 0.0;
    double ki_xyz = 0.0;
    double int_max = 0.0;
    nh.param<double>("traj/kp_xy", kp_xy, 3.0);
    nh.param<double>("traj/kp_z", kp_z, 2.5);
    nh.param<double>("traj/kv_xy", kv_xy, 2.0);
    nh.param<double>("traj/kv_z", kv_z, 3.5);
    nh.param<double>("traj/ki", ki_xyz, 0.01);
    nh.param<double>("traj/integral_max", int_max, 1.0);
    nh.param<double>("traj/feedback_acc_max", max_fb_acc, 4.0);
    
    this->kp << kp_xy, kp_xy, kp_z;
    this->kv << kv_xy, kv_xy, kv_z;
    this->ki << ki_xyz, ki_xyz, ki_xyz;
    this->integral_max << int_max, int_max, int_max;
    std::cout << "-------------------kp-------------------" << std::endl;
    std::cout << this->kp.col(0) << std::endl;
    std::cout << "-------------------kv-------------------" << std::endl;
    std::cout << this->kv.col(0) << std::endl;
    std::cout << "-------------------ki-------------------" << std::endl;
    std::cout << this->ki.col(0) << std::endl;
    std::cout << "--------------integral_max--------------" << std::endl;
    std::cout << this->integral_max.col(0) << std::endl;
    std::cout << "------------feedback_acc_max------------" << std::endl;
    std::cout << this->max_fb_acc << std::endl;
    this->gravity << 0.0, 0.0, -9.8;
    this->integral.setZero();
    this->local_position.setZero();
    this->local_velocity.setZero();
}

void UavControl::CommandUpdateReset(void)
{
    this->command_reception.update = false;
}

void UavControl::CommandExecution(void)
{
    if(this->command_reception.command_type == px4_application::UavCommand::NORMAL)
    {
        NormalCommandFixed();
    }
    else if(this->command_reception.command_type == px4_application::UavCommand::TRAJECTORY)
    {
        TrajerctoryCommandFixed();
    }
    else
    {
        ROS_ERROR("Error command type!!!");
    }
}

Eigen::Vector3d UavControl::ComputeDesiredAcc(const Eigen::Vector3d& target_pos, const Eigen::Vector3d& target_vel, const Eigen::Vector3d& target_acc, double dt)
{
    Eigen::Vector3d pos_error = target_pos - this->local_position;
    Eigen::Vector3d vel_error = target_vel - this->local_velocity;
    Eigen::Vector3d acc_feedforward = this->kp.asDiagonal() * pos_error
                                     + this->kv.asDiagonal() * vel_error
                                     + this->ki.asDiagonal() * this->integral;  // feedforward term for trajectory error


    this->integral += pos_error * dt;
    if(this->curr_mode != "OFFBOARD")
    {
        this->integral.setZero();
    }
    for(int i = 0; i < 3; i++)
    {
        this->integral[i] = MathUtils::Constrain(this->integral[i], this->integral_max[i]);
    }

    if (acc_feedforward.norm() > this->max_fb_acc)
        acc_feedforward = (this->max_fb_acc / acc_feedforward.norm()) * acc_feedforward;  // Clip acceleration if reference is too large

    Eigen::Vector3d acc_deisred = target_acc + acc_feedforward;

    return acc_deisred;
}

void UavControl::TrajerctoryCommandFixed(void)
{
    Eigen::Vector3d target_pos = TypeTransform::RosMsg2Eigen(this->flat_target.position);
    Eigen::Vector3d target_vel = TypeTransform::RosMsg2Eigen(this->flat_target.velocity);
    Eigen::Vector3d target_acc = TypeTransform::RosMsg2Eigen(this->flat_target.acceleration);
    Eigen::Vector3d acc_desired = ComputeDesiredAcc(target_pos, target_vel, target_acc,  (ros::Time::now() - this->command_target_uav.header.stamp).toSec());
    
    if(this->use_position_control) // 使用px4的位置控制
    {
        this->command_target_uav.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        this->command_target_uav.type_mask = 0b100111111000;
        if(this->command_reception.update)
        {
            this->command_target_uav.yaw = this->command_reception.yaw;
        }
        else
        {
            this->command_target_uav.yaw = this->flat_target.yaw;
        }
        this->command_target_uav.position.x = target_pos[0];
        this->command_target_uav.position.y = target_pos[1];
        this->command_target_uav.position.z = target_pos[2];
        this->command_target_uav.header.stamp = ros::Time::now();
        this->setpoint_raw_local_pub.publish(this->command_target_uav);
    }
    else // 使用轨迹控制，加速度输入，TODO：转化成姿态矩阵+推力的指令形式
    {
        this->command_target_uav.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        this->command_target_uav.type_mask = 0b100000111111;
        if(this->command_reception.update)
        {
            this->command_target_uav.yaw = this->command_reception.yaw;
        }
        else
        {
            this->command_target_uav.yaw = this->flat_target.yaw;
        }
        this->command_target_uav.acceleration_or_force.x = acc_desired[0];
        this->command_target_uav.acceleration_or_force.y = acc_desired[1];
        this->command_target_uav.acceleration_or_force.z = acc_desired[2];
        this->command_target_uav.header.stamp = ros::Time::now();
        this->setpoint_raw_local_pub.publish(this->command_target_uav);
    }



}

void UavControl::NormalCommandFixed(void)
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
            // this->command_target_uav.type_mask |= 0b000111000111;
            // this->command_target_uav.velocity.x = this->PositionX.ControlOutput(this->command_reception.x, this->local_position[0]);
            // this->command_target_uav.velocity.y = this->PositionY.ControlOutput(this->command_reception.y, this->local_position[1]);
            // this->command_target_uav.velocity.z = this->PositionZ.ControlOutput(this->command_reception.z, this->local_position[2]);
            this->command_target_uav.type_mask |= 0b000111111000;
            this->command_target_uav.position.x = this->command_reception.x;
            this->command_target_uav.position.y = this->command_reception.y;
            this->command_target_uav.position.z = this->command_reception.z;
            break;
        }
        case px4_application::UavCommand::VX_VY_VZ:
        {
            this->command_target_uav.type_mask |= 0b000111000111;
            if(this->command_reception.frame_id == px4_application::UavCommand::BODY)
            {
                /*修正固件版本导致的Body heading坐标系下控制指令方向的错误(Firmware 1.9.2)Body: Head X+  Left Y+  Up Z+ (Firmware 1.10.1)Body: Head Y+  Right X+  Up Z+)注：以1.9.2为基准*/
                // this->command_target_uav.velocity.x = -this->command_reception.y;
                // this->command_target_uav.velocity.y = this->command_reception.x;
                // this->command_target_uav.velocity.z = this->command_reception.z;
                
                // Head X+  Left Y+  Up Z+     1.11.1
                this->command_target_uav.velocity.x = this->command_reception.x;
                this->command_target_uav.velocity.y = this->command_reception.y;
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
            this->command_target_uav.velocity.z = this->PositionZ.ControlOutput(this->command_reception.z, this->local_position[2]);
            break;
        }
        case px4_application::UavCommand::VX_PY_VZ:
        {
            this->command_target_uav.type_mask |= 0b000111000111;
            this->command_target_uav.velocity.x = this->command_reception.x;
            this->command_target_uav.velocity.y = this->PositionY.ControlOutput(this->command_reception.y, this->local_position[1]);
            this->command_target_uav.velocity.z = this->command_reception.z;
            break;
        }
        case px4_application::UavCommand::PX_VY_VZ:
        {
            this->command_target_uav.type_mask |= 0b000111000111;
            this->command_target_uav.velocity.x = this->PositionX.ControlOutput(this->command_reception.x, this->local_position[0]);
            this->command_target_uav.velocity.y = this->command_reception.y;
            this->command_target_uav.velocity.z = this->command_reception.z;
            break;
        }
        case px4_application::UavCommand::PX_PY_VZ:
        {
            this->command_target_uav.type_mask |= 0b000111000111;
            this->command_target_uav.velocity.x = this->PositionX.ControlOutput(this->command_reception.x, this->local_position[0]);
            this->command_target_uav.velocity.y = this->PositionY.ControlOutput(this->command_reception.y, this->local_position[1]);
            this->command_target_uav.velocity.z = this->command_reception.z;
            break;
        }
        case px4_application::UavCommand::UX_UY_UZ:
        {
            this->command_target_uav.type_mask &= 0x4000;    //保持
            return ;
        }
        default: 
        {
            this->command_target_uav.type_mask &= 0x4000;
            return ;
        }
    }

    this->command_target_uav.header.stamp = ros::Time::now();
    this->setpoint_raw_local_pub.publish(this->command_target_uav);
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
    UavControl UavControl(nh, 1.0);
    // ros::spin();

    double ros_rate = 1.0;
    if(nh.getNamespace() == "/uav1")
    {
        ros::NodeHandle nh_temp("~");
        nh_temp.param<double>("leader_spin_rate/uav_control", ros_rate, 1.0);
    }
    else
    {
        ros::NodeHandle nh_temp("~");
        nh_temp.param<double>("folower_spin_rate/uav_control", ros_rate, 1.0);
    }

    std::cout << "-----------------------" << std::endl;
    std::cout << "ros rate: " << ros_rate << std::endl;
    ros::Rate loop(ros_rate);
    while(ros::ok())
    {
        ros::spinOnce();
        UavControl.LoopTaskWithoutVirtual();
        loop.sleep();
    }
    return 0;
}

