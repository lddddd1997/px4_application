/** 
* @file     uav_collaboration.cpp
* @brief    无人机协同程序
* @details  
* @note
* @author   lddddd
*           Email: lddddd1997@gmail.com
* @date     2020.6.28
* @version  1.0
* @par      Edit history:
*           1.0: lddddd, 2020.6.28, .
*/

#include "uav_collaboration.h"

void UavCollaboration::UavPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg)
{
    uav_info_.position.x = _msg->pose.position.x;
    uav_info_.position.y = _msg->pose.position.y;
    uav_info_.position.z = _msg->pose.position.z;
}

void UavCollaboration::UavVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg)
{
    uav_info_.velocity.x = _msg->twist.linear.x;
    uav_info_.velocity.y = _msg->twist.linear.y;
    uav_info_.velocity.z = _msg->twist.linear.z;
}

void UavCollaboration::ExtendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& _msg)
{
    uav_info_.extended_state = *_msg;
}

void UavCollaboration::EstimatorStatusCallback(const mavros_msgs::EstimatorStatus::ConstPtr& _msg)
{
    uav_info_.estimator_status = *_msg;
}

void UavCollaboration::LoopTask(void)
{
    UavState_->StateMachineSchedule(uav_info_,
                                     uav_command_pub_,
                                      &command_deliver_,
                                       &UavState_);    //运行状态机调度
}

void UavCollaboration::Initialize(void)
{
    // loop_timer_ = nh_.createTimer(ros::Duration(loop_period_), &ArucoLanding::LoopTimerCallback, this); 
    uav_command_pub_ = nh_.advertise<px4_application::UavCommand>("/px4_application/uav_command", 10);

    uav_local_position_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",
                                                                         10,
                                                                          &UavCollaboration::UavPositionCallback,
                                                                           this,
                                                                            ros::TransportHints().tcpNoDelay());
    uav_local_velocity_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local",
                                                                          10,
                                                                           &UavCollaboration::UavVelocityCallback,
                                                                            this,
                                                                             ros::TransportHints().tcpNoDelay());
    uav_estimator_sub_ = nh_.subscribe<mavros_msgs::EstimatorStatus>("/mavros/estimator_status",
                                                                      10,
                                                                       &UavCollaboration::EstimatorStatusCallback,
                                                                        this,
                                                                         ros::TransportHints().tcpNoDelay());
    uav_extended_state_sub_ = nh_.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state",
                                                                         10,
                                                                          &UavCollaboration::ExtendedStateCallback,
                                                                           this,
                                                                            ros::TransportHints().tcpNoDelay());
    UavState_ = new Prepare;    //初始为准备状态
}

UavCollaboration::UavCollaboration(const ros::NodeHandle& _nh, double _period) : RosBase(_nh, _period)
{
    Initialize();
}

UavCollaboration::~UavCollaboration()
{
    if(UavState_ != NULL)
    {
        delete UavState_;
        UavState_ = NULL;
    }
}

/**
* @name         void States::StateMachineSchedule(const UavInfo& _uav_info,
                                                   const ros::Publisher& _uav_command_pub,
                                                    px4_application::UavCommand* _command_deliver,
                                                     States** _State);
* @brief        简易状态机调度（基类）
* @param[in]    无人机状态：_uav_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   void
*/
void States::StateMachineSchedule(const UavInfo& _uav_info,
                                   const ros::Publisher& _uav_command_pub,
                                    px4_application::UavCommand* _command_deliver,
                                     States** _State)
{
    Run(_uav_info, _uav_command_pub, _command_deliver, _State);
}

States::States()
{
    
}

States::~States()
{
    
}

/**
* @name         void Prepare::Run(const UavInfo& _uav_info,
                                   const ros::Publisher& _uav_command_pub,
                                    px4_application::UavCommand* _command_deliver,
                                     States** _State)
* @brief        准备任务接口（派生类）
* @param[in]    无人机状态：_uav_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   void  
*/
void Prepare::Run(const UavInfo& _uav_info,
                   const ros::Publisher& _uav_command_pub,
                    px4_application::UavCommand* _command_deliver,
                     States** _State)
{

    if(!(_uav_info.estimator_status.attitude_status_flag
         && _uav_info.estimator_status.velocity_horiz_status_flag
          && _uav_info.estimator_status.velocity_vert_status_flag))
    {
        ROS_INFO_STREAM_THROTTLE( 1, "Waiting for state estimation to complete..." );
        _command_deliver->header.stamp = ros::Time::now();
        _command_deliver->period = 0.05;
        _command_deliver->update = true;
        _command_deliver->xyz_id = px4_application::UavCommand::UX_UY_UZ;
        _command_deliver->task_name = "Prepare";
        _uav_command_pub.publish(*_command_deliver);
        return ;
    }
    delete *_State;
    *_State = new TakeOff;    //状态转移

}

Prepare::Prepare()
{
    std::cout << "Prepare!" << std::endl;
}

Prepare::~Prepare()
{
    std::cout << "Prepare to Take off..." << std::endl;
}

/**
* @name         void TakeOff::Run(const UavInfo& _uav_info,
                                   const ros::Publisher& _uav_command_pub,
                                    px4_application::UavCommand* _command_deliver,
                                     States** _State)
* @brief        起飞任务接口（派生类）
* @param[in]    无人机状态：_uav_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   void
*/
void TakeOff::Run(const UavInfo& _uav_info,
                   const ros::Publisher& _uav_command_pub,
                    px4_application::UavCommand* _command_deliver,
                     States** _State)
{
    // if(!(_uav_info.estimator_status.attitude_status_flag
    //      && _uav_info.estimator_status.velocity_horiz_status_flag
    //       && _uav_info.estimator_status.velocity_vert_status_flag))
    // {
    //     ROS_INFO_STREAM_THROTTLE( 1, "Waiting for state estimation to complete..." );
    //     return ;
    // }

    if(_uav_info.extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
    {
        if(takeoff_id_)
        {
            takeoff_position_uav_.x = _uav_info.position.x;
            takeoff_position_uav_.y = _uav_info.position.y;
            takeoff_position_uav_.z = _uav_info.position.z + takeoff_relative_height_param_;
        }
        else
        {
            takeoff_position_uav_.x = takeoff_absolute_position_param_.x;
            takeoff_position_uav_.y = takeoff_absolute_position_param_.y;
            takeoff_position_uav_.z = takeoff_absolute_position_param_.z;
        }
    }

    if(!(abs(_uav_info.position.x - takeoff_position_uav_.x) < 0.2 &&
          abs(_uav_info.position.y - takeoff_position_uav_.y) < 0.2 &&
           abs(_uav_info.position.z - takeoff_position_uav_.z) < 0.2))    //认为起飞未完成
    {
        _command_deliver->header.stamp = ros::Time::now();
        _command_deliver->period = 0.05;
        _command_deliver->update = true;
        _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_PZ;
        _command_deliver->yaw_id = px4_application::UavCommand::NO_YAW;
        _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
        _command_deliver->x = takeoff_position_uav_.x;
        _command_deliver->y = takeoff_position_uav_.y;
        _command_deliver->z = takeoff_position_uav_.z;
        _command_deliver->yaw = 0;
        _command_deliver->task_name = "TakeOff";
        _uav_command_pub.publish(*_command_deliver);
        return ;
    }

    delete *_State;
    *_State = new Assemble;
}

TakeOff::TakeOff()
{
    ros::NodeHandle nh("~");
    nh.param<bool>("take_off/id", takeoff_id_, true);
    nh.param<double>("take_off/x", takeoff_absolute_position_param_.x, 0.0);
    nh.param<double>("take_off/y", takeoff_absolute_position_param_.y, 0.0);
    nh.param<double>("take_off/z", takeoff_absolute_position_param_.z, 1.0);
    nh.param<double>("take_off/h", takeoff_relative_height_param_, 1.0);

    std::cout << "Take off!" << std::endl;
}

TakeOff::~TakeOff()
{
    std::cout << "TakeOff to Assemble..." << std::endl;
}

/**
* @name         void Assemble::Run(const UavInfo& _uav_info,
                                    const ros::Publisher& _uav_command_pub,
                                     px4_application::UavCommand* _command_deliver,
                                      States** _State)
* @brief        集结任务接口（派生类）
* @param[in]    无人机状态：_uav_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   void
*/
void Assemble::Run(const UavInfo& _uav_info,
                    const ros::Publisher& _uav_command_pub,
                     px4_application::UavCommand* _command_deliver,
                      States** _State)
{
    if(!(abs(_uav_info.position.x - assemble_position_uav_.x) < 0.2 &&
          abs(_uav_info.position.y - assemble_position_uav_.y) < 0.2 &&
           abs(_uav_info.position.z - assemble_position_uav_.z) < 0.2))
    {
        _command_deliver->header.stamp = ros::Time::now();
        _command_deliver->period = 0.05;
        _command_deliver->update = true;
        _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_PZ;
        _command_deliver->yaw_id = px4_application::UavCommand::NO_YAW;
        _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
        _command_deliver->x = assemble_position_uav_.x;
        _command_deliver->y = assemble_position_uav_.y;
        _command_deliver->z = assemble_position_uav_.z;
        _command_deliver->yaw = 0;
        _command_deliver->task_name = "Assemble";
        _uav_command_pub.publish(*_command_deliver);
        return ;
    }

    delete *_State;
    *_State = new Tracking;
}

Assemble::Assemble()
{
    ros::NodeHandle nh("~");
    nh.param<double>("assemble/x", assemble_position_uav_.x, 15.0);
    nh.param<double>("assemble/y", assemble_position_uav_.y, 15.0);
    nh.param<double>("assemble/z", assemble_position_uav_.z, 10.0);

    std::cout << "Assemble!" << std::endl;
}

Assemble::~Assemble()
{
    std::cout << "Assemble to Tracking..." << std::endl;
}

/**
* @name         void Tracking::Run(const UavInfo& _uav_info,
                                    const ros::Publisher& _uav_command_pub,
                                     px4_application::UavCommand* _command_deliver,
                                      States** _State)
* @brief        追踪任务接口（派生类）
* @param[in]    无人机状态：_uav_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   void
*/
void Tracking::Run(const UavInfo& _uav_info,
                    const ros::Publisher& _uav_command_pub,
                     px4_application::UavCommand* _command_deliver,
                      States** _State)
{
    delete *_State;
    *_State = new ReturnHome;
}

Tracking::Tracking()
{
    std::cout << "Tracking!" << std::endl;
}

Tracking::~Tracking()
{
    std::cout << "Tracking to ReturnHome..." << std::endl;
}

/**
* @name         void ReturnHome::Run(const UavInfo& _uav_info,
                                      const ros::Publisher& _uav_command_pub,
                                       px4_application::UavCommand* _command_deliver,
                                        States** _State)
* @brief        返航任务接口（派生类）
* @param[in]    无人机状态：_uav_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   void
*/
void ReturnHome::Run(const UavInfo& _uav_info,
                      const ros::Publisher& _uav_command_pub,
                       px4_application::UavCommand* _command_deliver,
                        States** _State)
{
    if(!(abs(_uav_info.position.x - home_position_uav_.x) < 0.2 &&
          abs(_uav_info.position.y - home_position_uav_.y) < 0.2 &&
           abs(_uav_info.position.z - home_position_uav_.z) < 0.2))
    {
        _command_deliver->header.stamp = ros::Time::now();
        _command_deliver->period = 0.05;
        _command_deliver->update = true;
        _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_PZ;
        _command_deliver->yaw_id = px4_application::UavCommand::NO_YAW;
        _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
        _command_deliver->x = home_position_uav_.x;
        _command_deliver->y = home_position_uav_.y;
        _command_deliver->z = home_position_uav_.z;
        _command_deliver->yaw = 0;
        _command_deliver->task_name = "Return";
        _uav_command_pub.publish(*_command_deliver);
        return ;
    }

    delete *_State;
    *_State = new Landing;
}

ReturnHome::ReturnHome()
{
    ros::NodeHandle nh("~");
    nh.param<double>("home/x", home_position_uav_.x, 0.0);
    nh.param<double>("home/y", home_position_uav_.y, 0.0);
    nh.param<double>("home/z", home_position_uav_.z, 10.0);

    std::cout << "ReturnHome!" << std::endl;
}

ReturnHome::~ReturnHome()
{
    std::cout << "ReturnHome to Landing..." << std::endl;
}

/**
* @name         void Landing::Run(const UavInfo& _uav_info,
                                   const ros::Publisher& _uav_command_pub,
                                    px4_application::UavCommand* _command_deliver,
                                     States** _State)
* @brief        降落任务接口（派生类）
* @param[in]    无人机状态：_uav_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   void
*/
void Landing::Run(const UavInfo& _uav_info,
                   const ros::Publisher& _uav_command_pub,
                    px4_application::UavCommand* _command_deliver,
                     States** _State)
{
    if(!(_uav_info.extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND))    //未检测到着陆
    {
        _command_deliver->header.stamp = ros::Time::now();
        _command_deliver->period = 0.05;
        _command_deliver->update = true;
        // _command_deliver->xyz_id = px4_application::UavCommand::VX_VY_VZ;
        // _command_deliver->yaw_id = px4_application::UavCommand::NO_YAW;
        // _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
        // _command_deliver->x = 1.5*(landing_pos_vel_uav_.x - _uav_info.position.x);
        // _command_deliver->y = 1.5*(landing_pos_vel_uav_.y - _uav_info.position.y);
        // _command_deliver->z = landing_pos_vel_uav_.z;
        _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_VZ;
        _command_deliver->yaw_id = px4_application::UavCommand::NO_YAW;
        _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
        _command_deliver->x = landing_pos_vel_uav_.x;
        _command_deliver->y = landing_pos_vel_uav_.y;
        _command_deliver->z = landing_pos_vel_uav_.z;
        _command_deliver->yaw = 0;
        _command_deliver->task_name = "Landing";
        _uav_command_pub.publish(*_command_deliver);
        return ;
    }
    
    delete *_State;
    *_State = new Finished;
}

Landing::Landing()
{
    ros::NodeHandle nh("~");
    nh.param<double>("landing/x", landing_pos_vel_uav_.x, 0.0);
    nh.param<double>("landing/y", landing_pos_vel_uav_.y, 0.0);
    nh.param<double>("landing/vz", landing_pos_vel_uav_.z, -0.5);

    std::cout << "Landing!" << std::endl;
}

Landing::~Landing()
{
    std::cout << "Landing to Finished..." << std::endl;
}

/**
* @name         void Finished::Run(const UavInfo& _uav_info,
                                    const ros::Publisher& _uav_command_pub,
                                     px4_application::UavCommand* _command_deliver,
                                      States** _State)
* @brief        完成任务接口（派生类）
* @param[in]    无人机状态：_uav_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   void
*/
void Finished::Run(const UavInfo& _uav_info,
                    const ros::Publisher& _uav_command_pub,
                     px4_application::UavCommand* _command_deliver,
                      States** _State)
{
    _command_deliver->header.stamp = ros::Time::now();
    _command_deliver->period = 0.05;
    _command_deliver->update = false;
    _command_deliver->xyz_id = px4_application::UavCommand::UX_UY_UZ;
    _command_deliver->task_name = "Finished";
    _uav_command_pub.publish(*_command_deliver);
    // return ;

    // delete *_State;
    // *_State = NULL;

    // *_State = new TakeOff;
}

Finished::Finished()
{

    std::cout << "Finished!" << std::endl;
}

Finished::~Finished()
{
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_collaboration");
    ros::NodeHandle nh("~");
    UavCollaboration UavCollaboration(nh, 0.05);
    
    ros::spin();
    return 0;
}

