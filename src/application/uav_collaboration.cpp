/** 
* @file     uav_collaboration.cpp
* @brief    无人机协同程序
* @details  
* @note
* @author   lddddd
*           Email: lddddd1997@gmail.com
*           Github: https://github.com/lddddd1997
* @date     2020.7.21
* @version  2.0
* @par      Edit history:
*           1.0: lddddd, 2020.6.28, .
*           2.0: lddddd, 2020.7.21, 更新节点句柄与topic的命名空间.
* TODO:     Add collaboration and visual algorithms.
*/

#include "uav_collaboration.h"

void UavCollaboration::LoopTask(void)
{
    UavState_->StateMachineSchedule(current_info_.status,
                                     uav_command_pub_,
                                      &command_deliver_,
                                       &UavState_);    //运行状态机调度
}

void UavCollaboration::Initialize(void)
{
    uav_command_pub_ = nh_.advertise<px4_application::UavCommand>("px4_application/uav_command", 10);

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
* @name         void States::StateMachineSchedule(const px4_application::UavStatus& _uav_info,
                                                   const ros::Publisher& _uav_command_pub,
                                                    px4_application::UavCommand* _command_deliver,
                                                     States** _State);
* @brief        简易状态机调度（基类）
* @param[in]    无人机状态：_uav_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL
*/
void States::StateMachineSchedule(const px4_application::UavStatus& _uav_info,
                                   const ros::Publisher& _uav_command_pub,
                                    px4_application::UavCommand* _command_deliver,
                                     States** _State)
{
    Run(_uav_info, _uav_command_pub, _command_deliver, _State);
}

States::States()
{
    ros::NodeHandle nh("~");
    nh.param<double>("range/x", reach_point_range_.x, 0.1);
    nh.param<double>("range/y", reach_point_range_.y, 0.1);
    nh.param<double>("range/z", reach_point_range_.z, 0.1);
}

States::~States()
{
    
}

/**
* @name         void Prepare::Run(const px4_application::UavStatus& _uav_info,
                                   const ros::Publisher& _uav_command_pub,
                                    px4_application::UavCommand* _command_deliver,
                                     States** _State)
* @brief        准备任务接口（派生类）
* @param[in]    无人机状态：_uav_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL  
*/
void Prepare::Run(const px4_application::UavStatus& _uav_info,
                   const ros::Publisher& _uav_command_pub,
                    px4_application::UavCommand* _command_deliver,
                     States** _State)
{

    if(!(_uav_info.estimator_status.attitude_status_flag
         && _uav_info.estimator_status.velocity_horiz_status_flag
          && _uav_info.estimator_status.velocity_vert_status_flag))
    {
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
    std::cout << "[ Prepare ]" << std::endl;
    ROS_ERROR("Waiting for state estimation to complete..." );
}

Prepare::~Prepare()
{
    std::cout << "Prepare state transition..." << std::endl;
}

/**
* @name         void TakeOff::Run(const px4_application::UavStatus& _uav_info,
                                   const ros::Publisher& _uav_command_pub,
                                    px4_application::UavCommand* _command_deliver,
                                     States** _State)
* @brief        起飞任务接口（派生类）
* @param[in]    无人机状态：_uav_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL
*/
void TakeOff::Run(const px4_application::UavStatus& _uav_info,
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
        _command_deliver->yaw = _uav_info.attitude_angle.z;    //设置为初始航向
    }

    if(!(abs(_uav_info.position.x - takeoff_position_uav_.x) < reach_point_range_.x &&
          abs(_uav_info.position.y - takeoff_position_uav_.y) < reach_point_range_.y &&
           abs(_uav_info.position.z - takeoff_position_uav_.z) < reach_point_range_.z))    //认为起飞未完成
    {
        _command_deliver->header.stamp = ros::Time::now();
        _command_deliver->period = 0.05;
        _command_deliver->update = true;
        _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_PZ;
        _command_deliver->yaw_id = px4_application::UavCommand::YAW;
        _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
        _command_deliver->x = takeoff_position_uav_.x;
        _command_deliver->y = takeoff_position_uav_.y;
        _command_deliver->z = takeoff_position_uav_.z;
        // _command_deliver->yaw = 0;
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

    std::cout << "[ Take off ]" << std::endl;
}

TakeOff::~TakeOff()
{
    std::cout << "Takeoff state transition..." << std::endl;
}

/**
* @name         void Assemble::Run(const px4_application::UavStatus& _uav_info,
                                    const ros::Publisher& _uav_command_pub,
                                     px4_application::UavCommand* _command_deliver,
                                      States** _State)
* @brief        集结任务接口（派生类）
* @param[in]    无人机状态：_uav_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL
*/
void Assemble::Run(const px4_application::UavStatus& _uav_info,
                    const ros::Publisher& _uav_command_pub,
                     px4_application::UavCommand* _command_deliver,
                      States** _State)
{
    if(!(abs(_uav_info.position.x - assemble_position_uav_.x) < reach_point_range_.x &&
          abs(_uav_info.position.y - assemble_position_uav_.y) < reach_point_range_.y &&
           abs(_uav_info.position.z - assemble_position_uav_.z) < reach_point_range_.z))
    {
        _command_deliver->header.stamp = ros::Time::now();
        _command_deliver->period = 0.05;
        _command_deliver->update = true;
        _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_PZ;
        _command_deliver->yaw_id = px4_application::UavCommand::YAW;
        _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
        _command_deliver->x = assemble_position_uav_.x;
        _command_deliver->y = assemble_position_uav_.y;
        _command_deliver->z = assemble_position_uav_.z;
        // _command_deliver->yaw = lock_yaw_;
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

    std::cout << "[ Assemble ]" << std::endl;
}

Assemble::~Assemble()
{
    std::cout << "Assemble state transition..." << std::endl;
}

/**
* @name         void Tracking::Run(const px4_application::UavStatus& _uav_info,
                                    const ros::Publisher& _uav_command_pub,
                                     px4_application::UavCommand* _command_deliver,
                                      States** _State)
* @brief        追踪任务接口（派生类）
* @param[in]    无人机状态：_uav_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL
*/
void Tracking::Run(const px4_application::UavStatus& _uav_info,
                    const ros::Publisher& _uav_command_pub,
                     px4_application::UavCommand* _command_deliver,
                      States** _State)
{
    delete *_State;
    *_State = new ReturnHome;
}

Tracking::Tracking()
{
    std::cout << "[ Tracking ]" << std::endl;
}

Tracking::~Tracking()
{
    std::cout << "Tracking state transition..." << std::endl;
}

/**
* @name         void ReturnHome::Run(const px4_application::UavStatus& _uav_info,
                                      const ros::Publisher& _uav_command_pub,
                                       px4_application::UavCommand* _command_deliver,
                                        States** _State)
* @brief        返航任务接口（派生类）
* @param[in]    无人机状态：_uav_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL
*/
void ReturnHome::Run(const px4_application::UavStatus& _uav_info,
                      const ros::Publisher& _uav_command_pub,
                       px4_application::UavCommand* _command_deliver,
                        States** _State)
{
    if(!(abs(_uav_info.position.x - home_position_uav_.x) < reach_point_range_.x &&
          abs(_uav_info.position.y - home_position_uav_.y) < reach_point_range_.y &&
           abs(_uav_info.position.z - home_position_uav_.z) < reach_point_range_.z))
    {
        _command_deliver->header.stamp = ros::Time::now();
        _command_deliver->period = 0.05;
        _command_deliver->update = true;
        _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_PZ;
        _command_deliver->yaw_id = px4_application::UavCommand::YAW;
        _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
        _command_deliver->x = home_position_uav_.x;
        _command_deliver->y = home_position_uav_.y;
        _command_deliver->z = home_position_uav_.z;
        // _command_deliver->yaw = lock_yaw_;
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
    nh.param<double>("home/z", home_position_uav_.z, 5.0);

    std::cout << "[ ReturnHome ]" << std::endl;
}

ReturnHome::~ReturnHome()
{
    std::cout << "ReturnHome state transition..." << std::endl;
}

/**
* @name         void Landing::Run(const px4_application::UavStatus& _uav_info,
                                   const ros::Publisher& _uav_command_pub,
                                    px4_application::UavCommand* _command_deliver,
                                     States** _State)
* @brief        降落任务接口（派生类）
* @param[in]    无人机状态：_uav_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL
*/
void Landing::Run(const px4_application::UavStatus& _uav_info,
                   const ros::Publisher& _uav_command_pub,
                    px4_application::UavCommand* _command_deliver,
                     States** _State)
{
    if(!(_uav_info.extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND && _uav_info.state.armed))    //未检测到着陆与上锁
    {
        _command_deliver->header.stamp = ros::Time::now();
        _command_deliver->period = 0.05;
        _command_deliver->update = true;
        _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_VZ;
        _command_deliver->yaw_id = px4_application::UavCommand::YAW;
        _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
        _command_deliver->x = landing_pos_vel_uav_.x;
        _command_deliver->y = landing_pos_vel_uav_.y;
        _command_deliver->z = landing_pos_vel_uav_.z;
        // _command_deliver->yaw = lock_yaw_;
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

    std::cout << "[ Landing ]" << std::endl;
}

Landing::~Landing()
{
    std::cout << "Landing state transition..." << std::endl;
}

/**
* @name         void Finished::Run(const px4_application::UavStatus& _uav_info,
                                    const ros::Publisher& _uav_command_pub,
                                     px4_application::UavCommand* _command_deliver,
                                      States** _State)
* @brief        完成任务接口（派生类）
* @param[in]    无人机状态：_uav_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL
*/
void Finished::Run(const px4_application::UavStatus& _uav_info,
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

    std::cout << "[ Finished ]" << std::endl;
}

Finished::~Finished()
{
    std::cout << "Finished state transition..." << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_collaboration");
    ros::NodeHandle nh;
    UavCollaboration UavCollaboration(nh, 0.05);
    
    ros::spin();
    return 0;
}

