/** 
* @file     uav_mission_template.cpp
* @brief    无人机任务模板程序
* @details  
* @note
* @author   lddddd
*           Email: lddddd1997@gmail.com
*           Github: https://github.com/lddddd1997
* @date     2020.11.9
* @version  1.0
* @par      Edit history:
*           1.0: lddddd, 2020.11.9, .
*/

#include "uav_mission_template.h"

void UavMission::LoopTask(void)
{
    this->UavState->StateMachineSchedule(this->current_info,
                                          this->uav_command_pub,
                                           &this->command_deliver,
                                            &this->UavState);    //运行状态机调度
}

void UavMission::Initialize(void)
{
    this->uav_command_pub = this->nh.advertise<px4_application::UavCommand>("px4_application/uav_command", 10);

    this->UavState = new Prepare;    //初始为准备状态
}

UavMission::UavMission(const ros::NodeHandle& _nh, double _period) : RosBase(_nh, _period)
{
    Initialize();
}

UavMission::~UavMission()
{
    if(this->UavState != NULL)
    {
        delete this->UavState;
        this->UavState = NULL;
    }
}

/**
* @name         void States::StateMachineSchedule(const StatusSubscriber& _current_info,
                                                   const ros::Publisher& _uav_command_pub,
                                                    px4_application::UavCommand* _command_deliver,
                                                     States** _State);
* @brief        简易状态机调度（基类）
* @param[in]    无人机与目标状态：_current_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL
*/
void States::StateMachineSchedule(const StatusSubscriber& _current_info,
                                   const ros::Publisher& _uav_command_pub,
                                    px4_application::UavCommand* _command_deliver,
                                     States** _State)
{
    Run(_current_info, _uav_command_pub, _command_deliver, _State);
}

States::States()
{
    ros::NodeHandle nh("~");
    nh.param<double>("range/x", this->reach_point_range.x, 0.1);
    nh.param<double>("range/y", this->reach_point_range.y, 0.1);
    nh.param<double>("range/z", this->reach_point_range.z, 0.1);
}

States::~States()
{
    
}

/**
* @name         void Prepare::Run(const StatusSubscriber& _current_info,
                                   const ros::Publisher& _uav_command_pub,
                                    px4_application::UavCommand* _command_deliver,
                                     States** _State)
* @brief        准备任务接口（派生类）
* @param[in]    无人机与目标状态：_current_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL  
*/
void Prepare::Run(const StatusSubscriber& _current_info,
                   const ros::Publisher& _uav_command_pub,
                    px4_application::UavCommand* _command_deliver,
                     States** _State)
{

    if(!(_current_info.uav_status.estimator_status.attitude_status_flag
         && _current_info.uav_status.estimator_status.velocity_horiz_status_flag
          && _current_info.uav_status.estimator_status.velocity_vert_status_flag))
    {
        _command_deliver->header.stamp = ros::Time::now();
        _command_deliver->period = 0.05;
        _command_deliver->update = true;
        _command_deliver->xyz_id = px4_application::UavCommand::UX_UY_UZ;
        _command_deliver->task_name = "Prepare";
        _command_deliver->yaw = _current_info.uav_status.attitude_angle.z;    //设置初始航向，起飞状态也会设置，在此设置是为了防止忽略起飞状态，直接跳到其他状态
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
* @name         void TakeOff::Run(const StatusSubscriber& _current_info,
                                   const ros::Publisher& _uav_command_pub,
                                    px4_application::UavCommand* _command_deliver,
                                     States** _State)
* @brief        起飞任务接口（派生类）
* @param[in]    无人机与目标状态：_current_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL
*/
void TakeOff::Run(const StatusSubscriber& _current_info,
                   const ros::Publisher& _uav_command_pub,
                    px4_application::UavCommand* _command_deliver,
                     States** _State)
{
    // if(!(_current_info.uav_status.estimator_status.attitude_status_flag
    //      && _current_info.uav_status.estimator_status.velocity_horiz_status_flag
    //       && _current_info.uav_status.estimator_status.velocity_vert_status_flag))
    // {
    //     ROS_INFO_STREAM_THROTTLE( 1, "Waiting for state estimation to complete..." );
    //     return ;
    // }

    if(_current_info.uav_status.extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
    {
        if(this->takeoff_id)
        {
            this->takeoff_position.x = _current_info.uav_status.local_position.x;
            this->takeoff_position.y = _current_info.uav_status.local_position.y;
            this->takeoff_position.z = _current_info.uav_status.local_position.z + this->takeoff_relative_height_param;
        }
        else
        {
            this->takeoff_position.x = this->takeoff_absolute_position_param.x;
            this->takeoff_position.y = this->takeoff_absolute_position_param.y;
            this->takeoff_position.z = this->takeoff_absolute_position_param.z;
        }
        _command_deliver->yaw = _current_info.uav_status.attitude_angle.z;    //设置为初始航向
    }

    if(!(abs(_current_info.uav_status.local_position.x - this->takeoff_position.x) < this->reach_point_range.x &&
          abs(_current_info.uav_status.local_position.y - this->takeoff_position.y) < this->reach_point_range.y &&
           abs(_current_info.uav_status.local_position.z - this->takeoff_position.z) < this->reach_point_range.z))    //认为起飞未完成
    {
        _command_deliver->header.stamp = ros::Time::now();    //发送时间戳
        _command_deliver->period = 0.05;    //发送指令的周期，暂未用到
        _command_deliver->update = true;    //指令是否更新
        _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_PZ;    //x y z的控制模式P对应位置控制，V对应速度控制，U未定义
        _command_deliver->yaw_id = px4_application::UavCommand::YAW;    //是否进行航向控制
        _command_deliver->frame_id = px4_application::UavCommand::LOCAL;    //控制坐标系选择
        _command_deliver->x = this->takeoff_position.x;    //x的指令信息
        _command_deliver->y = this->takeoff_position.y;    //y的指令信息
        _command_deliver->z = this->takeoff_position.z;    //z的指令信息
        // _command_deliver->yaw = 0;    //航向的指令信息
        _command_deliver->task_name = "TakeOff";    //任务名为TakeOff
        _uav_command_pub.publish(*_command_deliver);    //发送至飞控
        return ;
    }

    delete *_State;
    *_State = new Mission;    //切换到任务状态
}

TakeOff::TakeOff()
{
    ros::NodeHandle nh("~");
    nh.param<bool>("take_off/id", this->takeoff_id, true);
    nh.param<double>("take_off/x", this->takeoff_absolute_position_param.x, 0.0);
    nh.param<double>("take_off/y", this->takeoff_absolute_position_param.y, 0.0);
    nh.param<double>("take_off/z", this->takeoff_absolute_position_param.z, 1.0);
    nh.param<double>("take_off/h", this->takeoff_relative_height_param, 1.0);

    std::cout << "[ Take off ]" << std::endl;
}

TakeOff::~TakeOff()
{
    std::cout << "Takeoff state transition..." << std::endl;
}

/**
* @name         void Mission::Run(const StatusSubscriber& _current_info,
                                    const ros::Publisher& _uav_command_pub,
                                     px4_application::UavCommand* _command_deliver,
                                      States** _State)
* @brief        任务接口（派生类）
* @param[in]    无人机与目标状态：_current_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL
*/
void Mission::Run(const StatusSubscriber& _current_info,
                    const ros::Publisher& _uav_command_pub,
                     px4_application::UavCommand* _command_deliver,
                      States** _State)
{
    switch(this->own_id)
    {
        /*1号无人机任务*/
        case OtherSubscriber::uav_1 + 1:
        {
            if(_current_info.uav_status.state.mode != "OFFBOARD")
                _command_deliver->z = _current_info.uav_status.local_position.z;    //设定高度
            
            /*任务指令*/
            _command_deliver->header.stamp = ros::Time::now();
            _command_deliver->period = 0.05;
            _command_deliver->update = true;
            _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_PZ;
            _command_deliver->yaw_id = px4_application::UavCommand::YAW;
            _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
            _command_deliver->x = 50;
            _command_deliver->y = 50;
            _command_deliver->z = 5;
            // _command_deliver->yaw = lock_yaw_;
            _command_deliver->task_name = "1_Mission";
            _uav_command_pub.publish(*_command_deliver);
            return ;
        }
        break;
        /*2号无人机任务*/
        case OtherSubscriber::uav_2 + 1:
        {
            if(_current_info.uav_status.state.mode != "OFFBOARD")
                _command_deliver->z = _current_info.uav_status.local_position.z;    //设定高度

            /*任务指令*/
            _command_deliver->header.stamp = ros::Time::now();
            _command_deliver->period = 0.05;
            _command_deliver->update = true;
            _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_PZ;
            _command_deliver->yaw_id = px4_application::UavCommand::YAW;
            _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
            _command_deliver->x = total_info.uav_status[OtherSubscriber::uav_1].local_position.x;
            _command_deliver->y = total_info.uav_status[OtherSubscriber::uav_1].local_position.y;
            _command_deliver->z = total_info.uav_status[OtherSubscriber::uav_1].local_position.z;
            // _command_deliver->yaw = lock_yaw_;
            _command_deliver->task_name = "2_Mission";
            _uav_command_pub.publish(*_command_deliver);
            return ;
        }
        break;
        /*3号无人机任务*/
        case OtherSubscriber::uav_3 + 1:
        {
            if(_current_info.uav_status.state.mode != "OFFBOARD")
                _command_deliver->yaw = _current_info.uav_status.attitude_angle.z;    //设定航向

            /*任务指令*/
            _command_deliver->header.stamp = ros::Time::now();
            _command_deliver->period = 0.05;
            _command_deliver->update = true;
            _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_PZ;
            _command_deliver->yaw_id = px4_application::UavCommand::YAW;
            _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
            _command_deliver->x = total_info.uav_status[OtherSubscriber::uav_1].local_position.x;
            _command_deliver->y = total_info.uav_status[OtherSubscriber::uav_1].local_position.y;
            _command_deliver->z = total_info.uav_status[OtherSubscriber::uav_1].local_position.z;
            // _command_deliver->yaw = lock_yaw_;
            _command_deliver->task_name = "3_Mission";
            _uav_command_pub.publish(*_command_deliver);
            return ;
        }
        break;
        /*4号无人机任务*/
        case OtherSubscriber::uav_4 + 1:
        {
            if(_current_info.uav_status.state.mode != "OFFBOARD")
                _command_deliver->z = _current_info.uav_status.local_position.z;    //设定高度

            /*任务指令*/
            _command_deliver->header.stamp = ros::Time::now();
            _command_deliver->period = 0.05;
            _command_deliver->update = true;
            _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_PZ;
            _command_deliver->yaw_id = px4_application::UavCommand::YAW;
            _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
            _command_deliver->x = total_info.uav_status[OtherSubscriber::uav_1].local_position.x;
            _command_deliver->y = total_info.uav_status[OtherSubscriber::uav_1].local_position.y;
            _command_deliver->z = total_info.uav_status[OtherSubscriber::uav_1].local_position.z;
            // _command_deliver->yaw = lock_yaw_;
            _command_deliver->task_name = "4_Mission";
            _uav_command_pub.publish(*_command_deliver);
            return ;
        }
        break;
        /*5号无人机任务*/
        case OtherSubscriber::uav_5 + 1:
        {
            if(_current_info.uav_status.state.mode != "OFFBOARD")
                _command_deliver->z = _current_info.uav_status.local_position.z;    //设定高度

            /*任务指令*/
            _command_deliver->header.stamp = ros::Time::now();
            _command_deliver->period = 0.05;
            _command_deliver->update = true;
            _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_PZ;
            _command_deliver->yaw_id = px4_application::UavCommand::YAW;
            _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
            _command_deliver->x = total_info.uav_status[OtherSubscriber::uav_1].local_position.x;
            _command_deliver->y = total_info.uav_status[OtherSubscriber::uav_1].local_position.y;
            _command_deliver->z = total_info.uav_status[OtherSubscriber::uav_1].local_position.z;
            // _command_deliver->yaw = lock_yaw_;
            _command_deliver->task_name = "5_Mission";
            _uav_command_pub.publish(*_command_deliver);
            return ;
        }
        break;
        default: break;
    }

    // delete *_State;
    // *_State = new ReturnHome;
}

Mission::Mission()
{
    ros::NodeHandle nh("~");
    nh.param<int>("uav_id", this->own_id, 0);

    std::cout << "[ Mission ]" << std::endl;
}

Mission::~Mission()
{
    std::cout << "Mission state transition..." << std::endl;
}

/**
* @name         void ReturnHome::Run(const StatusSubscriber& _current_info,
                                      const ros::Publisher& _uav_command_pub,
                                       px4_application::UavCommand* _command_deliver,
                                        States** _State)
* @brief        返航任务接口（派生类）
* @param[in]    无人机与目标状态：_current_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL
*/
void ReturnHome::Run(const StatusSubscriber& _current_info,
                      const ros::Publisher& _uav_command_pub,
                       px4_application::UavCommand* _command_deliver,
                        States** _State)
{
    if(!(abs(_current_info.uav_status.local_position.x - this->home_position.x) < this->reach_point_range.x &&
          abs(_current_info.uav_status.local_position.y - this->home_position.y) < this->reach_point_range.y &&
           abs(_current_info.uav_status.local_position.z - this->home_position.z) < this->reach_point_range.z))
    {
        _command_deliver->header.stamp = ros::Time::now();
        _command_deliver->period = 0.05;
        _command_deliver->update = true;
        _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_PZ;
        _command_deliver->yaw_id = px4_application::UavCommand::YAW;
        _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
        _command_deliver->x = this->home_position.x;
        _command_deliver->y = this->home_position.y;
        _command_deliver->z = this->home_position.z;
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
    nh.param<double>("home/x", this->home_position.x, 0.0);
    nh.param<double>("home/y", this->home_position.y, 0.0);
    nh.param<double>("home/z", this->home_position.z, 5.0);

    std::cout << "[ ReturnHome ]" << std::endl;
}

ReturnHome::~ReturnHome()
{
    std::cout << "ReturnHome state transition..." << std::endl;
}

/**
* @name         void Landing::Run(const StatusSubscriber& _current_info,
                                   const ros::Publisher& _uav_command_pub,
                                    px4_application::UavCommand* _command_deliver,
                                     States** _State)
* @brief        降落任务接口（派生类）
* @param[in]    无人机与目标状态：_current_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL
*/
void Landing::Run(const StatusSubscriber& _current_info,
                   const ros::Publisher& _uav_command_pub,
                    px4_application::UavCommand* _command_deliver,
                     States** _State)
{
    if(!(_current_info.uav_status.extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND && !_current_info.uav_status.state.armed))    //未检测到着陆与上锁
    {
        _command_deliver->header.stamp = ros::Time::now();
        _command_deliver->period = 0.05;
        _command_deliver->update = true;
        _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_VZ;
        _command_deliver->yaw_id = px4_application::UavCommand::YAW;
        _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
        _command_deliver->x = this->landing_pos_vel.x;
        _command_deliver->y = this->landing_pos_vel.y;
        _command_deliver->z = this->landing_pos_vel.z;
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
    nh.param<double>("landing/x", this->landing_pos_vel.x, 0.0);
    nh.param<double>("landing/y", this->landing_pos_vel.y, 0.0);
    nh.param<double>("landing/vz", this->landing_pos_vel.z, -0.5);

    std::cout << "[ Landing ]" << std::endl;
}

Landing::~Landing()
{
    std::cout << "Landing state transition..." << std::endl;
}

/**
* @name         void Finished::Run(const StatusSubscriber& _current_info,
                                    const ros::Publisher& _uav_command_pub,
                                     px4_application::UavCommand* _command_deliver,
                                      States** _State)
* @brief        完成任务接口（派生类）
* @param[in]    无人机与目标状态：_current_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL
*/
void Finished::Run(const StatusSubscriber& _current_info,
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
    UavMission UavMission(nh, 0.05);
    
    ros::spin();
    return 0;
}

