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
* TODO:     Add collaboration.
*/

#include "uav_collaboration.h"

void UavCollaboration::LoopTask(void)
{
    this->UavState->StateMachineSchedule(this->current_info,
                                          this->uav_command_pub,
                                           &this->command_deliver,
                                            &this->UavState);    //运行状态机调度
}

void UavCollaboration::Initialize(void)
{
    this->uav_command_pub = this->nh.advertise<px4_application::UavCommand>("px4_application/uav_command", 10);

    this->UavState = new Prepare;    //初始为准备状态
}

UavCollaboration::UavCollaboration(const ros::NodeHandle& _nh, double _period) : RosBase(_nh, _period)
{
    Initialize();
}

UavCollaboration::~UavCollaboration()
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
    // *_State = new Tracking;    //实验直接切换到追踪状态

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
        _command_deliver->header.stamp = ros::Time::now();
        _command_deliver->period = 0.05;
        _command_deliver->update = true;
        _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_PZ;
        _command_deliver->yaw_id = px4_application::UavCommand::YAW;
        _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
        _command_deliver->x = this->takeoff_position.x;
        _command_deliver->y = this->takeoff_position.y;
        _command_deliver->z = this->takeoff_position.z;
        // _command_deliver->yaw = 0;
        _command_deliver->task_name = "TakeOff";
        _uav_command_pub.publish(*_command_deliver);
        return ;
    }

    delete *_State;
    // *_State = new Assemble;
    *_State = new Tracking;
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
* @name         void Assemble::Run(const StatusSubscriber& _current_info,
                                    const ros::Publisher& _uav_command_pub,
                                     px4_application::UavCommand* _command_deliver,
                                      States** _State)
* @brief        集结任务接口（派生类）
* @param[in]    无人机与目标状态：_current_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL
*/
void Assemble::Run(const StatusSubscriber& _current_info,
                    const ros::Publisher& _uav_command_pub,
                     px4_application::UavCommand* _command_deliver,
                      States** _State)
{
    if(!(abs(_current_info.uav_status.local_position.x - this->assemble_position.x) < this->reach_point_range.x &&
          abs(_current_info.uav_status.local_position.y - this->assemble_position.y) < this->reach_point_range.y &&
           abs(_current_info.uav_status.local_position.z - this->assemble_position.z) < this->reach_point_range.z))
    {
        _command_deliver->header.stamp = ros::Time::now();
        _command_deliver->period = 0.05;
        _command_deliver->update = true;
        _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_PZ;
        _command_deliver->yaw_id = px4_application::UavCommand::YAW;
        _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
        _command_deliver->x = this->assemble_position.x;
        _command_deliver->y = this->assemble_position.y;
        _command_deliver->z = this->assemble_position.z;
        // _command_deliver->yaw = lock_yaw_;
        _command_deliver->task_name = "Assemble";
        _uav_command_pub.publish(*_command_deliver);
        return ;
    }

    delete *_State;
    // *_State = new Tracking;
    *_State = new ReturnHome;
}

Assemble::Assemble()
{
    ros::NodeHandle nh("~");
    nh.param<double>("assemble/x", this->assemble_position.x, 15.0);
    nh.param<double>("assemble/y", this->assemble_position.y, 15.0);
    nh.param<double>("assemble/z", this->assemble_position.z, 10.0);

    std::cout << "[ Assemble ]" << std::endl;
}

Assemble::~Assemble()
{
    std::cout << "Assemble state transition..." << std::endl;
}

/**
* @name         void Tracking::Run(const StatusSubscriber& _current_info,
                                    const ros::Publisher& _uav_command_pub,
                                     px4_application::UavCommand* _command_deliver,
                                      States** _State)
* @brief        追踪任务接口（派生类）
* @param[in]    无人机与目标状态：_current_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL
*/
void Tracking::Run(const StatusSubscriber& _current_info,
                    const ros::Publisher& _uav_command_pub,
                     px4_application::UavCommand* _command_deliver,
                      States** _State)
{
    if(this->debug_id)
    {
        _command_deliver->header.stamp = ros::Time::now();
        _command_deliver->period = 0.05;
        _command_deliver->update = true;
        _command_deliver->xyz_id = px4_application::UavCommand::VX_VY_VZ;
        _command_deliver->yaw_id = px4_application::UavCommand::YAW;
        _command_deliver->frame_id = px4_application::UavCommand::BODY;
        _command_deliver->x = this->tracking_position.x;
        _command_deliver->y = this->tracking_position.y;
        _command_deliver->z = this->tracking_position.z;
        // _command_deliver->yaw = lock_yaw_;
        _command_deliver->task_name = "Debug";
        _uav_command_pub.publish(*_command_deliver);
        if(_current_info.uav_status.state.mode == "OFFBOARD")
        {
            FunctionUtils::DataFileWrite(this->tracking_position, _current_info.uav_status.body_heading_velocity, this->saved_file_path + "debug_controller.txt");
        }
        return ;
    }
    else
    {   
        switch(this->own_id)
        {
            /*1号无人机追踪3号无人机*/
            /*2号无人机追踪数字牌*/
            case OtherSubscriber::uav_1 + 1:
            {
                // if(_current_info.uav_status.state.mode != "OFFBOARD")
                // {
                //     _command_deliver->yaw = _current_info.uav_status.attitude_angle.z;    //设定航向
                //     _command_deliver->z = _current_info.uav_status.local_position.z;    //设定高度
                // }
                
                // double output_x = 1.5 * (this->total_info.uav_status[OtherSubscriber::uav_3].local_position.x - _current_info.uav_status.local_position.x) + this->total_info.uav_status[OtherSubscriber::uav_3].local_velocity.x;
                // double output_y = 1.5 * (this->total_info.uav_status[OtherSubscriber::uav_3].local_position.y - _current_info.uav_status.local_position.y) + this->total_info.uav_status[OtherSubscriber::uav_3].local_velocity.y;
                // output_x = MathUtils::Constrain(output_x, 2.0);
                // output_y = MathUtils::Constrain(output_y, 2.0);

                // _command_deliver->header.stamp = ros::Time::now();
                // _command_deliver->period = 0.05;
                // _command_deliver->update = true;
                // _command_deliver->xyz_id = px4_application::UavCommand::VX_VY_PZ;
                // _command_deliver->yaw_id = px4_application::UavCommand::YAW;
                // _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
                // _command_deliver->x = output_x;
                // _command_deliver->y = output_y;
                // // _command_deliver->z = this->total_info.uav_status[2].local_position.z;
                // // _command_deliver->yaw = 0;
                // _command_deliver->task_name = "Follow_3";
                // _uav_command_pub.publish(*_command_deliver);
                // return ;
                if(_current_info.uav_status.state.mode != "OFFBOARD")
                    _command_deliver->yaw = _current_info.uav_status.attitude_angle.z;    //设定航向
                _command_deliver->header.stamp = ros::Time::now();
                _command_deliver->period = 0.05;
                _command_deliver->update = true;
                _command_deliver->xyz_id = px4_application::UavCommand::VX_VY_VZ;
                _command_deliver->yaw_id = px4_application::UavCommand::YAW;
                _command_deliver->frame_id = px4_application::UavCommand::BODY;
                _command_deliver->x = -this->TrackingX.ControlOutput(this->tracking_position.x, _current_info.target_status.camera_position.x);
                _command_deliver->y = -this->TrackingY.ControlOutput(this->tracking_position.y, _current_info.target_status.camera_position.y);
                _command_deliver->z = -this->TrackingZ.ControlOutput(this->tracking_position.z, _current_info.target_status.camera_position.z);
                // _command_deliver->yaw = lock_yaw_;
                _command_deliver->task_name = "1_Tracking";
                _uav_command_pub.publish(*_command_deliver);
                if(_current_info.uav_status.state.mode == "OFFBOARD")
                {
                    geometry_msgs::Vector3 saved_expect;
                    saved_expect.x = _command_deliver->x;
                    saved_expect.y = _command_deliver->y;
                    saved_expect.z = _command_deliver->z;
                    FunctionUtils::DataFileWrite(this->tracking_position, _current_info.target_status.camera_position, this->saved_file_path + "camera_tracking.txt");
                    FunctionUtils::DataFileWrite(saved_expect, _current_info.uav_status.body_heading_velocity, this->saved_file_path + "controller_tracking.txt");
                }
                return;
            }
            break;
            /*2号无人机追踪3号无人机*/
            /*2号无人机追踪数字牌*/
            case OtherSubscriber::uav_2 + 1:
            {
                // if(_current_info.uav_status.state.mode != "OFFBOARD")
                // {
                //     _command_deliver->yaw = _current_info.uav_status.attitude_angle.z;    //设定航向
                //     _command_deliver->z = _current_info.uav_status.local_position.z;    //设定高度
                // }

                // double output_x = 1.5 * (this->total_info.uav_status[OtherSubscriber::uav_3].local_position.x - _current_info.uav_status.local_position.x) + this->total_info.uav_status[OtherSubscriber::uav_3].local_velocity.x;
                // double output_y = 1.5 * (this->total_info.uav_status[OtherSubscriber::uav_3].local_position.y - _current_info.uav_status.local_position.y) + this->total_info.uav_status[OtherSubscriber::uav_3].local_velocity.y;
                // output_x = MathUtils::Constrain(output_x, 2.0);
                // output_y = MathUtils::Constrain(output_y, 2.0);

                // _command_deliver->header.stamp = ros::Time::now();
                // _command_deliver->period = 0.05;
                // _command_deliver->update = true;
                // _command_deliver->xyz_id = px4_application::UavCommand::VX_VY_PZ;
                // _command_deliver->yaw_id = px4_application::UavCommand::YAW;
                // _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
                // _command_deliver->x = output_x;
                // _command_deliver->y = output_y;
                // // _command_deliver->z = this->total_info.uav_status[2].local_position.z;
                // // _command_deliver->yaw = 0;
                // _command_deliver->task_name = "Follow_3";
                // _uav_command_pub.publish(*_command_deliver);
                // return ;
                if(_current_info.uav_status.state.mode != "OFFBOARD")
                    _command_deliver->yaw = _current_info.uav_status.attitude_angle.z;    //设定航向
                _command_deliver->header.stamp = ros::Time::now();
                _command_deliver->period = 0.05;
                _command_deliver->update = true;
                _command_deliver->xyz_id = px4_application::UavCommand::VX_VY_VZ;
                _command_deliver->yaw_id = px4_application::UavCommand::YAW;
                _command_deliver->frame_id = px4_application::UavCommand::BODY;
                _command_deliver->x = -this->TrackingX.ControlOutput(this->tracking_position.x, _current_info.target_status.camera_position.x);
                _command_deliver->y = -this->TrackingY.ControlOutput(this->tracking_position.y, _current_info.target_status.camera_position.y);
                _command_deliver->z = -this->TrackingZ.ControlOutput(this->tracking_position.z, _current_info.target_status.camera_position.z);
                // _command_deliver->yaw = lock_yaw_;
                _command_deliver->task_name = "2_Tracking";
                _uav_command_pub.publish(*_command_deliver);
                if(_current_info.uav_status.state.mode == "OFFBOARD")
                {
                    geometry_msgs::Vector3 saved_expect;
                    saved_expect.x = _command_deliver->x;
                    saved_expect.y = _command_deliver->y;
                    saved_expect.z = _command_deliver->z;
                    FunctionUtils::DataFileWrite(this->tracking_position, _current_info.target_status.camera_position, this->saved_file_path + "camera_tracking.txt");
                    FunctionUtils::DataFileWrite(saved_expect, _current_info.uav_status.body_heading_velocity, this->saved_file_path + "controller_tracking.txt");
                }
                return;
            }
            break;
            /*3号无人机追踪电子数字牌*/
            case OtherSubscriber::uav_3 + 1:
            {
                if(_current_info.uav_status.state.mode != "OFFBOARD")
                    _command_deliver->yaw = _current_info.uav_status.attitude_angle.z;    //设定航向
                if(_current_info.target_status.number != 0)
                {
                    this->tracking_position.x = 4.0;
                }
                else
                {
                    this->tracking_position.x = 5.0;
                }
                
                _command_deliver->header.stamp = ros::Time::now();
                _command_deliver->period = 0.05;
                _command_deliver->update = true;
                _command_deliver->xyz_id = px4_application::UavCommand::VX_VY_VZ;
                _command_deliver->yaw_id = px4_application::UavCommand::YAW;
                _command_deliver->frame_id = px4_application::UavCommand::BODY;
                _command_deliver->x = -this->TrackingX.ControlOutput(this->tracking_position.x, _current_info.target_status.camera_position.x);
                _command_deliver->y = -this->TrackingY.ControlOutput(this->tracking_position.y, _current_info.target_status.camera_position.y);
                _command_deliver->z = -this->TrackingZ.ControlOutput(this->tracking_position.z, _current_info.target_status.camera_position.z);
                // _command_deliver->yaw = lock_yaw_;
                _command_deliver->task_name = "3_Tracking";
                _uav_command_pub.publish(*_command_deliver);
                if(_current_info.uav_status.state.mode == "OFFBOARD")
                {
                    geometry_msgs::Vector3 saved_expect;
                    saved_expect.x = _command_deliver->x;
                    saved_expect.y = _command_deliver->y;
                    saved_expect.z = _command_deliver->z;
                    FunctionUtils::DataFileWrite(this->tracking_position, _current_info.target_status.camera_position, this->saved_file_path + "camera_tracking.txt");
                    FunctionUtils::DataFileWrite(saved_expect, _current_info.uav_status.body_heading_velocity, this->saved_file_path + "controller_tracking.txt");
                }
                return;
            }
            break;
            /*4号无人机追踪3号无人机*/
            case OtherSubscriber::uav_4 + 1:
            {
                if(_current_info.uav_status.state.mode != "OFFBOARD")
                {
                    _command_deliver->yaw = _current_info.uav_status.attitude_angle.z;    //设定航向
                    _command_deliver->z = _current_info.uav_status.local_position.z;    //设定高度
                }
                double offset_x = 0.0;
                if(this->total_info.target_status.number != 2)
                {
                    offset_x = -2.0;
                }
                else
                {
                    offset_x = 0.0;
                }
                
                double output_x = 1.5 * (offset_x + this->total_info.uav_status[OtherSubscriber::uav_3].local_position.x - _current_info.uav_status.local_position.x) + this->total_info.uav_status[OtherSubscriber::uav_3].local_velocity.x;
                double output_y = 1.5 * (this->total_info.uav_status[OtherSubscriber::uav_3].local_position.y - _current_info.uav_status.local_position.y) + this->total_info.uav_status[OtherSubscriber::uav_3].local_velocity.y;
                output_x = MathUtils::Constrain(output_x, 2.0);
                output_y = MathUtils::Constrain(output_y, 2.0);

                _command_deliver->header.stamp = ros::Time::now();
                _command_deliver->period = 0.05;
                _command_deliver->update = true;
                _command_deliver->xyz_id = px4_application::UavCommand::VX_VY_PZ;
                _command_deliver->yaw_id = px4_application::UavCommand::YAW;
                _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
                _command_deliver->x = output_x;
                _command_deliver->y = output_y;
                // _command_deliver->z = this->total_info.uav_status[2].local_position.z;
                // _command_deliver->yaw = 0;
                _command_deliver->task_name = "Follow_3";
                _uav_command_pub.publish(*_command_deliver);
                return ;
            }
            break;
            /*5号无人机追踪2号无人机*/
            case OtherSubscriber::uav_5 + 1:
            {
                if(_current_info.uav_status.state.mode != "OFFBOARD")
                {
                    _command_deliver->yaw = _current_info.uav_status.attitude_angle.z;    //设定航向
                    _command_deliver->z = _current_info.uav_status.local_position.z;    //设定高度
                }

                double output_x = 1.5 * (this->total_info.uav_status[OtherSubscriber::uav_2].local_position.x - _current_info.uav_status.local_position.x) + this->total_info.uav_status[OtherSubscriber::uav_2].local_velocity.x;
                double output_y = 1.5 * (this->total_info.uav_status[OtherSubscriber::uav_2].local_position.y - _current_info.uav_status.local_position.y) + this->total_info.uav_status[OtherSubscriber::uav_2].local_velocity.y;
                output_x = MathUtils::Constrain(output_x, 2.0);
                output_y = MathUtils::Constrain(output_y, 2.0);

                _command_deliver->header.stamp = ros::Time::now();
                _command_deliver->period = 0.05;
                _command_deliver->update = true;
                _command_deliver->xyz_id = px4_application::UavCommand::VX_VY_PZ;
                _command_deliver->yaw_id = px4_application::UavCommand::YAW;
                _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
                _command_deliver->x = output_x;
                _command_deliver->y = output_y;
                // _command_deliver->z = this->total_info.uav_status[2].local_position.z;
                // _command_deliver->yaw = 0;
                _command_deliver->task_name = "Follow_2";
                _uav_command_pub.publish(*_command_deliver);
                return ;
            }
            break;
            default: break;
        }
    }
    
    delete *_State;
    *_State = new ReturnHome;
}

Tracking::Tracking() : TrackingX(PidController::NORMAL),
                        TrackingY(PidController::NORMAL),
                         TrackingZ(PidController::NORMAL)
{
    ros::NodeHandle nh("~");
    nh.param<int>("uav_id", this->own_id, 0);
    nh.param<bool>("tracking/debug", this->debug_id, true);
    nh.param<std::string>("tracking/saved_file_path", this->saved_file_path, "/home/ld/px4_ws/src/px4_application/");
    nh.param<double>("tracking/x", this->tracking_position.x, 4.0);
    nh.param<double>("tracking/y", this->tracking_position.y, 0.0);
    nh.param<double>("tracking/z", this->tracking_position.z, -2.0);
    nh.param<double>("tracking/yaw", this->tracking_yaw, 0.0);

    nh.param<double>("threshold/x", this->tracking_threshold.x, 0.3);
    nh.param<double>("threshold/y", this->tracking_threshold.y, 0.3);
    nh.param<double>("threshold/z", this->tracking_threshold.z, 0.3);

    PidParameters param;
    nh.param<float>("pid_xy/tracking/kp", param.kp, 1.0);
    nh.param<float>("pid_xy/tracking/ki", param.ki, 0.0);
    nh.param<float>("pid_xy/tracking/kd", param.kd, 0.0);
    nh.param<float>("pid_xy/tracking/ff", param.ff, 0.0);
    nh.param<float>("pid_xy/tracking/error_max", param.error_max, 10.0);
    nh.param<float>("pid_xy/tracking/integral_max", param.integral_max, 5.0);
    nh.param<float>("pid_xy/tracking/output_max", param.output_max, 8.0);    //QGC XY速度最大期望默认10m/s
    this->TrackingX.SetParameters(param);
    this->TrackingY.SetParameters(param);
    nh.param<float>("pid_z/tracking/kp", param.kp, 1.0);
    nh.param<float>("pid_z/tracking/ki", param.ki, 0.0);
    nh.param<float>("pid_z/tracking/kd", param.kd, 0.0);
    nh.param<float>("pid_z/tracking/ff", param.ff, 0.0);
    nh.param<float>("pid_z/tracking/error_max", param.error_max, 3.0);
    nh.param<float>("pid_z/tracking/integral_max", param.integral_max, 2.0);
    nh.param<float>("pid_z/tracking/output_max", param.output_max, 2.0);    //QGC Z速度最大上升默认3m/s 下降1m/s
    this->TrackingZ.SetParameters(param);
    std::cout << "----------X tracking controller parameters----------" << std::endl;
    this->TrackingX.PrintParameters();
    std::cout << "----------Y tracking controller parameters----------" << std::endl;
    this->TrackingY.PrintParameters();
    std::cout << "----------Z tracking controller parameters----------" << std::endl;
    this->TrackingZ.PrintParameters();

    std::cout << "[ Tracking ]" << std::endl;
}

Tracking::~Tracking()
{
    std::cout << "Tracking state transition..." << std::endl;
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
    UavCollaboration UavCollaboration(nh, 0.05);
    
    ros::spin();
    return 0;
}

