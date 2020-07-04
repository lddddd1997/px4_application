/** 
* @file     aruco_landing.cpp
* @brief    二维码引导降落
* @details  
* @note
* @author   lddddd
*           Email: lddddd1997@gmail.com
* @date     2020.6.8
* @version  1.0
* @par      Edit history:
*           1.0: lddddd, 2020.6.8, .
*/

#include "aruco_landing.h"

void ArucoLanding::ArucoDetectResultCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& _msg)    //订阅二维码位姿
{
	for(auto &item : _msg->markers)    //C++11新特性，遍历获取容器中的每个元素
	{
		if(item.id == 4)
		{
            double roll_temp, pitch_temp, yaw_temp;
            tf::Quaternion quat;
            position_central_marker_.x = item.pose.pose.position.x;
            position_central_marker_.y = item.pose.pose.position.y;
            position_central_marker_.z = item.pose.pose.position.z;

            tf::quaternionMsgToTF(item.pose.pose.orientation,quat);
            tf::Matrix3x3(quat).getRPY(roll_temp,pitch_temp,yaw_temp);
			yaw_central_marker_ = yaw_temp;
        }
    }
}

void ArucoLanding::UavPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg)
{
    position_uav_.x = _msg->pose.position.x;
    position_uav_.y = _msg->pose.position.y;
    position_uav_.z = _msg->pose.position.z;
}

void ArucoLanding::LoopTask(void)
{
    UavState_->StateMachineSchedule(position_uav_,
                                     position_central_marker_,
                                      yaw_central_marker_,
                                       uav_command_pub_,
                                        &command_deliver_,
                                         &UavState_);    //运行状态机调度
}

void ArucoLanding::Initialize(void)
{
    // loop_timer_ = nh_.createTimer(ros::Duration(loop_period_), &ArucoLanding::LoopTimerCallback, this); 
    uav_command_pub_ = nh_.advertise<px4_application::UavCommand>("/px4_application/uav_command", 10);
    marker_detect_sub_ = nh_.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker",
                                                                           10,
                                                                            &ArucoLanding::ArucoDetectResultCallback,
                                                                             this,
                                                                              ros::TransportHints().tcpNoDelay());
    uav_local_position_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",
                                                                         10,
                                                                          &ArucoLanding::UavPositionCallback,
                                                                           this,
                                                                            ros::TransportHints().tcpNoDelay());
    UavState_ = new TakeOff;    //初始为起飞状态
}

ArucoLanding::ArucoLanding(const ros::NodeHandle& _nh, double _period) : RosBase(_nh, _period)
{
    Initialize();
}

ArucoLanding::~ArucoLanding()
{
    if(UavState_ != NULL)
    {
        delete UavState_;
        UavState_ = NULL;
    }
}

/**
* @name         void States::StateMachineSchedule(const geometry_msgs::Vector3& _position_uav,
                                                   const ros::Publisher& _uav_command_pub,
                                                    px4_application::UavCommand* _command_deliver,
                                                     States** _State)
* @brief        简易状态机调度
* @param[in]    无人机ENU位置：_position_uav
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    无人机状态：_State
* @param[out]   void
*/
void States::StateMachineSchedule(const geometry_msgs::Vector3& _position_uav,
                                   const geometry_msgs::Vector3& _position_central_marker,
                                    float _yaw_central_marker,
                                     const ros::Publisher& _uav_command_pub,
                                      px4_application::UavCommand* _command_deliver,
                                       States** _State)
{
    Run(_position_uav, _position_central_marker, _yaw_central_marker, _uav_command_pub, _command_deliver, _State);
}

States::States()
{
    
}

States::~States()
{
    
}

/**
* @name         void TakeOff::Run(const geometry_msgs::Vector3& _position_uav,
                                   const ros::Publisher& _uav_command_pub,
                                    px4_application::UavCommand* _command_deliver,
                                     States** _State)
* @brief        起飞任务接口
* @param[in]    无人机ENU位置：_position_uav
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    无人机状态：_State
* @param[out]   void
*/
void TakeOff::Run(const geometry_msgs::Vector3& _position_uav,
                   const geometry_msgs::Vector3& _position_central_marker,
                    float _yaw_central_marker,
                     const ros::Publisher& _uav_command_pub,
                      px4_application::UavCommand* _command_deliver,
                       States** _State)
{
    if(!(abs(_position_uav.x - takeoff_position_uav_.x) < 0.2 &&
          abs(_position_uav.y - takeoff_position_uav_.y) < 0.2 && 
           abs(_position_uav.z - takeoff_position_uav_.z) < 0.2))    //认为起飞未完成
    {
        _command_deliver->period = 0.05;
        _command_deliver->update = true;
        _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_PZ;
        _command_deliver->yaw_id = px4_application::UavCommand::NO_YAW;
        _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
        _command_deliver->x = 0;
        _command_deliver->y = 0;
        _command_deliver->z = 5;
        _command_deliver->yaw = 0;
        _command_deliver->task_name = "TakeOff";
        _uav_command_pub.publish(*_command_deliver);
        return ;
    }

    delete *_State;
    *_State = new Searching;    //状态转移
}

TakeOff::TakeOff()
{
    ros::NodeHandle nh;
    nh.param<double>("take_off_x", takeoff_position_uav_.x, 0.0);
    nh.param<double>("take_off_y", takeoff_position_uav_.y, 0.0);
    nh.param<double>("take_off_z", takeoff_position_uav_.z, 5.0);

    std::cout << "TakeOff!" << std::endl;
}

TakeOff::~TakeOff()
{
    std::cout << "TakeOff to Searching..." << std::endl;
}

/**
* @name         void Searching::Run(const geometry_msgs::Vector3& _position_uav,
                                     const ros::Publisher& _uav_command_pub,
                                      px4_application::UavCommand* _command_deliver,
                                       States** _State)
* @brief        搜索任务接口
* @param[in]    无人机ENU位置：_position_uav
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    无人机状态：_State
* @param[out]   void
*/
void Searching::Run(const geometry_msgs::Vector3& _position_uav,
                     const geometry_msgs::Vector3& _position_central_marker,
                      float _yaw_central_marker,
                       const ros::Publisher& _uav_command_pub,
                        px4_application::UavCommand* _command_deliver,
                         States** _State)
{

    _command_deliver->period = 0.05;
    _command_deliver->update = true;
    _command_deliver->xyz_id = px4_application::UavCommand::VX_VY_PZ;
    _command_deliver->yaw_id = px4_application::UavCommand::NO_YAW;
    _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
    _command_deliver->x = 0.2;
    _command_deliver->y = 0.2;
    _command_deliver->z = 2;
    _command_deliver->yaw = 0;
    _command_deliver->task_name = "Searching";
    _uav_command_pub.publish(*_command_deliver);

    // delete *_State;
    // *_State = new Tracking;
}

Searching::Searching()
{
    std::cout << "Searching!" << std::endl;
}

Searching::~Searching()
{
    std::cout << "Searching to Tracking..." << std::endl;
}

/**
* @name         void Tracking::Run(const geometry_msgs::Vector3& _position_uav,
                                    const ros::Publisher& _uav_command_pub,
                                     px4_application::UavCommand* _command_deliver,
                                      States** _State)
* @brief        跟踪任务接口
* @param[in]    无人机ENU位置：_position_uav
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    无人机状态：_State
* @param[out]   void
*/
void Tracking::Run(const geometry_msgs::Vector3& _position_uav,
                    const geometry_msgs::Vector3& _position_central_marker,
                     float _yaw_central_marker,
                      const ros::Publisher& _uav_command_pub,
                       px4_application::UavCommand* _command_deliver,
                        States** _State)
{
    delete *_State;
    *_State = new Landing;
}

Tracking::Tracking()
{
    std::cout << "Tracking!" << std::endl;
}

Tracking::~Tracking()
{
    std::cout << "Tracking to Landing..." << std::endl;
}

/**
* @name         void Landing::Run(const geometry_msgs::Vector3& _position_uav,
                                   const ros::Publisher& _uav_command_pub,
                                    px4_application::UavCommand* _command_deliver,
                                     States** _State)
* @brief        降落任务接口
* @param[in]    无人机ENU位置：_position_uav
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    无人机状态：_State
* @param[out]   void
*/
void Landing::Run(const geometry_msgs::Vector3& _position_uav,
                   const geometry_msgs::Vector3& _position_central_marker,
                    float _yaw_central_marker,
                     const ros::Publisher& _uav_command_pub,
                      px4_application::UavCommand* _command_deliver,
                       States** _State)
{
    delete *_State;
    *_State = new Finished;
}

Landing::Landing()
{
    std::cout << "Landing!" << std::endl;
}

Landing::~Landing()
{
    std::cout << "Landing to Finished..." << std::endl;
}

/**
* @name         void Finished::Run(const geometry_msgs::Vector3& _position_uav,
                                    const ros::Publisher& _uav_command_pub,
                                     px4_application::UavCommand* _command_deliver,
                                      States** _State)
* @brief        完成任务接口
* @param[in]    无人机ENU位置：_position_uav
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    无人机状态：_State
* @param[out]   void
*/
void Finished::Run(const geometry_msgs::Vector3& _position_uav,
                    const geometry_msgs::Vector3& _position_central_marker,
                     float _yaw_central_marker,
                      const ros::Publisher& _uav_command_pub,
                       px4_application::UavCommand* _command_deliver,
                        States** _State)
{
    delete *_State;
    *_State = new TakeOff;
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
    ros::init(argc, argv, "aruco_landing");
    ros::NodeHandle nh("~");
    ArucoLanding ArucoLanding(nh, 0.05);
    
    ros::spin();
    return 0;
}

