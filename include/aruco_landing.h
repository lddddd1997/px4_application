#ifndef PX4_APPLICATION_ARUCO_LANDING_H_
#define PX4_APPLICATION_ARUCO_LANDING_H_

#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <tf/transform_datatypes.h>
#include "px4_application/UavCommand.h"
#include "ros_base.h"

class States
{
public:
    States();
    virtual ~States();
    void StateMachineSchedule(const geometry_msgs::Vector3& _position_uav,
                               const geometry_msgs::Vector3& _position_central_marker,
                                const float _yaw_central_marker,
                                 const ros::Publisher& _uav_command_pub,
                                  px4_application::UavCommand* _command_deliver,
                                   States** _State);    //注：使用指针的指针，确保能访问到对象指针本身，因为状态转移需delete与new操作
private:
    virtual void Run(const geometry_msgs::Vector3& _position_uav,
                      const geometry_msgs::Vector3& _position_central_marker,
                       const float _yaw_central_marker,
                        const ros::Publisher& _uav_command_pub,
                         px4_application::UavCommand* _command_deliver,
                          States** _State) = 0;
};

class TakeOff : public States
{
public:
    TakeOff();
    ~TakeOff();
private:
    virtual void Run(const geometry_msgs::Vector3& _position_uav,
                      const geometry_msgs::Vector3& _position_central_marker,
                       const float _yaw_central_marker,
                        const ros::Publisher& _uav_command_pub,
                         px4_application::UavCommand* _command_deliver,
                          States** _State);
    geometry_msgs::Vector3 takeoff_position_uav_;
};

class Searching : public States
{
public:
    Searching();
    ~Searching();
private:
    virtual void Run(const geometry_msgs::Vector3& _position_uav,
                      const geometry_msgs::Vector3& _position_central_marker,
                       const float _yaw_central_marker,
                        const ros::Publisher& _uav_command_pub,
                         px4_application::UavCommand* _command_deliver,
                          States** _State);
};

class Tracking : public States
{
public:
    Tracking();
    ~Tracking();
private:
    virtual void Run(const geometry_msgs::Vector3& _position_uav,
                      const geometry_msgs::Vector3& _position_central_marker,
                       const float _yaw_central_marker,
                        const ros::Publisher& _uav_command_pub,
                         px4_application::UavCommand* _command_deliver,
                          States** _State);
};

class Landing : public States
{
public:
    Landing();
    ~Landing();
private:
    virtual void Run(const geometry_msgs::Vector3& _position_uav,
                      const geometry_msgs::Vector3& _position_central_marker,
                       const float _yaw_central_marker,
                        const ros::Publisher& _uav_command_pub,
                         px4_application::UavCommand* _command_deliver,
                          States** _State);
};

class Finished : public States
{
public:
    Finished();
    ~Finished();
private:
    virtual void Run(const geometry_msgs::Vector3& _position_uav,
                      const geometry_msgs::Vector3& _position_central_marker,
                       const float _yaw_central_marker,
                        const ros::Publisher& _uav_command_pub,
                         px4_application::UavCommand* _command_deliver,
                          States** _State);
};

class ArucoLanding : public RosBase 
{
public:
    ArucoLanding(const ros::NodeHandle& _nh, const double _period);
    ~ArucoLanding();
    
private:
    // ros::NodeHandle nh_;
    // ros::Timer loop_timer_;
    // double loop_period_;
    ros::Publisher uav_command_pub_;
    ros::Subscriber marker_detect_sub_;
    ros::Subscriber uav_local_position_sub_;

    px4_application::UavCommand command_deliver_;
    geometry_msgs::Vector3 position_central_marker_;
    float yaw_central_marker_;
    geometry_msgs::Vector3 position_uav_;

    States* UavState_;

    // void LoopTimerCallback(const ros::TimerEvent& _event);
    void ArucoDetectResultCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& _msg);
    void UavPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg);
    void Initialize(void);

    virtual void LoopTask(void);
};

#endif