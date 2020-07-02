#ifndef PX4_APPLICATION_UAV_COLLABORATION_H_
#define PX4_APPLICATION_UAV_COLLABORATION_H_

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/EstimatorStatus.h>
#include <mavros_msgs/ExtendedState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "px4_application/UavCommand.h"
#include "ros_base.h"

class States
{
public:
    States();
    virtual ~States();
    void StateMachineSchedule(const mavros_msgs::EstimatorStatus& _estimator_status_uav,
                               const mavros_msgs::ExtendedState& _extended_state_uav,
                                const geometry_msgs::Vector3& _position_uav,
                                 const geometry_msgs::Vector3& _velocity_uav,
                                  const ros::Publisher& _uav_command_pub,
                                   px4_application::UavCommand* _command_deliver,
                                    States** _State);    //注：使用指针的指针，确保能访问到对象指针本身，因为状态转移需delete与new操作
private:
    virtual void Run(const mavros_msgs::EstimatorStatus& _estimator_status_uav,
                      const mavros_msgs::ExtendedState& _extended_state_uav,
                       const geometry_msgs::Vector3& _position_uav,
                        const geometry_msgs::Vector3& _velocity_uav,
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
    virtual void Run(const mavros_msgs::EstimatorStatus& _estimator_status_uav,
                      const mavros_msgs::ExtendedState& _extended_state_uav,
                       const geometry_msgs::Vector3& _position_uav,
                        const geometry_msgs::Vector3& _velocity_uav,
                         const ros::Publisher& _uav_command_pub,
                          px4_application::UavCommand* _command_deliver,
                           States** _State);

    geometry_msgs::Vector3 takeoff_position_uav_;
    geometry_msgs::Vector3 takeoff_absolute_position_param_;    //绝对起飞位置
    double takeoff_relative_height_param_;    //相对起飞高度
    bool takeoff_id_;
};

class Assemble : public States
{
public:
    Assemble();
    ~Assemble();
private:
    virtual void Run(const mavros_msgs::EstimatorStatus& _estimator_status_uav,
                      const mavros_msgs::ExtendedState& _extended_state_uav,
                       const geometry_msgs::Vector3& _position_uav,
                        const geometry_msgs::Vector3& _velocity_uav,
                         const ros::Publisher& _uav_command_pub,
                          px4_application::UavCommand* _command_deliver,
                           States** _State);
    geometry_msgs::Vector3 assemble_position_uav_;
};

class Tracking : public States
{
public:
    Tracking();
    ~Tracking();
private:
    virtual void Run(const mavros_msgs::EstimatorStatus& _estimator_status_uav,
                      const mavros_msgs::ExtendedState& _extended_state_uav,
                       const geometry_msgs::Vector3& _position_uav,
                        const geometry_msgs::Vector3& _velocity_uav,
                         const ros::Publisher& _uav_command_pub,
                          px4_application::UavCommand* _command_deliver,
                           States** _State);
};

class ReturnHome : public States
{
public:
    ReturnHome();
    ~ReturnHome();
private:
    virtual void Run(const mavros_msgs::EstimatorStatus& _estimator_status_uav,
                      const mavros_msgs::ExtendedState& _extended_state_uav,
                       const geometry_msgs::Vector3& _position_uav,
                        const geometry_msgs::Vector3& _velocity_uav,
                         const ros::Publisher& _uav_command_pub,
                          px4_application::UavCommand* _command_deliver,
                           States** _State);
    geometry_msgs::Vector3 home_position_uav_;
};

class Landing : public States
{
public:
    Landing();
    ~Landing();
private:
    virtual void Run(const mavros_msgs::EstimatorStatus& _estimator_status_uav,
                      const mavros_msgs::ExtendedState& _extended_state_uav,
                       const geometry_msgs::Vector3& _position_uav,
                        const geometry_msgs::Vector3& _velocity_uav,
                         const ros::Publisher& _uav_command_pub,
                          px4_application::UavCommand* _command_deliver,
                           States** _State);
    geometry_msgs::Vector3 landing_pos_vel_uav_;
};

class Finished : public States
{
public:
    Finished();
    ~Finished();
private:
    // ros::NodeHandle nh_;
    // mavros_msgs::CommandBool uav_arm_cmd_;
    // ros::ServiceClient uav_arming_client_;
    // ros::Subscriber uav_state_sub_;

    virtual void Run(const mavros_msgs::EstimatorStatus& _estimator_status_uav,
                      const mavros_msgs::ExtendedState& _extended_state_uav,
                       const geometry_msgs::Vector3& _position_uav,
                        const geometry_msgs::Vector3& _velocity_uav,
                         const ros::Publisher& _uav_command_pub,
                          px4_application::UavCommand* _command_deliver,
                           States** _State);
};

class UavCollaboration : public RosBase 
{
public:
    UavCollaboration(const ros::NodeHandle& _nh, const double _period);
    ~UavCollaboration();
    
private:
    // ros::NodeHandle nh_;
    // ros::Timer loop_timer_;
    // double loop_period_;
    ros::Publisher uav_command_pub_;
    ros::Subscriber uav_local_position_sub_;
    ros::Subscriber uav_local_velocity_sub_;
    ros::Subscriber uav_estimator_sub_;
    ros::Subscriber uav_extended_state_sub_;

    px4_application::UavCommand command_deliver_;
    geometry_msgs::Vector3 position_uav_;
    geometry_msgs::Vector3 velocity_uav_;
    mavros_msgs::EstimatorStatus estimator_status_uav_;    //状态估计标记
    mavros_msgs::ExtendedState extended_state_uav_;    //扩展状态

    States* UavState_;

    // void LoopTimerCallback(const ros::TimerEvent& _event);
    void UavPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg);
    void UavVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg);
    void EstimatorStatusCallback(const mavros_msgs::EstimatorStatus::ConstPtr& _msg);
    void ExtendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& _msg);
    void Initialize(void);

    virtual void LoopTask(void);
};

#endif