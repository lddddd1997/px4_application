#ifndef PX4_APPLICATION_GCS_DISPLAY_H_
#define PX4_APPLICATION_GCS_DISPLAY_H_

#include <mavros_msgs/State.h>
#include <mavros_msgs/EstimatorStatus.h>
#include <mavros_msgs/ExtendedState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include "ros_base.h"
#include <tf/transform_datatypes.h>
#include "math_utils.h"
#include "px4_application/UavCommand.h"

class GcsDisplay : public RosBase
{
public:
    GcsDisplay(const ros::NodeHandle& _nh, const double _period);
    ~GcsDisplay();

private:
    //ros::NodeHandle nh_;
    ros::Time begin_time_;
    //ros::Timer loop_timer_;
    //double loop_period_;
    ros::Subscriber uav_state_sub_;
    ros::Subscriber uav_local_position_sub_;
    ros::Subscriber uav_local_velocity_sub_;
    ros::Subscriber uav_imu_sub_;
    ros::Subscriber uav_command_sub_;
    ros::Subscriber uav_estimator_sub_;
    ros::Subscriber uav_extended_state_sub_;

    mavros_msgs::State current_state_uav_;
    geometry_msgs::Vector3 local_position_uav_;
    geometry_msgs::Vector3 local_velocity_uav_;
    geometry_msgs::Quaternion quaternion_uav_;
    geometry_msgs::Vector3 attitude_angle_uav_;    //rad
    geometry_msgs::Vector3 attitude_rate_uav_;    //rad/s
    mavros_msgs::EstimatorStatus estimator_status_uav_;    //状态估计标记
    mavros_msgs::ExtendedState extended_state_uav_;    //扩展状态

    px4_application::UavCommand command_reception_;

    float GetTimePassSec(void);
    //void LoopTimerCallback(const ros::TimerEvent& event);
    void UavStateCallback(const mavros_msgs::State::ConstPtr& _msg);
    void UavPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg);
    void UavVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg);
    void UavImuCallback(const sensor_msgs::Imu::ConstPtr& _msg);
    void UavCommandCallback(const px4_application::UavCommand::ConstPtr& _msg);
    void EstimatorStatusCallback(const mavros_msgs::EstimatorStatus::ConstPtr& _msg);
    void ExtendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& _msg);

    void Initialize(void);
    void UavStateDisplay(void);
    void CommandUpdateReset(void);

    virtual void LoopTask(void);
};


#endif