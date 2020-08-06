#ifndef PX4_APPLICATION_UAV_STATUS_SUBSCRIBER_H_
#define PX4_APPLICATION_UAV_STATUS_SUBSCRIBER_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include"px4_application/UavStatus.h"


class StatusSubscriber
{
public:
    StatusSubscriber();
    ~StatusSubscriber();
    px4_application::UavStatus status;

private:
    ros::NodeHandle nh_;

    ros::Subscriber uav_state_sub_;
    ros::Subscriber uav_local_position_sub_;
    ros::Subscriber uav_local_velocity_sub_;
    ros::Subscriber uav_imu_sub_;
    ros::Subscriber uav_estimator_sub_;
    ros::Subscriber uav_extended_state_sub_;

    void UavStateCallback(const mavros_msgs::State::ConstPtr& _msg);
    void UavPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg);
    void UavVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg);
    void UavImuCallback(const sensor_msgs::Imu::ConstPtr& _msg);
    void EstimatorStatusCallback(const mavros_msgs::EstimatorStatus::ConstPtr& _msg);
    void ExtendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& _msg);
};

StatusSubscriber::StatusSubscriber()
{
    uav_state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state",
                                                        10,
                                                         &StatusSubscriber::UavStateCallback,
                                                          this,
                                                           ros::TransportHints().tcpNoDelay());   //tcpNoDelay默认true降低延迟
    uav_local_position_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",
                                                                         10,
                                                                          &StatusSubscriber::UavPositionCallback,
                                                                           this,
                                                                            ros::TransportHints().tcpNoDelay());
    uav_local_velocity_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local",
                                                                          10,
                                                                           &StatusSubscriber::UavVelocityCallback,
                                                                            this,
                                                                             ros::TransportHints().tcpNoDelay());
    uav_imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("mavros/imu/data",
                                                    10,
                                                     &StatusSubscriber::UavImuCallback,
                                                      this,
                                                       ros::TransportHints().tcpNoDelay());
    uav_estimator_sub_ = nh_.subscribe<mavros_msgs::EstimatorStatus>("mavros/estimator_status",
                                                                      10,
                                                                       &StatusSubscriber::EstimatorStatusCallback,
                                                                        this,
                                                                         ros::TransportHints().tcpNoDelay());
    uav_extended_state_sub_ = nh_.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state",
                                                                         10,
                                                                          &StatusSubscriber::ExtendedStateCallback,
                                                                           this,
                                                                            ros::TransportHints().tcpNoDelay());
}

StatusSubscriber::~StatusSubscriber()
{

}

void StatusSubscriber::UavStateCallback(const mavros_msgs::State::ConstPtr& _msg)
{
    status.state = *_msg;
}

void StatusSubscriber::UavPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg)
{
    status.position.x = _msg->pose.position.x;
    status.position.y = _msg->pose.position.y;
    status.position.z = _msg->pose.position.z;
}

void StatusSubscriber::UavVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg)
{
    status.velocity.x = _msg->twist.linear.x;
    status.velocity.y = _msg->twist.linear.y;
    status.velocity.z = _msg->twist.linear.z;
}

void StatusSubscriber::UavImuCallback(const sensor_msgs::Imu::ConstPtr& _msg)
{
    status.attitude_rate.x = _msg->angular_velocity.x;
    status.attitude_rate.y = _msg->angular_velocity.y;
    status.attitude_rate.z = _msg->angular_velocity.z;

    tf::Quaternion quat(_msg->orientation.x, _msg->orientation.y, _msg->orientation.z, _msg->orientation.w);
    tf::Matrix3x3(quat).getRPY(status.attitude_angle.x, status.attitude_angle.y, status.attitude_angle.z);    //四元数转欧拉角
}

void StatusSubscriber::EstimatorStatusCallback(const mavros_msgs::EstimatorStatus::ConstPtr& _msg)
{
    status.estimator_status = *_msg;
}

void StatusSubscriber::ExtendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& _msg)
{
    status.extended_state = *_msg;
}

#endif