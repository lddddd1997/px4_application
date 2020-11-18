#ifndef PX4_APPLICATION_STATUS_SUBSCRIBER_H_
#define PX4_APPLICATION_STATUS_SUBSCRIBER_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include "px4_application/UavStatus.h"
#include "px4_application/TargetStatus.h"
#include "math_utils.h"
// #include "MonoCamera/object.h"

class StatusSubscriber
{
public:
    StatusSubscriber();
    ~StatusSubscriber();
    px4_application::UavStatus uav_status;
    px4_application::TargetStatus target_status;

private:
    ros::NodeHandle nh;

    /*matlab display*/
    ros::Publisher matlab_pub;
    geometry_msgs::PoseStamped matlab_target_data;

    /*目标*/
    // ros::Subscriber target_sub;

    // void TargetDetectCallback(const MonoCamera::object::ConstPtr& _msg);

    /*无人机*/
    ros::Subscriber state_sub;
    ros::Subscriber local_position_sub;
    ros::Subscriber local_velocity_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber estimator_sub;
    ros::Subscriber extended_state_sub;

    void StateCallback(const mavros_msgs::State::ConstPtr& _msg);
    void PositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg);
    void VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg);
    void ImuCallback(const sensor_msgs::Imu::ConstPtr& _msg);
    void EstimatorStatusCallback(const mavros_msgs::EstimatorStatus::ConstPtr& _msg);
    void ExtendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& _msg);
};

StatusSubscriber::StatusSubscriber()
{
    this->state_sub = this->nh.subscribe<mavros_msgs::State>("mavros/state",
                                                              10,
                                                               &StatusSubscriber::StateCallback,
                                                                this,
                                                                 ros::TransportHints().tcpNoDelay());    //tcpNoDelay默认true降低延迟
    this->local_position_sub = this->nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",
                                                                               10,
                                                                                &StatusSubscriber::PositionCallback,
                                                                                 this,
                                                                                  ros::TransportHints().tcpNoDelay());
    this->local_velocity_sub = this->nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local",
                                                                                10,
                                                                                 &StatusSubscriber::VelocityCallback,
                                                                                  this,
                                                                                   ros::TransportHints().tcpNoDelay());
    this->imu_sub = this->nh.subscribe<sensor_msgs::Imu>("mavros/imu/data",
                                                          10,
                                                           &StatusSubscriber::ImuCallback,
                                                            this,
                                                             ros::TransportHints().tcpNoDelay());
    this->estimator_sub = this->nh.subscribe<mavros_msgs::EstimatorStatus>("mavros/estimator_status",
                                                                            10,
                                                                             &StatusSubscriber::EstimatorStatusCallback,
                                                                              this,
                                                                               ros::TransportHints().tcpNoDelay());
    this->extended_state_sub = this->nh.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state",
                                                                               10,
                                                                                &StatusSubscriber::ExtendedStateCallback,
                                                                                 this,
                                                                                  ros::TransportHints().tcpNoDelay());
    // this->target_sub = this->nh.subscribe<MonoCamera::object>("object_pub",
    //                                                            5,
    //                                                             &StatusSubscriber::TargetDetectCallback,
    //                                                              this,
    //                                                               ros::TransportHints().tcpNoDelay());
    this->matlab_pub = this->nh.advertise<geometry_msgs::PoseStamped>("matlab/display/target", 10);
}

StatusSubscriber::~StatusSubscriber()
{

}

void StatusSubscriber::StateCallback(const mavros_msgs::State::ConstPtr& _msg)
{
    this->uav_status.state = *_msg;
}

void StatusSubscriber::PositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg)
{
    this->uav_status.local_position.x = _msg->pose.position.x;
    this->uav_status.local_position.y = _msg->pose.position.y;
    this->uav_status.local_position.z = _msg->pose.position.z;
}

void StatusSubscriber::VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg)
{
    this->uav_status.local_velocity.x = _msg->twist.linear.x;
    this->uav_status.local_velocity.y = _msg->twist.linear.y;
    this->uav_status.local_velocity.z = _msg->twist.linear.z;
    MathUtils::Local2BodyHeading(this->uav_status.local_velocity, this->uav_status.body_heading_velocity, this->uav_status.attitude_angle.z);
}

void StatusSubscriber::ImuCallback(const sensor_msgs::Imu::ConstPtr& _msg)
{
    this->uav_status.attitude_rate.x = _msg->angular_velocity.x;
    this->uav_status.attitude_rate.y = _msg->angular_velocity.y;
    this->uav_status.attitude_rate.z = _msg->angular_velocity.z;

    tf::Quaternion quat(_msg->orientation.x, _msg->orientation.y, _msg->orientation.z, _msg->orientation.w);
    tf::Matrix3x3(quat).getRPY(this->uav_status.attitude_angle.x, this->uav_status.attitude_angle.y, this->uav_status.attitude_angle.z);    //四元数转欧拉角
}

void StatusSubscriber::EstimatorStatusCallback(const mavros_msgs::EstimatorStatus::ConstPtr& _msg)
{
    this->uav_status.estimator_status = *_msg;
}

void StatusSubscriber::ExtendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& _msg)
{
    this->uav_status.extended_state = *_msg;
}

// void StatusSubscriber::TargetDetectCallback(const MonoCamera::object::ConstPtr& _msg)
// {
//     this->target_status.update = _msg->isDetected;
//     this->target_status.number = _msg->object_number;
//     this->target_status.camera_position.x = _msg->object_position.x / 100.0;
//     this->target_status.camera_position.y = _msg->object_position.y / 100.0;
//     this->target_status.camera_position.z = _msg->object_position.z / 100.0;

//     this->matlab_target_data.pose.position.x = this->target_status.camera_position.x;
//     this->matlab_target_data.pose.position.y = this->target_status.camera_position.y;
//     this->matlab_target_data.pose.position.z = this->target_status.camera_position.z;
//     this->matlab_target_data.pose.orientation.w = this->target_status.update;
//     this->matlab_pub.publish(this->matlab_target_data);
// }

class OtherSubscriber
{
public:
    OtherSubscriber();
    ~OtherSubscriber();
    px4_application::UavStatus uav_status[5];
    px4_application::TargetStatus target_status;
    enum
    {
        uav_1 = 0u,
        uav_2 = 1u,
        uav_3 = 2u,
        uav_4 = 3u,
        uav_5 = 4u,
    };
private:
    ros::NodeHandle nh;

    /*无人机*/
    ros::Subscriber local_position_sub[5];
    ros::Subscriber local_velocity_sub[5];

    void Uav1PositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg);
    void Uav1VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg);

    void Uav2PositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg);
    void Uav2VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg);

    void Uav3PositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg);
    void Uav3VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg);

    void Uav4PositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg);
    void Uav4VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg);
    
    void Uav5PositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg);
    void Uav5VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg);

    /*目标*/
    // ros::Subscriber target_sub;
    // void TargetDetectCallback(const MonoCamera::object::ConstPtr& _msg);

};

OtherSubscriber::OtherSubscriber()
{
    this->local_position_sub[uav_1] = this->nh.subscribe<geometry_msgs::PoseStamped>("/uav1/mavros/local_position/pose",
                                                                                      10,
                                                                                       &OtherSubscriber::Uav1PositionCallback,
                                                                                        this,
                                                                                         ros::TransportHints().tcpNoDelay());
    this->local_velocity_sub[uav_1] = this->nh.subscribe<geometry_msgs::TwistStamped>("/uav1/mavros/local_position/velocity_local",
                                                                                       10,
                                                                                        &OtherSubscriber::Uav1VelocityCallback,
                                                                                         this,
                                                                                          ros::TransportHints().tcpNoDelay());
    this->local_position_sub[uav_2] = this->nh.subscribe<geometry_msgs::PoseStamped>("/uav2/mavros/local_position/pose",
                                                                                     10,
                                                                                      &OtherSubscriber::Uav2PositionCallback,
                                                                                       this,
                                                                                        ros::TransportHints().tcpNoDelay());
    this->local_velocity_sub[uav_2] = this->nh.subscribe<geometry_msgs::TwistStamped>("/uav2/mavros/local_position/velocity_local",
                                                                                      10,
                                                                                       &OtherSubscriber::Uav2VelocityCallback,
                                                                                        this,
                                                                                         ros::TransportHints().tcpNoDelay());
    this->local_position_sub[uav_3] = this->nh.subscribe<geometry_msgs::PoseStamped>("/uav3/mavros/local_position/pose",
                                                                                      10,
                                                                                       &OtherSubscriber::Uav3PositionCallback,
                                                                                        this,
                                                                                         ros::TransportHints().tcpNoDelay());
    this->local_velocity_sub[uav_3] = this->nh.subscribe<geometry_msgs::TwistStamped>("/uav3/mavros/local_position/velocity_local",
                                                                                       10,
                                                                                        &OtherSubscriber::Uav3VelocityCallback,
                                                                                         this,
                                                                                          ros::TransportHints().tcpNoDelay());
    this->local_position_sub[uav_4] = this->nh.subscribe<geometry_msgs::PoseStamped>("/uav4/mavros/local_position/pose",
                                                                                      10,
                                                                                       &OtherSubscriber::Uav4PositionCallback,
                                                                                        this,
                                                                                         ros::TransportHints().tcpNoDelay());
    this->local_velocity_sub[uav_4] = this->nh.subscribe<geometry_msgs::TwistStamped>("/uav4/mavros/local_position/velocity_local",
                                                                                       10,
                                                                                        &OtherSubscriber::Uav4VelocityCallback,
                                                                                         this,
                                                                                          ros::TransportHints().tcpNoDelay());
    this->local_position_sub[uav_5] = this->nh.subscribe<geometry_msgs::PoseStamped>("/uav5/mavros/local_position/pose",
                                                                                      10,
                                                                                       &OtherSubscriber::Uav5PositionCallback,
                                                                                        this,
                                                                                         ros::TransportHints().tcpNoDelay());
    this->local_velocity_sub[uav_5] = this->nh.subscribe<geometry_msgs::TwistStamped>("/uav5/mavros/local_position/velocity_local",
                                                                                       10,
                                                                                        &OtherSubscriber::Uav5VelocityCallback,
                                                                                         this,
                                                                                          ros::TransportHints().tcpNoDelay());
    // this->target_sub = this->nh.subscribe<MonoCamera::object>("/uav3/object_pub",
    //                                                            5,
    //                                                             &OtherSubscriber::TargetDetectCallback,
    //                                                              this,
    //                                                               ros::TransportHints().tcpNoDelay());
}

OtherSubscriber::~OtherSubscriber()
{

}

void OtherSubscriber::Uav1PositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg)
{
    this->uav_status[uav_1].local_position.x = _msg->pose.position.x;
    this->uav_status[uav_1].local_position.y = _msg->pose.position.y;
    this->uav_status[uav_1].local_position.z = _msg->pose.position.z;
}

void OtherSubscriber::Uav1VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg)
{
    this->uav_status[uav_1].local_velocity.x = _msg->twist.linear.x;
    this->uav_status[uav_1].local_velocity.y = _msg->twist.linear.y;
    this->uav_status[uav_1].local_velocity.z = _msg->twist.linear.z;
}

void OtherSubscriber::Uav2PositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg)
{
    this->uav_status[uav_2].local_position.x = _msg->pose.position.x;
    this->uav_status[uav_2].local_position.y = _msg->pose.position.y;
    this->uav_status[uav_2].local_position.z = _msg->pose.position.z;
}

void OtherSubscriber::Uav2VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg)
{
    this->uav_status[uav_2].local_velocity.x = _msg->twist.linear.x;
    this->uav_status[uav_2].local_velocity.y = _msg->twist.linear.y;
    this->uav_status[uav_2].local_velocity.z = _msg->twist.linear.z;
}

void OtherSubscriber::Uav3PositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg)
{
    this->uav_status[uav_3].local_position.x = _msg->pose.position.x;
    this->uav_status[uav_3].local_position.y = _msg->pose.position.y;
    this->uav_status[uav_3].local_position.z = _msg->pose.position.z;
}

void OtherSubscriber::Uav3VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg)
{
    this->uav_status[uav_3].local_velocity.x = _msg->twist.linear.x;
    this->uav_status[uav_3].local_velocity.y = _msg->twist.linear.y;
    this->uav_status[uav_3].local_velocity.z = _msg->twist.linear.z;
}

void OtherSubscriber::Uav4PositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg)
{
    this->uav_status[uav_4].local_position.x = _msg->pose.position.x;
    this->uav_status[uav_4].local_position.y = _msg->pose.position.y;
    this->uav_status[uav_4].local_position.z = _msg->pose.position.z;
}

void OtherSubscriber::Uav4VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg)
{
    this->uav_status[uav_4].local_velocity.x = _msg->twist.linear.x;
    this->uav_status[uav_4].local_velocity.y = _msg->twist.linear.y;
    this->uav_status[uav_4].local_velocity.z = _msg->twist.linear.z;
}

void OtherSubscriber::Uav5PositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg)
{
    this->uav_status[uav_5].local_position.x = _msg->pose.position.x;
    this->uav_status[uav_5].local_position.y = _msg->pose.position.y;
    this->uav_status[uav_5].local_position.z = _msg->pose.position.z;
}

void OtherSubscriber::Uav5VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg)
{
    this->uav_status[uav_5].local_velocity.x = _msg->twist.linear.x;
    this->uav_status[uav_5].local_velocity.y = _msg->twist.linear.y;
    this->uav_status[uav_5].local_velocity.z = _msg->twist.linear.z;
}

// void OtherSubscriber::TargetDetectCallback(const MonoCamera::object::ConstPtr& _msg)
// {
//     this->target_status.update = _msg->isDetected;
//     this->target_status.number = _msg->object_number;
//     this->target_status.camera_position.x = _msg->object_position.x / 100.0;
//     this->target_status.camera_position.y = _msg->object_position.y / 100.0;
//     this->target_status.camera_position.z = _msg->object_position.z / 100.0;
// }

#endif