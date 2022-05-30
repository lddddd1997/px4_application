#ifndef PX4_APPLICATION_STATUS_SUBSCRIBER_H_
#define PX4_APPLICATION_STATUS_SUBSCRIBER_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Eigen>
#include "px4_application/UavStatus.h"
#include "px4_application/TargetStatus.h"
#include "utilities/func_utils.h"
#include "utilities/filters_lp.h"

class StatusSubscriber
{
public:
    StatusSubscriber(const std::string& type);
    ~StatusSubscriber();
    px4_application::UavStatus uav_status;
    px4_application::TargetStatus drone_status;
    px4_application::TargetStatus door_status; // camera坐标系

    geometry_msgs::PoseStamped tf_drone_status;
    geometry_msgs::PoseStamped tf_door_status; // 转化到world坐标系下
    geometry_msgs::PoseStamped tf_door_after_status; // 门框后2m
    geometry_msgs::PoseStamped tf_door_before_status; // 门框前2m


private:
    ros::NodeHandle nh;

    /*目标*/
    ros::Subscriber yolo_drone_sub;
    void DroneStatusCallback(const px4_application::TargetStatus::ConstPtr& _msg);

    ros::Subscriber door_detection_sub;
    void DoorStatusCallback(const px4_application::TargetStatus::ConstPtr& _msg);
    
    Butterworth2 AccXFilter;
    Butterworth2 AccYFilter;
    Butterworth2 AccZFilter;

    /*无人机*/
    ros::Subscriber state_sub;
    ros::Subscriber local_position_sub;
    ros::Subscriber local_velocity_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber estimator_sub;
    ros::Subscriber extended_state_sub;
    tf::TransformBroadcaster tf_broadcaster;
    tf::TransformListener tf_listener;

    void StateCallback(const mavros_msgs::State::ConstPtr& _msg);
    void PositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg);
    void VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg);
    void ImuCallback(const sensor_msgs::Imu::ConstPtr& _msg);
    void EstimatorStatusCallback(const mavros_msgs::EstimatorStatus::ConstPtr& _msg);
    void ExtendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& _msg);
};

StatusSubscriber::StatusSubscriber(const std::string& type) : AccXFilter(100, 20), AccYFilter(100, 20), AccZFilter(100, 20)
{
    this->state_sub = this->nh.subscribe<mavros_msgs::State>("mavros/state",
                                                              10,
                                                               &StatusSubscriber::StateCallback,
                                                                this,
                                                                 ros::TransportHints().tcpNoDelay());    //tcpNoDelay默认true降低延迟
    this->local_position_sub = this->nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",
                                                                               1,
                                                                                &StatusSubscriber::PositionCallback,
                                                                                 this,
                                                                                  ros::TransportHints().tcpNoDelay());
    this->local_velocity_sub = this->nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local",
                                                                                1,
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
    if(this->nh.getNamespace() != "/uav1")
    {
        this->yolo_drone_sub = this->nh.subscribe<px4_application::TargetStatus>("detection_status/drone",
                                                                                  1,
                                                                                   &StatusSubscriber::DroneStatusCallback,
                                                                                    this,
                                                                                     ros::TransportHints().tcpNoDelay());
    }
    if(this->nh.getNamespace() == "/uav1")
    {
        this->door_detection_sub = this->nh.subscribe<px4_application::TargetStatus>("detection_status/door",
                                                                                      1,
                                                                                       &StatusSubscriber::DoorStatusCallback,
                                                                                        this,
                                                                                         ros::TransportHints().tcpNoDelay());
    }
    this->tf_drone_status.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    this->tf_door_status.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    this->tf_door_before_status.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    this->tf_door_after_status.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
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
    this->uav_status.header.stamp = _msg->header.stamp;
    this->uav_status.local_position.x = _msg->pose.position.x;
    this->uav_status.local_position.y = _msg->pose.position.y;
    this->uav_status.local_position.z = _msg->pose.position.z;
    this->uav_status.quat_pos = _msg->pose;
    tf_broadcaster.sendTransform(tf::StampedTransform(tf::Transform(
                                  tf::Quaternion(_msg->pose.orientation.x, _msg->pose.orientation.y,
                                                  _msg->pose.orientation.z, _msg->pose.orientation.w),
                                   tf::Vector3(_msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z)),
                                    ros::Time::now(), "world", this->nh.getNamespace() + "_body")); // w->b
    // tf_broadcaster.sendTransform(tf::StampedTransform(tf::Transform(
    //                               tf::inverse(tf::createQuaternionFromRPY(this->uav_status.attitude_angle.x, this->uav_status.attitude_angle.y, 0.0)), // 欧拉角为w->b的表征，故取逆转化成b->w，正常表示parent frame->child frame，inverse后，child frame->parent frame
    //                                tf::Vector3(0.0, 0.0, 0.0)),
    //                                 ros::Time::now(), this->nh.getNamespace() + "_body", this->nh.getNamespace() + "_body_heading"));
    // tf_broadcaster.sendTransform(tf::StampedTransform(tf::Transform(
    //                               tf::createQuaternionFromRPY(-1.57, 0.0, -1.57), // Z Y X顺序
    //                                tf::Vector3(0.2, 0.0, -0.2)),
    //                                 ros::Time::now(), this->nh.getNamespace() + "_body_heading", this->nh.getNamespace() + "_camera"));
    
    // if(this->nh.getNamespace() == "/uav1") // 领航无人机有云台，仿真中均有云台
    // {
        tf_broadcaster.sendTransform(tf::StampedTransform(tf::Transform( // 有云台
                                      tf::inverse(tf::createQuaternionFromRPY(this->uav_status.attitude_angle.x, this->uav_status.attitude_angle.y - 1.57, 1.57)), // 欧拉角为w->b的表征，故取逆转化成b->w
                                       tf::Vector3(0.2, 0.0, -0.3)),
                                        ros::Time::now(), this->nh.getNamespace() + "_body", this->nh.getNamespace() + "_camera")); // b->c
    // }
    // else // 跟随无人机无云台
    // {
    //     tf_broadcaster.sendTransform(tf::StampedTransform(tf::Transform( // 无云台，相机与无人机固联
    //                                   tf::createQuaternionFromRPY(-1.57, 0.0, -1.57), // Z Y X顺序
    //                                    tf::Vector3(0.2, 0.0, -0.2)),
    //                                     ros::Time::now(), this->nh.getNamespace() + "_body", this->nh.getNamespace() + "_camera"));
    // }
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
    tf::Matrix3x3(quat).getRPY(this->uav_status.attitude_angle.x, this->uav_status.attitude_angle.y, this->uav_status.attitude_angle.z);    //四元数转欧拉角ZYX
    
    tf::Matrix3x3 body2world(tf::Matrix3x3(quat).inverse());
    Eigen::Vector3d body_acc(_msg->linear_acceleration.x, _msg->linear_acceleration.y, _msg->linear_acceleration.z);
    Eigen::Matrix3d matrix_b2w;
    matrix_b2w << Eigen::Vector3d(body2world.getRow(0)[0], body2world.getRow(0)[1], body2world.getRow(0)[2]),
                Eigen::Vector3d(body2world.getRow(1)[0], body2world.getRow(1)[1], body2world.getRow(1)[2]),
                Eigen::Vector3d(body2world.getRow(2)[0], body2world.getRow(2)[1], body2world.getRow(2)[2]);

    Eigen::Vector3d world_acc = matrix_b2w * body_acc;
    this->uav_status.local_acceleration.x = world_acc[0];
    this->uav_status.local_acceleration.y = world_acc[1];
    this->uav_status.local_acceleration.z = world_acc[2] - 9.8;
    // this->uav_status.local_acceleration.x = this->AccXFilter.run(world_acc[0]);
    // this->uav_status.local_acceleration.y = this->AccYFilter.run(world_acc[1]);
    // this->uav_status.local_acceleration.z = this->AccZFilter.run(world_acc[2] - 9.8);
}

void StatusSubscriber::EstimatorStatusCallback(const mavros_msgs::EstimatorStatus::ConstPtr& _msg)
{
    this->uav_status.estimator_status = *_msg;
}

void StatusSubscriber::ExtendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& _msg)
{
    this->uav_status.extended_state = *_msg;
}

void StatusSubscriber::DroneStatusCallback(const px4_application::TargetStatus::ConstPtr& _msg)
{
    this->drone_status = *_msg;
    if(this->drone_status.update)
    {
        geometry_msgs::PoseStamped tf_before;
        tf_before.header.frame_id = this->nh.getNamespace() + "_camera";
        // tf_before.header.stamp = this->uav_status.header.stamp; // 强制时间戳同步
        tf_before.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
        tf_before.pose.position.x = this->drone_status.raw_pcl_position.x;
        tf_before.pose.position.y = this->drone_status.raw_pcl_position.y;
        tf_before.pose.position.z = this->drone_status.raw_pcl_position.z;

        if(tf_listener.frameExists("world") && tf_listener.frameExists(this->nh.getNamespace() + "_camera"))
        {
            tf_listener.transformPose("world", tf_before, tf_drone_status);
            // std::cout << "frame is exist." << std::endl;
        }
        else
        {
            // std::cout << "frame is not exist." << std::endl;
        }
    }
}

void StatusSubscriber::DoorStatusCallback(const px4_application::TargetStatus::ConstPtr& _msg)
{
    this->door_status = *_msg;
    if(this->door_status.update)
    {
        geometry_msgs::PoseStamped tf_before;
        tf_before.header.frame_id = this->nh.getNamespace() + "_camera";
        // tf_before.header.stamp = this->uav_status.header.stamp; // 强制时间戳同步
        tf_before.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, this->door_status.yaw, 0.0);
        // tf_before.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, tan(1.5 * this->door_status.yaw), 0.0); // 因为角度越大，越不准，加上tan函数映射（改到door_detection中)
        tf_before.pose.position.x = this->door_status.raw_pcl_position.x;
        tf_before.pose.position.y = this->door_status.raw_pcl_position.y;
        tf_before.pose.position.z = this->door_status.raw_pcl_position.z;

        if(tf_listener.frameExists(this->nh.getNamespace() + "_camera"))
        {
            tf_broadcaster.sendTransform(tf::StampedTransform(tf::Transform(
                                           tf::createQuaternionFromRPY(0.0, this->door_status.yaw, 0.0),
                                             tf::Vector3(tf_before.pose.position.x,
                                                           tf_before.pose.position.y,
                                                             tf_before.pose.position.z)),
                                                ros::Time::now(), this->nh.getNamespace() + "_camera", "det_door_central_link"));
        }
        else
        {
            tf_broadcaster.sendTransform(tf::StampedTransform(tf::Transform( // 如果未更新，则在无限远处
                                           tf::Quaternion(0.0, 0.0, 0.0, 1.0),
                                             tf::Vector3(10000.0,
                                                           10000.0,
                                                             10000.0)),
                                                ros::Time::now(), "world", "det_door__link_0"));
        }

        if(tf_listener.frameExists("world") && tf_listener.frameExists(this->nh.getNamespace() + "_camera")) // 转化相机坐标系的门框位置到世界坐标系
        {
            tf_listener.transformPose("world", tf_before, tf_door_status);

            double sin_phi = sin(this->door_status.yaw), cos_phi = cos(this->door_status.yaw);
            tf_before.pose.position.x += 2.0 * sin_phi;
            tf_before.pose.position.z += 2.0 * cos_phi;
            tf_listener.transformPose("world", tf_before, tf_door_after_status);

            tf_before.pose.position.x -= 4.0 * sin_phi;
            tf_before.pose.position.z -= 4.0 * cos_phi;
            tf_listener.transformPose("world", tf_before, tf_door_before_status);
            
            // tf_door_status.pose.position.z += 0.2;
            // tf_door_before_status.pose.position.z += 0.2;
            // tf_door_after_status.pose.position.z += 0.2; // 加上0.2m的相机偏移 // 实物要修改！
            // std::cout << "frame is exist." << std::endl;
        }
        else
        {
            // std::cout << "frame is not exist." << std::endl;
        }
    }
}



























































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
    // this->local_position_sub[uav_5] = this->nh.subscribe<geometry_msgs::PoseStamped>("/uav5/mavros/local_position/pose",
    //                                                                                   10,
    //                                                                                    &OtherSubscriber::Uav5PositionCallback,
    //                                                                                     this,
    //                                                                                      ros::TransportHints().tcpNoDelay());
    // this->local_velocity_sub[uav_5] = this->nh.subscribe<geometry_msgs::TwistStamped>("/uav5/mavros/local_position/velocity_local",
    //                                                                                    10,
    //                                                                                     &OtherSubscriber::Uav5VelocityCallback,
    //                                                                                      this,
    //                                                                                       ros::TransportHints().tcpNoDelay());
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
    tf::Quaternion quat(_msg->pose.orientation.x, _msg->pose.orientation.y, _msg->pose.orientation.z, _msg->pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(this->uav_status[uav_1].attitude_angle.x,
                                this->uav_status[uav_1].attitude_angle.y,
                                 this->uav_status[uav_1].attitude_angle.z);    //四元数转欧拉角ZYX
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
//     this->target_status.pcl_position.x = _msg->object_position.x / 100.0;
//     this->target_status.pcl_position.y = _msg->object_position.y / 100.0;
//     this->target_status.pcl_position.z = _msg->object_position.z / 100.0;
// }

#endif