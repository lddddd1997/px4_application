#include "pub/traj_publisher.h"

// void TrajectoryPublisher::StateCallback(const mavros_msgs::State::ConstPtr& _msg)
// {
//     this->curr_mode = _msg->mode;
//     this->armed = _msg->armed;
// }

// void TrajectoryPublisher::DoorStatusCallback(const px4_application::TargetStatus::ConstPtr& _msg)
// {
//     this->door_update = _msg->update;
    
//     if(this->door_update)
//     {
//         geometry_msgs::PoseStamped tf_before, tf_door_status;
//         tf_before.header.frame_id = this->nh.getNamespace() + "_camera";
//         // tf_before.header.stamp = this->uav_status.header.stamp; // 强制时间戳同步
//         tf_before.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, _msg->yaw, 0.0);
//         // tf_before.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, tan(1.5 * this->door_status.yaw), 0.0); // 因为角度越大，越不准，加上tan函数映射（改到door_detection中)
//         tf_before.pose.position.x = _msg->raw_pcl_position.x;
//         tf_before.pose.position.y = _msg->raw_pcl_position.y;
//         tf_before.pose.position.z = _msg->raw_pcl_position.z;

//         if(this->tf_listener.frameExists("world") && this->tf_listener.frameExists(this->nh.getNamespace() + "_camera"))
//         {
//             this->tf_listener.transformPose("world", tf_before, tf_door_status);
//             // std::cout << "frame is exist." << std::endl;
//         }
//         this->door_enu_yaw = tf::getYaw(tf_door_status.pose.orientation) + 1.57;
//     }
// }

void TrajectoryPublisher::BoundaryCondCallback(const px4_application::BoundaryConditions::ConstPtr& _msg)
{
    std::vector<min_jerk::BoundaryCond> bd_conds;
    std::vector<double> durs;
    for(auto& item : _msg->bound_conds)
    {
        min_jerk::BoundaryCond bd;
        bd.setZero();
        bd.col(0) = TypeTransform::RosMsg2Eigen(item.p0);
        bd.col(1) = TypeTransform::RosMsg2Eigen(item.v0);
        bd.col(2) = TypeTransform::RosMsg2Eigen(item.a0);
        bd.col(3) = TypeTransform::RosMsg2Eigen(item.pt);
        bd.col(4) = TypeTransform::RosMsg2Eigen(item.vt);
        bd.col(5) = TypeTransform::RosMsg2Eigen(item.at);
        bd_conds.push_back(bd);
        durs.push_back(item.duration);
    }

    // std::cout << this->total_duration << " " << (ros::Time::now() - this->start_time).toSec() << std::endl;

    min_jerk::Trajectory ref_traj(bd_conds, durs);
    this->min_jerk_traj.Restructure(ref_traj);
    this->total_duration = this->min_jerk_traj.getTotalDuration();
    this->start_time = ros::Time::now();
    this->start_time -= ros::Duration(this->forward_sample_time); // 前向采样，解决重规划导致的低速问题

    // 轨迹显示
    nav_msgs::Path rviz_min_jerk_path;
    TrajectoryUtils<min_jerk::Trajectory>::TrajectorySample(this->min_jerk_traj, 0.1, rviz_min_jerk_path);
    this->min_jerk_traj_visualization.PublishMessage(rviz_min_jerk_path);

    // std::cout << "replan..." << std::endl;
}

void TrajectoryPublisher::LoopTask(void)
{
    // if(this->min_jerk_traj.getPieceNum() == 0)
    // {
    //     ROS_ERROR_ONCE("Waiting for trajectory...");
    //     return ;
    // }

    // this->curr_time = ros::Time::now();
    // double traj_time = (this->curr_time - this->start_time).toSec();
    // // if(traj_time < 0.1) // 前向采样
    // // {
    // //     traj_time = 0.1;
    // //     this->start_time -= ros::Duration(0.1);
    // // }
    // if(traj_time > this->total_duration)
    //     traj_time = this->total_duration;
    // Eigen::Vector3d target_pos = this->min_jerk_traj.getPos(traj_time);
    // Eigen::Vector3d target_vel = this->min_jerk_traj.getVel(traj_time);
    // Eigen::Vector3d target_acc = this->min_jerk_traj.getAcc(traj_time);
    // TypeTransform::Eigen2RosMsg(target_pos, this->flat_target.position);
    // TypeTransform::Eigen2RosMsg(target_vel, this->flat_target.velocity);
    // TypeTransform::Eigen2RosMsg(target_acc, this->flat_target.acceleration);
    // if(abs(traj_time - this->total_duration) > 1e-6)
    // {
    //     this->flat_target.yaw = atan2(target_vel[1], target_vel[0]); // 目标不更新则使用速度航向
    // }
    // // if(this->door_update)
    // // {
    // //     this->flat_target.yaw = this->door_enu_yaw;
    // // }
    // this->flat_target.header.stamp = ros::Time::now();
    // this->flat_target.type_mask = px4_application::FlatTarget::IGNORE_SNAP_JERK;
    // this->flat_target_pub.publish(this->flat_target);
    
}

void TrajectoryPublisher::LoopTaskWithoutVirtual(void)
{
    if(this->min_jerk_traj.getPieceNum() == 0)
    {
        ROS_ERROR_ONCE("Waiting for trajectory...");
        return ;
    }

    this->curr_time = ros::Time::now();
    double traj_time = (this->curr_time - this->start_time).toSec();
    // if(traj_time < 0.1) // 前向采样
    // {
    //     traj_time = 0.1;
    //     this->start_time -= ros::Duration(0.1);
    // }
    if(traj_time > this->total_duration)
        traj_time = this->total_duration;
    Eigen::Vector3d target_pos = this->min_jerk_traj.getPos(traj_time);
    Eigen::Vector3d target_vel = this->min_jerk_traj.getVel(traj_time);
    Eigen::Vector3d target_acc = this->min_jerk_traj.getAcc(traj_time);
    TypeTransform::Eigen2RosMsg(target_pos, this->flat_target.position);
    TypeTransform::Eigen2RosMsg(target_vel, this->flat_target.velocity);
    TypeTransform::Eigen2RosMsg(target_acc, this->flat_target.acceleration);
    if(abs(traj_time - this->total_duration) > 1e-6)
    {
        this->flat_target.yaw = atan2(target_vel[1], target_vel[0]); // 目标不更新则使用速度航向
    }
    // if(this->door_update)
    // {
    //     this->flat_target.yaw = this->door_enu_yaw;
    // }
    this->flat_target.header.stamp = ros::Time::now();
    this->flat_target.type_mask = px4_application::FlatTarget::IGNORE_SNAP_JERK;
    this->flat_target_pub.publish(this->flat_target);
}

void TrajectoryPublisher::Initialize(void)
{
    // this->state_sub = this->nh.subscribe<mavros_msgs::State>("mavros/state",
    //                                                           10,
    //                                                            &TrajectoryPublisher::StateCallback,
    //                                                             this,
    //                                                              ros::TransportHints().tcpNoDelay());
    this->ref_traj_sub = this->nh.subscribe<px4_application::BoundaryConditions>("reference/boundary_conds/min_jerk",
                                                                                  10,
                                                                                   &TrajectoryPublisher::BoundaryCondCallback,
                                                                                    this,
                                                                                     ros::TransportHints().tcpNoDelay());
    this->flat_target_pub = this->nh.advertise<px4_application::FlatTarget>("reference/flat_setpoint", 1);

    // this->door_detection_sub = this->nh.subscribe<px4_application::TargetStatus>("detection_status/door",
    //                                                                               1,
    //                                                                                &TrajectoryPublisher::DoorStatusCallback,
    //                                                                                 this,
    //                                                                                  ros::TransportHints().tcpNoDelay());
    ros::NodeHandle nh_temp("~");
    nh_temp.param<double>("forward_sample_time", this->forward_sample_time, 0.1);
    std::cout << "forward sample time: " << this->forward_sample_time << std::endl;
    std::cout << "----------------------------------" << std::endl;
}

TrajectoryPublisher::TrajectoryPublisher(const ros::NodeHandle& _nh, double _period) : RosBase(_nh, _period), min_jerk_traj_visualization(this->nh, "trajectory_visualization/min_jerk")
{
    Initialize();
}

TrajectoryPublisher::~TrajectoryPublisher()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_publisher");
    ros::NodeHandle nh;

    TrajectoryPublisher TrajectoryPublisher(nh, 1.0);
    // ros::spin();

    double ros_rate = 1.0;
    if(nh.getNamespace() == "/uav1")
    {
        ros::NodeHandle nh_temp("~");
        nh_temp.param<double>("leader_spin_rate/traj_publisher", ros_rate, 1.0);
    }
    else
    {
        ros::NodeHandle nh_temp("~");
        nh_temp.param<double>("folower_spin_rate/traj_publisher", ros_rate, 1.0);
    }

    std::cout << "-----------------------" << std::endl;
    std::cout << "ros rate: " << ros_rate << std::endl;
    ros::Rate loop(ros_rate);
    while(ros::ok())
    {
        ros::spinOnce();
        TrajectoryPublisher.LoopTaskWithoutVirtual();
        loop.sleep();
    }
    return 0;
}

