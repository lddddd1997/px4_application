#include "pub/form_publisher.h"


Eigen::Vector3d FormationOutput(const OtherSubscriber& total_info, int uav_id)
{
    Eigen::Vector3d vel_deisred;
    vel_deisred.setZero();
    Eigen::Vector3d own_vel;
    own_vel = TypeTransform::RosMsg2Eigen(total_info.uav_status[uav_id].local_velocity);
    Eigen::Vector3d own_pos;
    own_pos = TypeTransform::RosMsg2Eigen(total_info.uav_status[uav_id].local_position);
    // vel_deisred += 0.5 * (TypeTransform::RosMsg2Eigen(total_info.uav_status[OtherSubscriber::uav_1].local_velocity) - own_vel);
    vel_deisred += 1.0 * (TypeTransform::RosMsg2Eigen(total_info.uav_status[OtherSubscriber::uav_1].local_velocity) - own_vel);
    for(int i = 1; i < 4; i++)
    {
        // vel_deisred += 0.5 * (TypeTransform::RosMsg2Eigen(total_info.uav_status[i].local_velocity) - own_vel);
        vel_deisred += 0.1 * (TypeTransform::RosMsg2Eigen(total_info.uav_status[i].local_velocity) - own_vel);
    }

    double leader_yaw = total_info.uav_status[OtherSubscriber::uav_1].attitude_angle.z;
    double sin_yaw = sin(leader_yaw);
    double cos_yaw = cos(leader_yaw);

    //      1
    //      2
    //      3
    //      4
    // Eigen::Vector3d deisred_position;
    // double offset = uav_id * 2.0;
    // deisred_position[0] = total_info.uav_status[OtherSubscriber::uav_1].local_position.x + offset - offset * cos_yaw;
    // deisred_position[1] = total_info.uav_status[OtherSubscriber::uav_1].local_position.y - offset * sin_yaw;
    // deisred_position[2] = total_info.uav_status[OtherSubscriber::uav_1].local_position.z;
    // // vel_deisred += 5.0 * (deisred_position - own_pos);
    // vel_deisred += 5.0 * (deisred_position - own_pos);

    // for(int i = 1; i < 4; i++)
    // {
    //     double offset_follower = (uav_id - i) * 2.0;
    //     // vel_deisred[0] += 0.5 * (offset_follower + total_info.uav_status[i].local_position.x - own_pos[0] + (i - uav_id) * 2.0 * cos_yaw);
    //     // vel_deisred[1] += 0.5 * (total_info.uav_status[i].local_position.y - own_pos[1] + (i - uav_id) * 2.0 * sin_yaw);
    //     vel_deisred[0] += 0.1 * (offset_follower + total_info.uav_status[i].local_position.x - own_pos[0] + (i - uav_id) * 2.0 * cos_yaw);
    //     vel_deisred[1] += 0.1 * (total_info.uav_status[i].local_position.y - own_pos[1] + (i - uav_id) * 2.0 * sin_yaw);
    // }


    //          1
    //      3   2   4
    if(uav_id == OtherSubscriber::uav_2) 
    {
        double offset_follower = (uav_id - OtherSubscriber::uav_3) * 2.0;
        vel_deisred[0] += 0.1 * (offset_follower + total_info.uav_status[OtherSubscriber::uav_3].local_position.x - own_pos[0] - 2.0 * sin_yaw);
        vel_deisred[1] += 0.1 * (total_info.uav_status[OtherSubscriber::uav_3].local_position.y - own_pos[1] + 2.0 * cos_yaw + 2.0 * cos_yaw);
        offset_follower = (uav_id - OtherSubscriber::uav_4) * 2.0;
        vel_deisred[0] += 0.1 * (offset_follower + total_info.uav_status[OtherSubscriber::uav_4].local_position.x - own_pos[0] + 2.0 * sin_yaw);
        vel_deisred[1] += 0.1 * (total_info.uav_status[OtherSubscriber::uav_4].local_position.y - own_pos[1] - 2.0 * cos_yaw - 2.0 * cos_yaw);

        Eigen::Vector3d deisred_position;
        deisred_position[0] = total_info.uav_status[OtherSubscriber::uav_1].local_position.x + 2.0 - 2.0 * cos_yaw;
        deisred_position[1] = total_info.uav_status[OtherSubscriber::uav_1].local_position.y - 2.0 * sin_yaw;
        deisred_position[2] = total_info.uav_status[OtherSubscriber::uav_1].local_position.z;
        vel_deisred += 5.0 * (deisred_position - own_pos);
    }
    else if(uav_id == OtherSubscriber::uav_3)
    {
        double offset_follower = (uav_id - OtherSubscriber::uav_2) * 2.0;
        vel_deisred[0] += 0.1 * (offset_follower + total_info.uav_status[OtherSubscriber::uav_2].local_position.x - own_pos[0] + 2.0 * sin_yaw);
        vel_deisred[1] += 0.1 * (total_info.uav_status[OtherSubscriber::uav_2].local_position.y - own_pos[1] - 2.0 * cos_yaw - 2.0 * cos_yaw);
        offset_follower = (uav_id - OtherSubscriber::uav_4) * 2.0;
        vel_deisred[0] += 0.1 * (offset_follower + total_info.uav_status[OtherSubscriber::uav_4].local_position.x - own_pos[0]  + 4.0 * sin_yaw);
        vel_deisred[1] += 0.1 * (total_info.uav_status[OtherSubscriber::uav_4].local_position.y - own_pos[1] - 4.0 * cos_yaw - 4.0 * cos_yaw);

        Eigen::Vector3d deisred_position;
        deisred_position[0] = total_info.uav_status[OtherSubscriber::uav_1].local_position.x + 4.0 - 2.0 * cos_yaw - 2.0 * sin_yaw;
        deisred_position[1] = total_info.uav_status[OtherSubscriber::uav_1].local_position.y - 2.0 * sin_yaw + 2.0 * cos_yaw;
        deisred_position[2] = total_info.uav_status[OtherSubscriber::uav_1].local_position.z;
        vel_deisred += 5.0 * (deisred_position - own_pos);
    }
    else
    {
        double offset_follower = (uav_id - OtherSubscriber::uav_2) * 2.0;
        vel_deisred[0] += 0.1 * (offset_follower + total_info.uav_status[OtherSubscriber::uav_2].local_position.x - own_pos[0] - 2.0 * sin_yaw);
        vel_deisred[1] += 0.1 * (total_info.uav_status[OtherSubscriber::uav_2].local_position.y - own_pos[1] + 2.0 * cos_yaw + 2.0 * cos_yaw);
        offset_follower = (uav_id - OtherSubscriber::uav_3) * 2.0;
        vel_deisred[0] += 0.1 * (offset_follower + total_info.uav_status[OtherSubscriber::uav_3].local_position.x - own_pos[0] - 4.0 * sin_yaw);
        vel_deisred[1] += 0.1 * (total_info.uav_status[OtherSubscriber::uav_3].local_position.y - own_pos[1] + 4.0 * cos_yaw + 4.0 * cos_yaw);

        Eigen::Vector3d deisred_position;
        deisred_position[0] = total_info.uav_status[OtherSubscriber::uav_1].local_position.x + 6.0 - 2.0 * cos_yaw + 2.0 * sin_yaw;
        deisred_position[1] = total_info.uav_status[OtherSubscriber::uav_1].local_position.y - 2.0 * sin_yaw - 2.0 * cos_yaw;
        deisred_position[2] = total_info.uav_status[OtherSubscriber::uav_1].local_position.z;
        vel_deisred += 5.0 * (deisred_position - own_pos);
    }
    
    if(vel_deisred[0] > 2.0)
        vel_deisred[0] = 2.0;
    if(vel_deisred[0] < -2.0)
        vel_deisred[0] = -2.0;
    if(vel_deisred[1] > 2.0)
        vel_deisred[1] = 2.0;
    if(vel_deisred[1] < -2.0)
        vel_deisred[1] = -2.0;

    return vel_deisred;
}

// void FormationPublisher::LeaderPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg)
// {
//     this->leader_position.x = _msg->pose.position.x + 2.0; // 放置偏差
//     this->leader_position.y = _msg->pose.position.y;
//     this->leader_position.z = _msg->pose.position.z;
//     tf::Quaternion quat(_msg->pose.orientation.x, _msg->pose.orientation.y, _msg->pose.orientation.z, _msg->pose.orientation.w);

//     double roll, pitch;
//     tf::Matrix3x3(quat).getRPY(roll, pitch, leader_yaw);    //四元数转欧拉角ZYX
// }

// void FormationPublisher::LeaderVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg)
// {
//     this->leader_velocity.x = _msg->twist.linear.x;
//     this->leader_velocity.y = _msg->twist.linear.y;
//     this->leader_velocity.z = _msg->twist.linear.z;
// }

void FormationPublisher::LeaderStatusCallback(const px4_application::LeaderStatus::ConstPtr& _msg)
{
    this->leader_status = *_msg;
}

void FormationPublisher::LoopTask(void)
{
    // // this->flat_target.position.x = this->leader_position.x - 2.0 * cos(leader_yaw);
    // // this->flat_target.position.y = this->leader_position.y - 2.0 * sin(leader_yaw);
    // // this->flat_target.position.z = this->leader_position.z;
    // // this->flat_target.velocity = leader_velocity;
    // // this->flat_target.acceleration = geometry_msgs::Vector3();
    // // this->flat_target.yaw = leader_yaw;

    // this->flat_target.position = this->leader_status.position;
    // this->flat_target.velocity = this->leader_status.velocity;
    // this->flat_target.acceleration = this->leader_status.acceleration;
    // this->flat_target.yaw = this->leader_status.yaw;

    // this->flat_target.header.stamp = ros::Time::now();
    // this->flat_target.type_mask = px4_application::FlatTarget::IGNORE_SNAP_JERK;
    // this->flat_target_pub.publish(this->flat_target);
    
}

void FormationPublisher::LoopTaskWithoutVirtual(void)
{
    // this->flat_target.position.x = this->leader_position.x - 2.0 * cos(leader_yaw);
    // this->flat_target.position.y = this->leader_position.y - 2.0 * sin(leader_yaw);
    // this->flat_target.position.z = this->leader_position.z;
    // this->flat_target.velocity = leader_velocity;
    // this->flat_target.acceleration = geometry_msgs::Vector3();
    // this->flat_target.yaw = leader_yaw;

    this->flat_target.position = this->leader_status.position;
    this->flat_target.velocity = this->leader_status.velocity;
    this->flat_target.acceleration = this->leader_status.acceleration;
    this->flat_target.yaw = this->leader_status.yaw;

    this->flat_target.header.stamp = ros::Time::now();
    this->flat_target.type_mask = px4_application::FlatTarget::IGNORE_SNAP_JERK;
    this->flat_target_pub.publish(this->flat_target);
}


void FormationPublisher::Initialize(void)
{
    this->flat_target_pub = this->nh.advertise<px4_application::FlatTarget>("reference/flat_setpoint", 1);
    // this->com_position_sub = this->nh.subscribe<geometry_msgs::PoseStamped>("/uav1/mavros/local_position/pose",
    //                                                                           10,
    //                                                                            &FormationPublisher::LeaderPositionCallback,
    //                                                                             this,
    //                                                                              ros::TransportHints().tcpNoDelay());
    // this->com_velocity_sub = this->nh.subscribe<geometry_msgs::TwistStamped>("/uav1/mavros/local_position/velocity_local",
    //                                                                           10,
    //                                                                            &FormationPublisher::LeaderVelocityCallback,
    //                                                                             this,
    //                                                                              ros::TransportHints().tcpNoDelay());
    this->leader_status_sub = this->nh.subscribe<px4_application::LeaderStatus>("leader_status/local",
                                                                                  10,
                                                                                   &FormationPublisher::LeaderStatusCallback,
                                                                                    this,
                                                                                     ros::TransportHints().tcpNoDelay());

}

FormationPublisher::FormationPublisher(const ros::NodeHandle& _nh, double _period) : RosBase(_nh, _period)
{
    Initialize();
}

FormationPublisher::~FormationPublisher()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "form_publisher");
    ros::NodeHandle nh;
    FormationPublisher FormationPublisher(nh, 1.0);
    // ros::spin();

    double ros_rate = 1.0;
    if(nh.getNamespace() == "/uav1")
    {
        ros::NodeHandle nh_temp("~");
        nh_temp.param<double>("leader_spin_rate/form_publisher", ros_rate, 1.0);
    }
    else
    {
        ros::NodeHandle nh_temp("~");
        nh_temp.param<double>("folower_spin_rate/form_publisher", ros_rate, 1.0);
    }

    std::cout << "-----------------------" << std::endl;
    std::cout << "ros rate: " << ros_rate << std::endl;
    ros::Rate loop(ros_rate);
    while(ros::ok())
    {
        ros::spinOnce();
        FormationPublisher.LoopTaskWithoutVirtual();
        loop.sleep();
    }
    return 0;
}

