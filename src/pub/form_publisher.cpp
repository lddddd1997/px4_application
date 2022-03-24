#include "pub/form_publisher.h"

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

