/** 
* @file     uav_estimator.cpp
* @brief    无人机估计器
* @details  
* @note
* @author   lddddd
*           Email: lddddd1997@gmail.com
*           Github: https://github.com/lddddd1997
* @date     2021.1.06
* @version  1.0
* @par      Edit history:
*           1.0: lddddd, 2021.1.06, 添加了无人机在gazebo环境中的真值估计.
*/
#include "uav/uav_estimator.h"

void UavEstimator::GroundTruthCallback(const nav_msgs::Odometry::ConstPtr& _msg)
{
    this->ground_truth.pose.pose.position.x = _msg->pose.pose.position.x;
    this->ground_truth.pose.pose.position.y = _msg->pose.pose.position.y;
    this->ground_truth.pose.pose.position.z = _msg->pose.pose.position.z;

    this->ground_truth.pose.pose.orientation.w = _msg->pose.pose.orientation.w;
    this->ground_truth.pose.pose.orientation.x = _msg->pose.pose.orientation.x;
    this->ground_truth.pose.pose.orientation.y = _msg->pose.pose.orientation.y;
    this->ground_truth.pose.pose.orientation.z = _msg->pose.pose.orientation.z;
}

void UavEstimator::LoopTask(void)
{
    // switch(this->source_input)
    // {
    //     case UavEstimator::truth:
    //     {
    //         vision_uav.pose.orientation.w = this->ground_truth.pose.pose.orientation.w;
    //         vision_uav.pose.orientation.x = this->ground_truth.pose.pose.orientation.x;
    //         vision_uav.pose.orientation.y = this->ground_truth.pose.pose.orientation.y;
    //         vision_uav.pose.orientation.z = this->ground_truth.pose.pose.orientation.z;

    //         vision_uav.pose.position.x = this->ground_truth.pose.pose.position.x;
    //         vision_uav.pose.position.y = this->ground_truth.pose.pose.position.y;
    //         vision_uav.pose.position.z = this->ground_truth.pose.pose.position.z;

    //         break;
    //     }

    //     default: return;
    // }
    // vision_uav.header.stamp = ros::Time::now();
    // vision_pub.publish(vision_uav);
}

void UavEstimator::LoopTaskWithoutVirtual(void)
{
    switch(this->source_input)
    {
        case UavEstimator::truth:
        {
            vision_uav.pose.orientation.w = this->ground_truth.pose.pose.orientation.w;
            vision_uav.pose.orientation.x = this->ground_truth.pose.pose.orientation.x;
            vision_uav.pose.orientation.y = this->ground_truth.pose.pose.orientation.y;
            vision_uav.pose.orientation.z = this->ground_truth.pose.pose.orientation.z;

            vision_uav.pose.position.x = this->ground_truth.pose.pose.position.x;
            vision_uav.pose.position.y = this->ground_truth.pose.pose.position.y;
            vision_uav.pose.position.z = this->ground_truth.pose.pose.position.z;

            break;
        }

        default: return;
    }
    vision_uav.header.stamp = ros::Time::now();
    vision_pub.publish(vision_uav);
}

void UavEstimator::Initialize(void)
{
    ros::NodeHandle nh("~");
    nh.param<int>("uav_id", this->own_id, 0);
    nh.param<int>("source_input", this->source_input, UavEstimator::truth);
    this->vision_pub = this->nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);
    
    this->ground_truth_sub = this->nh.subscribe<nav_msgs::Odometry>("ground_truth/state",
                                                                     10,
                                                                      &UavEstimator::GroundTruthCallback,
                                                                       this,
                                                                        ros::TransportHints().tcpNoDelay());
}

UavEstimator::UavEstimator(const ros::NodeHandle& _nh, double _period) : RosBase(_nh, _period)
{
    Initialize();
    std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>uav_" << this->own_id << " estimator<<<<<<<<<<<<<<<<<<<<<<<<<<<<< " << std::endl;
    std::cout << "source input: ";
    switch(this->source_input)
    {
        case UavEstimator::truth: std::cout << "ground truth" << std::endl; break;
        
        default: std::cout << "unknow" << std::endl; break;
    }
    std::cout << "rate: " << (int)(1 / _period) << "Hz" << std::endl;
    std::cout << "Please set EKF2_AID_MASK parameter in the QGC" << std::endl;
    std::cout << "Estimator starts..." << std::endl;
}

UavEstimator::~UavEstimator()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_estimator");
    ros::NodeHandle nh;
    UavEstimator UavEstimator(nh, 1.0);
    // ros::spin();

    ros::Rate loop(20);
    while(ros::ok())
    {
        ros::spinOnce();
        UavEstimator.LoopTaskWithoutVirtual();
        loop.sleep();
    }
    return 0;
}