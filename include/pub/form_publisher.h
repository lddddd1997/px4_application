#ifndef PX4_APPLICATION_FORM_PUBLISHER_H_
#define PX4_APPLICATION_FORM_PUBLISHER_H_

#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Eigen>
#include <mavros_msgs/State.h>
#include "ros_base/ros_base.h"
#include "px4_application/BoundaryConditions.h"
#include "px4_application/UavStatus.h"
#include "px4_application/FlatTarget.h"
#include "px4_application/LeaderStatus.h"
#include "large_scale_traj_optimizer/traj_min_jerk.hpp"
#include "trajectory_utilities/trajectory_utils.hpp"
#include "gcs/gcs_display.h"

class FormationPublisher : public RosBase
{
public:
    FormationPublisher(const ros::NodeHandle& _nh, double _period);
    ~FormationPublisher();
    void LoopTaskWithoutVirtual(void);
private:
    ros::Publisher flat_target_pub;
    ros::Subscriber leader_status_sub;
    // ros::Subscriber com_position_sub; // 通信获取的无人机信息
    // ros::Subscriber com_velocity_sub;
    // void LeaderPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg); // 通信获取
    // void LeaderVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg);

    void LeaderStatusCallback(const px4_application::LeaderStatus::ConstPtr& _msg); // 检测获取

    
    // geometry_msgs::Vector3 leader_position; // 通信获取
    // geometry_msgs::Vector3 leader_velocity;
    // double leader_yaw;

    px4_application::LeaderStatus leader_status; // 检测获取

    px4_application::FlatTarget flat_target;

    void Initialize(void);

    virtual void LoopTask(void);

};

#endif
