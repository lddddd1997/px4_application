#ifndef PX4_APPLICATION_TRAJ_PUBLISHER_H_
#define PX4_APPLICATION_TRAJ_PUBLISHER_H_

#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Eigen>
#include <mavros_msgs/State.h>
#include "ros_base/ros_base.h"
#include "px4_application/BoundaryConditions.h"
#include "px4_application/FlatTarget.h"
#include "large_scale_traj_optimizer/traj_min_jerk.hpp"
#include "trajectory_utilities/trajectory_utils.hpp"
#include "gcs/gcs_display.h"

class TrajectoryPublisher : public RosBase
{
public:
    TrajectoryPublisher(const ros::NodeHandle& _nh, double _period);
    ~TrajectoryPublisher();
    void LoopTaskWithoutVirtual(void);
private:
    // ros::Subscriber state_sub;
    ros::Publisher flat_target_pub;
    ros::Subscriber ref_traj_sub;
    // ros::Subscriber door_detection_sub;

    px4_application::FlatTarget flat_target;
    // bool door_update = false;
    // double door_enu_yaw;
    // tf::TransformListener tf_listener;

    RvizVisualization<nav_msgs::Path> min_jerk_traj_visualization;

    ros::Time start_time;
    ros::Time curr_time;
    min_jerk::Trajectory min_jerk_traj;
    double total_duration = 0.0;
    double forward_sample_time = 0.1;
    // std::string curr_mode;
    // uint8_t armed;

    // void DoorStatusCallback(const px4_application::TargetStatus::ConstPtr& _msg);
    // void StateCallback(const mavros_msgs::State::ConstPtr& _msg);
    void BoundaryCondCallback(const px4_application::BoundaryConditions::ConstPtr& _msg);

    void Initialize(void);

    virtual void LoopTask(void);

};

#endif
