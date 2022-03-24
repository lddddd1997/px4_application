#ifndef PX4_APPLICATION_UAV_CONTROL_H_
#define PX4_APPLICATION_UAV_CONTROL_H_

#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Eigen>
#include <mavros_msgs/State.h>
#include "ros_base/ros_base.h"
#include "px4_application/UavCommand.h"
#include "px4_application/BoundaryConditions.h"
#include "px4_application/FlatTarget.h"
#include "large_scale_traj_optimizer/traj_min_jerk.hpp"
#include "trajectory_utilities/trajectory_utils.hpp"
#include "utilities/pid_controller.h"
#include "gcs/gcs_display.h"

class UavControl : public RosBase
{
public:
    UavControl(const ros::NodeHandle& _nh, double _period);
    ~UavControl();
    void LoopTaskWithoutVirtual(void);
private:
    ros::Publisher setpoint_raw_local_pub;
    ros::Subscriber state_sub;
    ros::Subscriber uav_command_sub;
    ros::Subscriber uav_local_position_sub;
    ros::Subscriber uav_local_velocity_sub;
    ros::Subscriber flat_target_sub;

    std::string curr_mode;
    mavros_msgs::PositionTarget command_target_uav;
    px4_application::UavCommand command_reception;
    Eigen::Vector3d local_position;
    Eigen::Vector3d local_velocity;
    px4_application::FlatTarget flat_target;

    PidController PositionX;
    PidController PositionY;
    PidController PositionZ;
    int own_id;

    bool use_position_control;
    Eigen::Vector3d kp;
    Eigen::Vector3d kv;
    Eigen::Vector3d ki;
    Eigen::Vector3d integral;
    Eigen::Vector3d integral_max;
    double max_fb_acc;
    Eigen::Vector3d gravity;
    Eigen::Vector3d ComputeDesiredAcc(const Eigen::Vector3d& target_pos, const Eigen::Vector3d& target_vel, const Eigen::Vector3d& target_acc, double dt);

    void StateCallback(const mavros_msgs::State::ConstPtr& _msg);
    void CommandCallback(const px4_application::UavCommand::ConstPtr& _msg);
    void PositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg);
    void VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _msg);
    void FlatTargetCallback(const px4_application::FlatTarget::ConstPtr& _msg);

    void Initialize(void);
    void CommandUpdateReset(void);
    void NormalCommandFixed(void);
    void TrajerctoryCommandFixed(void);
    void CommandExecution(void);

    virtual void LoopTask(void);
};

#endif