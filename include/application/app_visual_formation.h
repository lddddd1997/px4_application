#ifndef PX4_APPLICATION_APP_VISUAL_FORMATION_H_
#define PX4_APPLICATION_APP_VISUAL_FORMATION_H_

#include <chrono>
#include <geometry_msgs/PoseArray.h>
#include "ros_base/ros_base.h"
#include "subscriber/status_subscriber.h"
#include "gcs/gcs_display.h"
#include "px4_application/UavCommand.h"
#include "px4_application/BoundaryConditions.h"
#include "px4_application/FlatTarget.h"
#include "px4_application/LeaderStatus.h"
#include "utilities/pid_controller.h"
#include "utilities/func_utils.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "minimum_snap_genration/trajectory_generator_waypoint.hpp"
#include "large_scale_traj_optimizer/traj_min_snap.hpp"
#include "large_scale_traj_optimizer/traj_min_jerk.hpp"
#include "trajectory_utilities/trajectory_utils.hpp"

class States
{
public:
    States();
    virtual ~States();
    void StateMachineSchedule(const StatusSubscriber& _current_info,
                               const ros::Publisher& _uav_command_pub,
                                px4_application::UavCommand* _command_deliver,
                                 States** _State);    //注：使用指针的指针，确保能访问到对象指针本身，因为状态转移需delete与new操作
private:
    virtual void Run(const StatusSubscriber& _current_info,
                      const ros::Publisher& _uav_command_pub,
                       px4_application::UavCommand* _command_deliver,
                        States** _State) = 0;
protected:
    geometry_msgs::Vector3 reach_point_range;
};

class Prepare : public States
{
public:
    Prepare();
    ~Prepare();
private:
    virtual void Run(const StatusSubscriber& _current_info,
                      const ros::Publisher& _uav_command_pub,
                       px4_application::UavCommand* _command_deliver,
                        States** _State);
};

class TakeOff : public States
{
public:
    TakeOff();
    ~TakeOff();
private:
    virtual void Run(const StatusSubscriber& _current_info,
                      const ros::Publisher& _uav_command_pub,
                       px4_application::UavCommand* _command_deliver,
                        States** _State);

    geometry_msgs::Vector3 takeoff_position;
    geometry_msgs::Vector3 takeoff_absolute_position_param;    //绝对起飞位置
    double takeoff_relative_height_param;    //相对起飞高度
    bool takeoff_id;    //false绝对起飞 true相对起飞
};

class Mission : public States
{
public:
    Mission();
    ~Mission();
private:
    virtual void Run(const StatusSubscriber& _current_info,
                      const ros::Publisher& _uav_command_pub,
                       px4_application::UavCommand* _command_deliver,
                        States** _State);
    int own_id;
    OtherSubscriber total_info;    //所有无人机信息
    ros::NodeHandle nh_private;
    geometry_msgs::Vector3 desired_command;

    // RvizVisualization<nav_msgs::Path> min_jerk_traj_visualization;
    // RvizVisualization<nav_msgs::Path> min_snap_traj_visualization;
    RvizVisualization<geometry_msgs::PoseArray> waypoints_visualization;
    RvizVisualization<geometry_msgs::Vector3> acc_visualization;
    
    double vel, acc;
    Eigen::MatrixXd route;
    Eigen::MatrixXd ref_door_pose;
    int door_index = 1;

    min_jerk::JerkOpt jerk_opt;
    min_jerk::Trajectory min_jerk_traj;

    // min_snap::SnapOpt snap_opt;
    // min_snap::Trajectory min_snap_traj;

    double traj_duration = 0.0;

    int cycles = 0;
    ros::Publisher ref_traj_pub;
    ros::Subscriber flat_target_sub;
    px4_application::FlatTarget flat_target;
    void FlatTargetCallback(const px4_application::FlatTarget::ConstPtr& _msg);




    ros::Publisher leader_status_pub;
    px4_application::LeaderStatus leader_status;
};

class ReturnHome : public States
{
public:
    ReturnHome();
    ~ReturnHome();
private:
    virtual void Run(const StatusSubscriber& _current_info,
                      const ros::Publisher& _uav_command_pub,
                       px4_application::UavCommand* _command_deliver,
                        States** _State);
    geometry_msgs::Vector3 home_position;
};

class Landing : public States
{
public:
    Landing();
    ~Landing();
private:
    virtual void Run(const StatusSubscriber& _current_info,
                      const ros::Publisher& _uav_command_pub,
                       px4_application::UavCommand* _command_deliver,
                        States** _State);
    geometry_msgs::Vector3 landing_pos_vel;
    
};

class Finished : public States
{
public:
    Finished();
    ~Finished();
private:
    virtual void Run(const StatusSubscriber& _current_info,
                      const ros::Publisher& _uav_command_pub,
                       px4_application::UavCommand* _command_deliver,
                        States** _State);
};

class UavMission : public RosBase 
{
public:
    UavMission(const ros::NodeHandle& _nh, double _period);
    ~UavMission();
    void LoopTaskWithoutVirtual(void);
private:

    ros::Publisher uav_command_pub;

    px4_application::UavCommand command_deliver;
    StatusSubscriber current_info;    //本无人机和目标状态

    States* UavState;

    void Initialize(void);

    virtual void LoopTask(void);
};

#endif