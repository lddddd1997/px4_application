#ifndef PX4_APPLICATION_UAV_CONTROL_H_
#define PX4_APPLICATION_UAV_CONTROL_H_

#include "ros_base.h"
#include "px4_application/UavCommand.h"
#include "pid_controller.h"
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>

class UavControl : public RosBase
{
public:
    UavControl(const ros::NodeHandle& _nh, double _period);
    ~UavControl();

private:
    // ros::NodeHandle nh_;
    // ros::Timer loop_timer_;
    // double loop_period_;
    ros::Publisher setpoint_raw_local_pub_;
    ros::Subscriber uav_command_sub_;
    ros::Subscriber uav_local_position_sub_;

    mavros_msgs::PositionTarget command_target_uav_;
    px4_application::UavCommand command_reception_;
    geometry_msgs::Vector3 local_position_uav_;

    PidController PoseX;
    PidController PoseY;
    PidController PoseZ;

    // void LoopTimerCallback(const ros::TimerEvent& event);
    void UavCommandCallback(const px4_application::UavCommand::ConstPtr& _msg);
    void UavPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg);

    void Initialize(void);
    void CommandUpdateReset(void);
    void CommandExecution(void);

    virtual void LoopTask(void);

};

#endif