#ifndef PX4_APPLICATION_UAV_CONTROL_H_
#define PX4_APPLICATION_UAV_CONTROL_H_

#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include "ros_base.h"
#include "px4_application/UavCommand.h"
#include "pid_controller.h"

class UavControl : public RosBase
{
public:
    UavControl(const ros::NodeHandle& _nh, double _period);
    ~UavControl();

private:
    ros::Publisher setpoint_raw_local_pub;
    ros::Subscriber uav_command_sub;
    ros::Subscriber uav_local_position_sub;

    mavros_msgs::PositionTarget command_target_uav;
    px4_application::UavCommand command_reception;
    geometry_msgs::Vector3 local_position_uav;

    PidController PositionX;
    PidController PositionY;
    PidController PositionZ;

    void CommandCallback(const px4_application::UavCommand::ConstPtr& _msg);
    void PositionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg);

    void Initialize(void);
    void CommandUpdateReset(void);
    void CommandExecution(void);

    virtual void LoopTask(void);

};

#endif