#ifndef PX4_APPLICATION_GCS_SETTING_H_
#define PX4_APPLICATION_GCS_SETTING_H_

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "ros_base.h"

class GcsSetting : public RosBase
{
public:
    GcsSetting(const ros::NodeHandle& _nh, double _period);
    ~GcsSetting();
    void ModeSelect(void);
private:
    ros::Subscriber uav_state_sub;
    ros::ServiceClient uav_set_mode_client;
    ros::ServiceClient uav_arming_client;

    mavros_msgs::State current_state_uav;
    std::string control_mode;
    bool armed_cmd;
    mavros_msgs::SetMode uav_mode_cmd;
    mavros_msgs::CommandBool uav_arm_cmd;

    void StateCallback(const mavros_msgs::State::ConstPtr& _msg);
    void Initialize(void);

    virtual void LoopTask(void);
};


#endif



