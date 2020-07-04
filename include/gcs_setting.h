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
    // ros::NodeHandle nh_;
    // ros::Timer loop_timer_;
    //double loop_period_;
    ros::Subscriber uav_state_sub_;
    ros::ServiceClient uav_set_mode_client_;
    ros::ServiceClient uav_arming_client_;

    mavros_msgs::State current_state_uav_;
    std::string control_mode_;
    bool armed_cmd_;
    mavros_msgs::SetMode uav_mode_cmd_;
    mavros_msgs::CommandBool uav_arm_cmd_;

    // void LoopTimerCallback(const ros::TimerEvent& event);
    void UavStateCallback(const mavros_msgs::State::ConstPtr& _msg);
    void Initialize(void);

    virtual void LoopTask(void);
};


#endif



