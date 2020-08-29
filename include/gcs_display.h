#ifndef PX4_APPLICATION_GCS_DISPLAY_H_
#define PX4_APPLICATION_GCS_DISPLAY_H_

#include "ros_base.h"
#include "status_subscriber.h"
#include "px4_application/UavCommand.h"

class GcsDisplay : public RosBase
{
public:
    GcsDisplay(const ros::NodeHandle& _nh, double _period);
    ~GcsDisplay();

private:
    ros::Time begin_time_;
    ros::Subscriber uav_command_sub_;

    StatusSubscriber current_info_;    //无人机与目标状态
    px4_application::UavCommand command_reception_;

    float GetTimePassSec(void);
    void UavCommandCallback(const px4_application::UavCommand::ConstPtr& _msg);

    void Initialize(void);
    void UavStateDisplay(void);
    void CommandUpdateReset(void);

    virtual void LoopTask(void);
};


#endif