#ifndef PX4_APPLICATION_DRONE_REALSENSE_YOLO_H_
#define PX4_APPLICATION_DRONE_REALSENSE_YOLO_H_


#include "ros_base/ros_base.h"
#include "px4_application/RealsenseTargets.h"
#include "px4_application/TargetStatus.h"

class RealsenseYoloDetection : public RosBase
{
public:
    RealsenseYoloDetection(const ros::NodeHandle& _nh, double _period);
    ~RealsenseYoloDetection();
    void LoopTaskWithoutVirtual(void);
private:
    virtual void LoopTask(void);
    void YoloDroneDetectCallback(const px4_application::RealsenseTargets::ConstPtr& _msg);

    px4_application::TargetStatus drone_status;
    ros::Subscriber yolo_drone_sub;
    ros::Publisher yolo_drone_pub;
};


#endif
