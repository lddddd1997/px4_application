#include "detection/drone_realsense_yolo.h"

void RealsenseYoloDetection::LoopTaskWithoutVirtual(void)
{

}

void RealsenseYoloDetection::LoopTask(void)
{

}

void RealsenseYoloDetection::YoloDroneDetectCallback(const px4_application::RealsenseTargets::ConstPtr& _msg)
{
    const px4_application::RealsenseTarget *drone = nullptr;
    px4_application::RealsenseTarget::_conf_type conf = 0;
    for(auto& item : _msg->realsense_targets)
    {
        if(item.conf > conf)
        {
            conf = item.conf;
            drone = &item;
        }
    }
    if(drone != nullptr)
    {
        this->drone_status.xmax = drone->xmax;
        this->drone_status.xmin = drone->xmin;
        this->drone_status.ymax = drone->ymax;
        this->drone_status.ymin = drone->ymin;
        if(drone->position_from_realsense.z != 0)
        {
            this->drone_status.raw_pcl_position = drone->position_from_realsense;
        }
        else
        {
            std::cout << "depth error! " << this->drone_status.raw_pcl_position << std::endl;
        }
    }
    this->drone_status.update = (_msg->realsense_targets.size() != 0);
    this->yolo_drone_pub.publish(this->drone_status);
}


RealsenseYoloDetection::RealsenseYoloDetection(const ros::NodeHandle& _nh, double _period) : RosBase(_nh, _period)
{
    this->yolo_drone_pub = this->nh.advertise<px4_application::TargetStatus>("detection_status/drone", 5);
    this->yolo_drone_sub = this->nh.subscribe<px4_application::RealsenseTargets>("yolo_detector/realsense_targets",
                                                                                1,
                                                                                 &RealsenseYoloDetection::YoloDroneDetectCallback,
                                                                                  this,
                                                                                   ros::TransportHints().tcpNoDelay());
}

RealsenseYoloDetection::~RealsenseYoloDetection()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_sub_yolo_info");
    ros::NodeHandle nh;
    
    RealsenseYoloDetection RealsenseYoloDetection(nh, 1);
    
    ros::spin();
    
    return 0;
}

