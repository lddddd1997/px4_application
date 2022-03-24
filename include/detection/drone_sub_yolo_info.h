#ifndef PX4_APPLICATION_DRONE_SUB_YOLO_INFO_H_
#define PX4_APPLICATION_DRONE_SUB_YOLO_INFO_H_

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/geometry.h>
#include "ros_base/ros_base.h"
#include "px4_application/BoundingBoxes.h"
#include "px4_application/TargetStatus.h"
#include "utilities/filters_lp.h"

class DroneDetection : public RosBase
{
public:
    DroneDetection(const ros::NodeHandle& _nh, double _period);
    ~DroneDetection();
    void LoopTaskWithoutVirtual(void);
private:
    virtual void LoopTask(void);
    void YoloDroneDetectCallback(const px4_application::BoundingBoxes::ConstPtr& _msg);
    void DepthCamCallback(const sensor_msgs::PointCloud2::ConstPtr& _msg);
    
    int pix_center_x;
    int pix_center_y;
    px4_application::TargetStatus drone_status;
    ros::Subscriber yolo_drone_sub;
    ros::Subscriber depth_cam_sub;
    ros::Publisher yolo_drone_pub;
    Butterworth2 FilterCamX;
    Butterworth2 FilterCamY;
    Butterworth2 FilterCamZ;
};

#endif
