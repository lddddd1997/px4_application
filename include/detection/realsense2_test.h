#ifndef PX4_APPLICATION_REALSENSE2_TEST_H_
#define PX4_APPLICATION_REALSENSE2_TEST_H_

#include <opencv2/core.hpp>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/geometry.h>
#include "ros_base/ros_base.h"
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
