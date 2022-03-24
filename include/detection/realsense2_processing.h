#ifndef PX4_APPLICATION_REALSENSE2_PROCESSING_H_
#define PX4_APPLICATION_REALSENSE2_PROCESSING_H_
#include <opencv2/core.hpp>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include "ros_base/ros_base.h"
#include "px4_application/BoundingBoxes.h"
#include "px4_application/TargetStatus.h"
#include "utilities/filters_lp.h"

class RealsenseProcessing : public RosBase
{
public:
    RealsenseProcessing(const ros::NodeHandle& _nh, double _period);
    ~RealsenseProcessing();
    void LoopTaskWithoutVirtual(void);
private:
    virtual void LoopTask(void);
    void YoloDroneDetectCallback(const px4_application::BoundingBoxes::ConstPtr& _msg);

    ros::Time last_time; // 用来计算fps

    rs2::config rs_cfg;
    rs2::pipeline rs_pipe;
    rs2::colorizer colorizer;
    rs2_intrinsics rs_intr; // 相机参数
    // 图像对齐
    rs2::align align_to_color; // RS2_STREAM_COLOR对齐到彩色图，RS2_STREAM_DEPTH对齐到深度图
    ros::Publisher color_image_pub;

    px4_application::TargetStatus drone_status;
    ros::Publisher realsense_drone_pub;
    ros::Subscriber yolo_drone_sub;
};



#endif
