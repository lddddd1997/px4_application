#ifndef PX4_APPLICATION_DOOR_DETECTION_H_
#define PX4_APPLICATION_DOOR_DETECTION_H_

#include <opencv2/core.hpp>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <time.h>
#include "ros_base/ros_base.h"
#include "px4_application/TargetStatus.h"
#include "utilities/filters_lp.h"

class DoorDetection : public RosBase
{
public:
    DoorDetection(const ros::NodeHandle& _nh, double _period);
    ~DoorDetection();
    void LoopTaskWithoutVirtual(void);
private:
    virtual void LoopTask(void);
    void CamImageCallback(const sensor_msgs::ImageConstPtr& _msg);

    cv::Vec3f RotationMatrix2EulerAngles(cv::Mat& R);
    bool PositionAttitudeEstimation();
    void DetectContours(cv::Mat& img, std::vector<std::vector<cv::Point>>& contours, std::vector<cv::Vec4i>& hierarchy);
    void FilterContours(std::vector<std::vector<cv::Point>>& contours, std::vector<cv::Vec4i>& hierarchy, std::vector<std::vector<cv::Point>>& filtered_contours);
    void DrawPoints(cv::Mat& img, const std::vector<cv::Point>& corners, cv::Scalar color, cv::Point text_offset, bool show_text);
    void ReorderCorners(std::vector<cv::Point>& corners);
    void ApproxPoly(std::vector<std::vector<cv::Point>>& filtered_contours, std::vector<std::vector<cv::Point>>& total_2d_points_set);
    void CornersSubPix(cv::Mat& img_calibrated, std::vector<cv::Point>& corners, std::vector<cv::Point2f>& sub_corners);

    bool use_video_cap;
    cv::VideoCapture video_cap;
    bool use_video_writer;
    cv::VideoWriter video_writer;
    int lost_timeout;
    int timeout_threshold;
    cv::Mat camera_matrix, dist_coeffs;
    cv::Mat map1, map2; // initUndistortRectifyMap输出映射，用于校正图像
    std::vector<cv::Point3f> out_3d_points;
    std::vector<cv::Point3f> in_3d_points;
    std::vector<cv::Point3f> total_3d_points;
    float out_length, out_width;
    float in_length, in_width;
    std::string topic_input;
    ros::Subscriber cam_image_sub;
    ros::Publisher door_detection_pub;
    ros::Publisher detection_image_pub;
    px4_application::TargetStatus door_status;
    cv_bridge::CvImagePtr cv_ptr;
    Butterworth2 FilterYaw;
    int hsv_min1[3], hsv_max1[3], hsv_min2[3], hsv_max2[3];
    bool debug_switch;
};

#endif
