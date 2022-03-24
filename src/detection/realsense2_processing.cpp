#include "detection/realsense2_processing.h"
using namespace cv;

void RealsenseProcessing::LoopTaskWithoutVirtual(void)
{

}

void RealsenseProcessing::LoopTask(void)
{
    // ros::Time start = ros::Time::now();

    rs2::frameset frame_set = this->rs_pipe.wait_for_frames(); // 等待图像
    // rs2::frame color_frame = frame_set.get_color_frame(); // 对齐前
    // rs2::frame depth_frame = colorizer.colorize(frame_set.get_depth_frame()); // 深度渲染图

    frame_set = align_to_color.process(frame_set); // 图像对齐
    rs2::frame aligned_color_frame = frame_set.get_color_frame(); // 对齐后
    rs2::frame aligned_depth_frame = frame_set.get_depth_frame();
    // rs2::frame rendered_depth_frame = colorizer.colorize(aligned_depth_frame); // 深度渲染图

    // const int aligned_depth_w = rendered_depth_frame.as<rs2::video_frame>().get_width();
    // const int aligned_depth_h = rendered_depth_frame.as<rs2::video_frame>().get_height();
    const int aligned_color_w = aligned_color_frame.as<rs2::video_frame>().get_width();
    const int aligned_color_h = aligned_color_frame.as<rs2::video_frame>().get_height();

    // Mat aligned_depth_image(Size(aligned_depth_w, aligned_depth_h),
    //                         CV_8UC3, (void*)rendered_depth_frame.get_data(), Mat::AUTO_STEP); // 渲染深度图 opencv mat表示
    Mat aligned_color_image(Size(aligned_color_w, aligned_color_h),
                            CV_8UC3, (void*)aligned_color_frame.get_data(), Mat::AUTO_STEP);

    sensor_msgs::ImagePtr ros_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", aligned_color_image).toImageMsg();
    this->color_image_pub.publish(ros_image);

    ros::Time curr_time = ros::Time::now();
    float fps = 1.0 / (curr_time - last_time).toSec();
    last_time = ros::Time::now();

    if(this->drone_status.update)
    {
        // 测深度
        float pixel_2d[2]; // From pixel
        pixel_2d[0] = (this->drone_status.xmax + this->drone_status.xmin) / 2; // 未同步！！！yolo使用过去的图像，而该节点使用最新的
        pixel_2d[1] = (this->drone_status.ymax + this->drone_status.ymin) / 2;
        rs2::depth_frame used_depth_frame(aligned_depth_frame);
        float dist_to_pixel = used_depth_frame.get_distance(static_cast<int>(pixel_2d[0]), static_cast<int>(pixel_2d[1]));
        // std::cout << "The camera is facing an object " << dist_to_center << " meters away " << std::endl;

        // 根据深度测三维位置
        float point_3d[3]; // From point (in 3D)
        // std::cout << "fx fy: " << this->rs_intr.fx << " " << this->rs_intr.fy << std::endl;
        // std::cout << "K: " << this->rs_intr.coeffs[0] << this->rs_intr.coeffs[1] << this->rs_intr.coeffs[2] << this->rs_intr.coeffs[3] << this->rs_intr.coeffs[4] << std::endl;
        rs2_deproject_pixel_to_point(point_3d, &this->rs_intr, pixel_2d, dist_to_pixel);
        // circle(aligned_depth_image, Point(static_cast<int>(pixel_2d[0]), static_cast<int>(pixel_2d[1])), 8, Scalar(255, 255, 255), -1, 8, 0);
        circle(aligned_color_image, Point(static_cast<int>(pixel_2d[0]), static_cast<int>(pixel_2d[1])), 5, Scalar(0, 255, 0), -1, 8, 0);
        // putText(aligned_color_image, "(" + std::to_string(point[0]) + "," + std::to_string(point[1]) + "," + std::to_string(point[2]) + ")", Point(aligned_color_w / 2 + 10, aligned_color_h / 2),
        //         FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255, 0), 2, CV_AVX);
        putText(aligned_color_image, "detected:", Point(5, 20),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 1, CV_AVX);
        putText(aligned_color_image, "True", Point(130, 20),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 1, CV_AVX);
        putText(aligned_color_image, "x: " + std::to_string(point_3d[0]), Point(5, 45),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 1, CV_AVX);
        putText(aligned_color_image, "y: " + std::to_string(point_3d[1]), Point(5, 70),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 1, CV_AVX);
        putText(aligned_color_image, "z: " + std::to_string(point_3d[2]), Point(5, 95),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 1, CV_AVX);

        putText(aligned_color_image, "fps: " + std::to_string(fps), Point(5, 120),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 1, CV_AVX);

        rectangle(aligned_color_image, Point(this->drone_status.xmin, this->drone_status.ymin), 
                    Point(this->drone_status.xmax, this->drone_status.ymax), Scalar(0, 255, 0), 4);

    }
    else
    {
        putText(aligned_color_image, "detected:", Point(5, 20),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 1, CV_AVX);
        putText(aligned_color_image, "False", Point(130, 20),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 1, CV_AVX);
        putText(aligned_color_image, "x: " + std::to_string(0.0), Point(5, 45),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 1, CV_AVX);
        putText(aligned_color_image, "y: " + std::to_string(0.0), Point(5, 70),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 1, CV_AVX);
        putText(aligned_color_image, "z: " + std::to_string(0.0), Point(5, 95),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 1, CV_AVX);
        
        putText(aligned_color_image, "fps: " + std::to_string(fps), Point(5, 120),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 1, CV_AVX);
    }
    

    // imshow("aligned depth image", aligned_depth_image);
    imshow("aligned color image", aligned_color_image);
    // std::cout << "delta time: " << (ros::Time::now() - start).toSec() << std::endl;
    waitKey(5);
}

void RealsenseProcessing::YoloDroneDetectCallback(const px4_application::BoundingBoxes::ConstPtr& _msg)
{
    const px4_application::BoundingBox *drone = nullptr;
    px4_application::BoundingBox::_conf_type conf = 0;
    for(auto& item : _msg->bounding_boxes)
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
    }
    this->drone_status.update = (_msg->bounding_boxes.size() != 0);
}

RealsenseProcessing::RealsenseProcessing(const ros::NodeHandle& _nh, double _period) : RosBase(_nh, _period), align_to_color(RS2_STREAM_COLOR)
{
    this->realsense_drone_pub = this->nh.advertise<px4_application::TargetStatus>("detection_status/drone", 5);
    this->yolo_drone_sub = this->nh.subscribe<px4_application::BoundingBoxes>("yolo_detector/bounding_boxes",
                                                                                1,
                                                                                 &RealsenseProcessing::YoloDroneDetectCallback,
                                                                                  this,
                                                                                   ros::TransportHints().tcpNoDelay());
    this->rs_cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);
    this->rs_cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
    this->rs_pipe.start(this->rs_cfg);
    
    this->color_image_pub = this->nh.advertise<sensor_msgs::Image>("realsense2/color_image", 10);

    for(int i = 0; i < 20; i++) // 获取相机内参
    {
        rs2::frameset frame_set = this->rs_pipe.wait_for_frames(); // 等待图像

        frame_set = align_to_color.process(frame_set); // 图像对齐
        rs2::depth_frame aligned_depth_frame = frame_set.get_depth_frame();
        this->rs_intr = aligned_depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    }
    std::cout << "----------------------------" << std::endl;
    std::cout << "fx fy: " << this->rs_intr.fx << " " << this->rs_intr.fy << std::endl;
    std::cout << "K: " << this->rs_intr.coeffs[0] << this->rs_intr.coeffs[1] << this->rs_intr.coeffs[2] << this->rs_intr.coeffs[3] << this->rs_intr.coeffs[4] << std::endl;
    std::cout << "----------------------------" << std::endl;
}

RealsenseProcessing::~RealsenseProcessing()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "realsense_processing");
    ros::NodeHandle nh;

    RealsenseProcessing RealsenseProcessing(nh, 0.02);

    ros::spin();
}

