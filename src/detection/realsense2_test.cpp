#include "detection/realsense2_test.h"
using namespace cv;

void DroneDetection::LoopTaskWithoutVirtual(void)
{

}

void DroneDetection::LoopTask(void)
{

}

void DroneDetection::DepthCamCallback(const sensor_msgs::PointCloud2::ConstPtr& _msg)
{
    // _msg->data 序列化后的数据，直接获得不了信息，序列化是为了方便信息传输和交换，使用时需要反序列化
    // _msg->fields 每个点的成员变量，其中name为对应成员变量的名字，datatype为变量的数据类型
    // _msg->header 包含的时间戳和坐标系
    // _msg->height 点云的高度，如果是无序点云，则为1
    // _msg->is_bigendian
    // _msg->is_dense 是否有非法数据点，true表示没有
    // _msg->point_step 每个点占用的比特数，1个字节对应8个比特数
    // _msg->row_step 每一行占用的比特数
    // _msg->width 每行点云的宽度
    // sensor_msgs::PointCloud out_point_cloud;
    // sensor_msgs::convertPointCloud2ToPointCloud(*_msg, out_point_cloud);
    static ros::Time start_time;
    std::cout << "duration " << (ros::Time::now() - start_time).toSec() << std::endl;
    start_time = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*_msg, pcl_cloud);

    this->pix_center_x = (this->drone_status.xmax + this->drone_status.xmin) / 2;
    this->pix_center_y = (this->drone_status.ymax + this->drone_status.ymin) / 2;

    this->drone_status.raw_pcl_position.x = (this->drone_status.xmax + this->drone_status.xmin) / 2;
    this->drone_status.raw_pcl_position.y = (this->drone_status.ymax + this->drone_status.ymin) / 2;
    // double perimeter = (this->drone_status.xmax - this->drone_status.xmin) * 2 + 
    //                     (this->drone_status.ymax - this->drone_status.ymin) * 2;
            
    // this->drone_status.raw_pcl_position.z = (3.0 * 277.191356) / perimeter;
    // if(!isnan(pcl_cloud.at(pix_center_x, pix_center_y).z))
    this->drone_status.raw_pcl_position.z = pcl_cloud.at(500, 500).z;
    std::cout << (ros::Time::now() - start_time).toSec() << "------- " << this->drone_status.raw_pcl_position.z << std::endl;

    

    // camera_position_frame Head Z+  Right X+  Down Y+
    // this->drone_status.raw_pcl_position.x = pcl_cloud.at(pix_center_x, pix_center_y).x;
    // this->drone_status.raw_pcl_position.y = pcl_cloud.at(pix_center_x, pix_center_y).y;
    // this->drone_status.raw_pcl_position.z = pcl_cloud.at(pix_center_x, pix_center_y).z;

    // if(!(isnan(this->drone_status.raw_pcl_position.x)
    //     || isnan(this->drone_status.raw_pcl_position.y)
    //         || isnan(this->drone_status.raw_pcl_position.z)))
    // {
    //     this->drone_status.filter_pcl_position.x = FilterCamX.run(this->drone_status.raw_pcl_position.x);
    //     this->drone_status.filter_pcl_position.y = FilterCamY.run(this->drone_status.raw_pcl_position.y);
    //     this->drone_status.filter_pcl_position.z = FilterCamZ.run(this->drone_status.raw_pcl_position.z);
    // }

    this->drone_status.update = true;
    this->yolo_drone_pub.publish(this->drone_status);
    // std::cout << "x y z " << this->pcl_position.x << " " << this->pcl_position.y << " " << this->pcl_position.z << std::endl;
    // std::cout << "center point:   " << pcl_cloud.at(this->pix_center.x, this->pix_center.y).z << std::endl;
    // std::cout << "320*240:   " << pcl_cloud.at(320, 240).z << std::endl;
}

DroneDetection::DroneDetection(const ros::NodeHandle& _nh, double _period) : RosBase(_nh, _period), FilterCamX(100, 20), FilterCamY(100, 20), FilterCamZ(100, 20)
{
    this->yolo_drone_pub = this->nh.advertise<px4_application::TargetStatus>("detection_status/drone", 5);
    this->depth_cam_sub = this->nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", // /camera/depth_registered/points
                                                                        1,
                                                                         &DroneDetection::DepthCamCallback,
                                                                          this,
                                                                           ros::TransportHints().tcpNoDelay());
}

DroneDetection::~DroneDetection()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "realsense2_test");
    ros::NodeHandle nh;

    // DroneDetection DroneDetection(nh, 0.01);
    // ros::spin();
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    rs2::pipeline pipe;
    rs2::pipeline_profile profile = pipe.start(cfg); 
    // 图像对齐
    rs2::align align_to_depth(RS2_STREAM_DEPTH); // 对齐到深度图
    rs2::align align_to_color(RS2_STREAM_COLOR); // 对齐到彩色图
    rs2::colorizer colorizer;
    ros::Publisher color_image_pub = nh.advertise<sensor_msgs::Image>("realsense2/color_image", 10);
    ros::Rate loop(20);
    while(ros::ok())
    {
        ros::Time start = ros::Time::now();

        rs2::frameset frame_set = pipe.wait_for_frames();


        rs2::frame color_frame = frame_set.get_color_frame(); // 对齐前
        rs2::frame depth_frame = colorizer.colorize(frame_set.get_depth_frame()); // 深度渲染图

        frame_set = align_to_color.process(frame_set); // 图像对齐
        rs2::frame aligned_color_frame = frame_set.get_color_frame(); // 对齐后
        rs2::frame aligned_depth_frame = frame_set.get_depth_frame();
        rs2::frame rendered_depth_frame = colorizer.colorize(aligned_depth_frame); // 深度渲染图

        
        const int depth_w = depth_frame.as<rs2::video_frame>().get_width();
        const int depth_h = depth_frame.as<rs2::video_frame>().get_height();
        const int color_w = color_frame.as<rs2::video_frame>().get_width();
        const int color_h = color_frame.as<rs2::video_frame>().get_height();
        Mat color_image(Size(color_w, color_h),
                                CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP); // 彩色图 opencv mat表示
        Mat depth_image(Size(depth_w, depth_h),
                                    CV_8UC3, (void*)depth_frame.get_data(), Mat::AUTO_STEP); // 渲染深度图 opencv mat表示

        
        const int aligned_depth_w = rendered_depth_frame.as<rs2::video_frame>().get_width();
        const int aligned_depth_h = rendered_depth_frame.as<rs2::video_frame>().get_height();
        const int aligned_color_w = aligned_color_frame.as<rs2::video_frame>().get_width();
        const int aligned_color_h = aligned_color_frame.as<rs2::video_frame>().get_height();
        Mat aligned_depth_image(Size(aligned_depth_w, aligned_depth_h),
                                CV_8UC3, (void*)rendered_depth_frame.get_data(), Mat::AUTO_STEP); // 渲染深度图 opencv mat表示
        Mat aligned_color_image(Size(aligned_color_w, aligned_color_h),
                                CV_8UC3, (void*)aligned_color_frame.get_data(), Mat::AUTO_STEP);
        // Mat flip_image;
        // circle(aligned_color_image, Point(50, 100), 8, Scalar(0, 255, 255), -1, 8, 0);
        // flip(aligned_color_image, flip_image, -1);
        // circle(flip_image, Point(640 - 1 - 50, 480 - 1 -100), 8, Scalar(255, 0, 255), -1, 8, 0);
        
        // 发布图像
        sensor_msgs::ImagePtr ros_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", aligned_color_image).toImageMsg();
        color_image_pub.publish(ros_image);

        // 测深度
        float pixel[2]; // From pixel
        pixel[0] = aligned_depth_w / 2;
        pixel[1] = aligned_depth_h / 2;
        // std::cout << "delta time: " << (ros::Time::now() - start).toSec() << std::endl;
        rs2::depth_frame used_depth_frame(aligned_depth_frame);
        float dist_to_center = used_depth_frame.get_distance(static_cast<int>(pixel[0]), static_cast<int>(pixel[1]));
        // std::cout << "The camera is facing an object " << dist_to_center << " meters away " << std::endl;

        // 根据深度测三维位置
        float point[3]; // From point (in 3D)
        rs2_intrinsics intr = used_depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
        // std::cout << "fx fy: " << intr.fx << " " << intr.fy << std::endl;
        // std::cout << "K: " << intr.coeffs[0] << intr.coeffs[1] << intr.coeffs[2] << intr.coeffs[3] << intr.coeffs[4] << std::endl;
        rs2_deproject_pixel_to_point(point, &intr, pixel, dist_to_center);
        // std::cout << "point[0]: " << point[0] << std::endl;
        // std::cout << "point[1]: " << point[1] << std::endl;
        // std::cout << "point[2]: " << point[2] << std::endl;

        circle(aligned_depth_image, Point(aligned_depth_w / 2, aligned_depth_h / 2), 8, Scalar(255, 255, 255), -1, 8, 0);
        circle(aligned_color_image, Point(aligned_color_w / 2, aligned_color_h / 2), 8, Scalar(255, 255, 255), -1, 8, 0);
        // putText(aligned_color_image, "(" + std::to_string(point[0]) + "," + std::to_string(point[1]) + "," + std::to_string(point[2]) + ")", Point(aligned_color_w / 2 + 10, aligned_color_h / 2),
        //         FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255, 0), 2, CV_AVX);
        putText(aligned_color_image, "x: " + std::to_string(point[0]), Point(5, 30),
                FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255, 0), 2, CV_AVX);
        putText(aligned_color_image, "y: " + std::to_string(point[1]), Point(5, 60),
                FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255, 0), 2, CV_AVX);
        putText(aligned_color_image, "z: " + std::to_string(point[2]), Point(5, 90),
                FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255, 0), 2, CV_AVX);
        // imshow("original color image", color_image);
        // imshow("original depth image", depth_image);
        imshow("aligned depth image", aligned_depth_image);
        imshow("aligned color image", aligned_color_image);
        waitKey(10);
        ros::spinOnce();
        // DroneDetection.LoopTaskWithoutVirtual();
        loop.sleep();

    }
    
    return 0;
}
