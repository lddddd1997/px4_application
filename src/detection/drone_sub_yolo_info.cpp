#include "detection/drone_sub_yolo_info.h"

void DroneDetection::LoopTaskWithoutVirtual(void)
{

}

void DroneDetection::LoopTask(void)
{

}

void DroneDetection::YoloDroneDetectCallback(const px4_application::BoundingBoxes::ConstPtr& _msg)
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
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*_msg, pcl_cloud);

    this->pix_center_x = (this->drone_status.xmax + this->drone_status.xmin) / 2;
    this->pix_center_y = (this->drone_status.ymax + this->drone_status.ymin) / 2;

    // this->drone_status.raw_pcl_position.x = (this->drone_status.xmax + this->drone_status.xmin) / 2;
    // this->drone_status.raw_pcl_position.y = (this->drone_status.ymax + this->drone_status.ymin) / 2;
    if(!isnan(pcl_cloud.at(pix_center_x, pix_center_y).z))
    {
        this->drone_status.raw_pcl_position.x = pcl_cloud.at(pix_center_x, pix_center_y).x;
        this->drone_status.raw_pcl_position.y = pcl_cloud.at(pix_center_x, pix_center_y).y;
        this->drone_status.raw_pcl_position.z = pcl_cloud.at(pix_center_x, pix_center_y).z;
    }


    // camera_position_frame Head Z+  Right X+  Down Y+
    // this->drone_status.raw_pcl_position.x = pcl_cloud.at(pix_center_x, pix_center_y).x;
    // this->drone_status.raw_pcl_position.y = pcl_cloud.at(pix_center_x, pix_center_y).y;
    // this->drone_status.raw_pcl_position.z = pcl_cloud.at(pix_center_x, pix_center_y).z;

    if(!(isnan(this->drone_status.raw_pcl_position.x)
        || isnan(this->drone_status.raw_pcl_position.y)
            || isnan(this->drone_status.raw_pcl_position.z)))
    {
        this->drone_status.filter_pcl_position.x = FilterCamX.run(this->drone_status.raw_pcl_position.x);
        this->drone_status.filter_pcl_position.y = FilterCamY.run(this->drone_status.raw_pcl_position.y);
        this->drone_status.filter_pcl_position.z = FilterCamZ.run(this->drone_status.raw_pcl_position.z);
    }

    this->yolo_drone_pub.publish(this->drone_status);
    // std::cout << "x y z " << this->pcl_position.x << " " << this->pcl_position.y << " " << this->pcl_position.z << std::endl;
    // std::cout << "center point:   " << pcl_cloud.at(this->pix_center.x, this->pix_center.y).z << std::endl;
    // std::cout << "320*240:   " << pcl_cloud.at(320, 240).z << std::endl;
}

DroneDetection::DroneDetection(const ros::NodeHandle& _nh, double _period) : RosBase(_nh, _period), FilterCamX(100, 20), FilterCamY(100, 20), FilterCamZ(100, 20)
{
    this->yolo_drone_pub = this->nh.advertise<px4_application::TargetStatus>("detection_status/drone", 5);
    this->yolo_drone_sub = this->nh.subscribe<px4_application::BoundingBoxes>("yolo_detector/bounding_boxes",
                                                                                1,
                                                                                 &DroneDetection::YoloDroneDetectCallback,
                                                                                  this,
                                                                                   ros::TransportHints().tcpNoDelay());
    this->depth_cam_sub = this->nh.subscribe<sensor_msgs::PointCloud2>("/typhoon_h480_2/depth_camera/depth/points",
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
    ros::init(argc, argv, "drone_sub_yolo_info");
    ros::NodeHandle nh;
    
    DroneDetection DroneDetection(nh, 0.01);
    
    ros::spin();

    // ros::Rate loop(20);
    // while(ros::ok())
    // {
    //     ros::spinOnce();
    //     DroneDetection.LoopTaskWithoutVirtual();
    //     loop.sleep();
    // }
    
    return 0;
}
