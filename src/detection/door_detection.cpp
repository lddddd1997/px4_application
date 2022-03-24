#include "detection/door_detection.h"
using namespace cv;

void DoorDetection::CamImageCallback(const sensor_msgs::ImageConstPtr &_msg)
{
    this->cv_ptr = cv_bridge::toCvCopy(_msg, sensor_msgs::image_encodings::BGR8);
}

void DoorDetection::DetectContours(Mat& img_calibrated, std::vector<std::vector<Point>>& contours, std::vector<Vec4i>& hierarchy)
{

    Mat img_hsv;
    cvtColor(img_calibrated, img_hsv, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    // imshow("img_hsv", img_hsv);
    Mat img_thresholded, img_thresholded1, img_thresholded2;
    inRange(img_hsv, Scalar(this->hsv_min1[0], this->hsv_min1[1], this->hsv_min1[2]), 
                     Scalar(this->hsv_max1[0], this->hsv_max1[1], this->hsv_max1[2]), img_thresholded1); // sim
    inRange(img_hsv, Scalar(this->hsv_min2[0], this->hsv_min2[1], this->hsv_min2[2]), 
                     Scalar(this->hsv_max2[0], this->hsv_max2[1], this->hsv_max2[2]), img_thresholded2);
    // inRange(img_hsv, Scalar(0, 70, 50), Scalar(15, 255, 255), img_thresholded1); // sim
    // inRange(img_hsv, Scalar(160, 70, 50), Scalar(180, 255, 255), img_thresholded2);

    // inRange(img_hsv, Scalar(0, 100, 150), Scalar(10, 255, 255), img_thresholded1); // real
    // inRange(img_hsv, Scalar(170, 100, 150), Scalar(180, 255, 255), img_thresholded2); // 
    img_thresholded = img_thresholded1 | img_thresholded2;
    // imshow("111 | 222", img_thresholded);
    // Mat filter_img;
    // filter_img = img_thresholded;
    // medianBlur(img_thresholded, filter_img, 5);

    Mat open_image;
    // Mat element = getStructuringElement(MORPH_RECT, Size(4, 4));
    // //morphologyEx(img_thresholded, img_thresholded, MORPH_OPEN, element1);
    // morphologyEx(img_thresholded, open_image, MORPH_OPEN, element); // MORPH_CLOSE闭运算，先膨胀，后腐蚀
    // imshow("MORPH_OPEN Image", open_image);

    Mat kernel1 = getStructuringElement(MORPH_RECT,cv::Size(5,5));
    Mat kernel2 = getStructuringElement(MORPH_RECT,cv::Size(5,5));
    Mat image_erode, image_dilate;
    erode(img_thresholded, image_erode, kernel1); // 先腐蚀
    // imshow("erode",image_erode);
    dilate(image_erode, image_dilate, kernel2); // 后膨胀
    // imshow("dilate",image_dilate);
    open_image = image_dilate;

    // Mat edge_mat;
    // Canny(open_image, edge_mat, 50, 200, 3); // 边缘检测，突出的边缘像素导致得到双倍轮廓
    // imshow("edge", edge_mat);

    findContours(open_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0)); // 轮廓检测
    // findContours(open_image, contours2, hierarchy2, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(45, 45));
    if(this->debug_switch)
    {
        imshow("img_thresholded", img_thresholded);
        imshow("open_image", open_image);
        drawContours(img_calibrated, contours, -1, Scalar(255, 0, 0), 2, 8);
    }
}

void DoorDetection::FilterContours(std::vector<std::vector<Point>>& contours, std::vector<Vec4i>& hierarchy, std::vector<std::vector<Point>>& filtered_contours)
{
    std::vector<double> area_list(contours.size());
    for (int index = 0; index < contours.size(); index++)
    {
        double tmparea = fabs(contourArea(contours[index])); // 计算轮廓的面积
        area_list[index] = tmparea;
    }
    for(int i = 0; i < contours.size(); i++)
    {
        /*int child = hierarchy[i][2]; // 当存在多个子轮廓时，就会有问题
        if(child == -1)
        {
            continue;
        }
        double ratio = area_list[i] / area_list[child]; // 外内轮廓面积比
        // std::cout << i <<  "------" << child << "----- " << ratio << std::endl;
        // if(ratio >= 1.5 && ratio <= 2.0) // 筛选轮廓 real
        if(ratio >= 1.5 && ratio <= 2.5) // sim
        {
            filtered_contours.push_back(contours[i]);
            filtered_contours.push_back(contours[child]);
        }
        */

        int parent = hierarchy[i][3];
        if(parent == -1 || hierarchy[parent][3] != -1) // 只会检测最外围的门框
        {
            continue;
        }
        double ratio = area_list[parent] / area_list[i]; // 外内轮廓面积比
        if(ratio >= 1.5 && ratio <= 2.5) // sim
        {
            filtered_contours.push_back(contours[parent]);
            filtered_contours.push_back(contours[i]);
        }
    }
}


void DoorDetection::ReorderCorners(std::vector<Point>& corners)
{
    auto min_point_iter = std::min_element(corners.begin(), corners.end(),
                                            [](const Point& lhs, const Point& rhs)->bool
                                            {
                                                return (lhs.x + lhs.y) < (rhs.x + rhs.y);
                                            });
    std::vector<Point> temp_point(corners.begin(), min_point_iter);
    corners.erase(corners.begin(), min_point_iter);
    corners.insert(corners.end(), temp_point.begin(), temp_point.end());
}

void DoorDetection::DrawPoints(Mat& img, const std::vector<Point>& corners, cv::Scalar color, Point text_offset, bool show_text)
{
    for (int i = 0; i < corners.size(); i++)
    {
        circle(img, corners[i], 5, color, 2, 8, 0);
        if(show_text)
        {
            std::string txt(1, i + '0');
            putText(img, txt,
                    corners[i] + text_offset, FONT_HERSHEY_SIMPLEX, 1.0,
                    color, 1, CV_AVX);
        }
    }
}

void DoorDetection::ApproxPoly(std::vector<std::vector<Point>>& filtered_contours, std::vector<std::vector<Point>>& total_2d_points_set)
{
    for(int i = 0; i < filtered_contours.size(); i += 2)
    {
        int out_idx = i, in_idx = i + 1;
        double out_epsilon = 0.1 * arcLength(filtered_contours[out_idx], true); // 计算封闭轮廓的周长或曲线的长度
        double in_epsilon = 0.1 * arcLength(filtered_contours[in_idx], true);
        // double perimeter = out_epsilon * 10.0;
        std::vector<Point> out_corners, in_corners;
        approxPolyDP(filtered_contours[out_idx], out_corners, out_epsilon, true); // 多边形拟合曲线
        approxPolyDP(filtered_contours[in_idx], in_corners, in_epsilon, true);
        // std::cout << contour_poly[0].size() << std::endl;
        if(out_corners.size() != 4 || in_corners.size() != 4) // 非四边形
        {
            // std::cout << "contour_poly.size() != 4" << std::endl;
            continue;
        }

        // 角点重新排序
        /* 外轮廓               内轮廓
            0 -------- 3       0 -------- 1
            |          |       |          |
            |          |       |          |
            1 -------- 2       3 -------- 2
        */
        ReorderCorners(out_corners);
        ReorderCorners(in_corners);
        std::vector<Point> total_2d_points(out_corners.begin(), out_corners.end());  // 内外轮廓八个点，顺序为[外0～3，内0～3]
        total_2d_points.insert(total_2d_points.end(), in_corners.begin(), in_corners.end());
        total_2d_points_set.push_back(total_2d_points);
    }
}

void DoorDetection::CornersSubPix(cv::Mat& img_calibrated, std::vector<Point>& corners, std::vector<Point2f>& sub_corners)
{
        Mat img_grey;
        cvtColor(img_calibrated, img_grey, COLOR_BGR2GRAY);
        // 角点细化
        sub_corners.insert(sub_corners.begin(), corners.begin(), corners.end());
        cornerSubPix(img_grey, sub_corners,
                         Size(5, 5),
                         Size(-1, -1),
                         TermCriteria(TermCriteria::MAX_ITER | TermCriteria::EPS,
                                      40,
                                      0.1));
}

Vec3f DoorDetection::RotationMatrix2EulerAngles(Mat& R)
{
    float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +  R.at<double>(1, 0) * R.at<double>(1, 0) );
 
    bool singular = sy < 1e-6; // 出现奇异值
 
    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2, 1) , R.at<double>(2, 2));
        y = atan2(-R.at<double>(2, 0), sy);
        z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    }
    else
    {
        x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }
    // return Vec3f(x, y, z) * 180 / 3.14159265358979;
    return Vec3f(x, y, z);
}

bool DoorDetection::PositionAttitudeEstimation()
{
    try
    {
        Mat img_original;
        Mat img_calibrated;
        do
        {
            if(this->use_video_cap)
            {
                video_cap >> img_original;
            }
            else
            {
                // img_original = this->cv_ptr->image.clone(); // 深拷贝
                img_original = this->cv_ptr->image; // 浅拷贝
            }
            // imshow("img_original", img_original);
            // ros::Time start = ros::Time::now();
            // undistort(img_original, img_calibrated, this->camera_matrix, this->dist_coeffs); // 图像矫正，速度慢
            remap(img_original, img_calibrated, map1, map2, INTER_CUBIC);
            // std::cout << (ros::Time::now() - start).toSec() << std::endl;
            // imshow("img_calibrated", img_calibrated);

            std::vector<std::vector<Point>> contours; // 检测到的所有轮廓
            std::vector<Vec4i> hierarchy; // hierarchy层次结构[后，前，第一个子轮廓，父]
            DetectContours(img_calibrated, contours, hierarchy);
            
            std::vector<std::vector<Point>> filtered_contours; // 满足条件的外内轮廓，偶为外轮廓，奇为内轮廓
            FilterContours(contours, hierarchy, filtered_contours); // 过滤轮廓
            if(filtered_contours.empty())
            {
                if(this->lost_timeout < timeout_threshold)
                    this->lost_timeout++;
                // std::cout << "FilterContours" << std::endl;
                continue;
            }

            std::vector<std::vector<Point>> total_2d_points_set;
            ApproxPoly(filtered_contours, total_2d_points_set); // 多边形拟合
            if(total_2d_points_set.empty()) // 非四边形
            {
                if(this->lost_timeout < timeout_threshold)
                    this->lost_timeout++;
                // std::cout << "ApproxPoly" << std::endl;
                continue;
            }

            // DrawPoints(img_calibrated, std::vector<Point>(total_2d_points_set[0].begin(),
            //                                                total_2d_points_set[0].end()), Scalar(255, 0, 0), Point(0, 0), false);
            // std::vector<Point2f> sub_corners;
            // CornersSubPix(img_calibrated, total_2d_points_set[0], sub_corners); // 角点细化
            // DrawPoints(img_calibrated, std::vector<Point>(sub_corners.begin(),
            //                                                sub_corners.end()), Scalar(255, 0, 0), Point(0, 0), false);
            
            // 位姿估计
            int closest_idx = -1; // 最靠近的目标
            std::vector<px4_application::TargetStatus> total_door_status(total_2d_points_set.size());
            for(int i = 0; i < total_2d_points_set.size(); i++)
            {
                drawContours(img_calibrated, std::vector<std::vector<Point>>(1, std::vector<Point>(total_2d_points_set[i].begin(), total_2d_points_set[i].begin() + 4)),
                             -1, Scalar(255, 0, 0), 2, 8); // 画出外轮廓图像
                drawContours(img_calibrated, std::vector<std::vector<Point>>(1, std::vector<Point>(total_2d_points_set[i].begin() + 4, total_2d_points_set[i].begin() + 8)),
                             -1, Scalar(255, 255, 0), 2, 8); // 画出内轮廓图像
                
                Mat rvecs, tvecs; // 相机坐标系（x右，y下，z前）下的门框位姿
                solvePnP(this->total_3d_points, std::vector<Point2f>(total_2d_points_set[i].begin(), total_2d_points_set[i].end()),
                         this->camera_matrix, this->dist_coeffs, rvecs, tvecs, false, SOLVEPNP_ITERATIVE); // 求解PnP

                aruco::drawAxis(img_calibrated, this->camera_matrix, this->dist_coeffs,
                            rvecs, tvecs, 0.3); // 画出坐标轴
                Mat rotation;
                Rodrigues(rvecs, rotation); // 罗德里格斯（Rodrigues）变换
                Vec3f angles = RotationMatrix2EulerAngles(rotation);
                // std::cout << "---------------------" << std::endl;
                // std::cout << "rotation: " << rvecs << std::endl;
                // std::cout << "translation: " << tvecs << std::endl;
                // std::cout << "rotation matrix: " << rotation << std::endl;
                // std::cout << "euler angles: " << angles << std::endl;
                total_door_status[i].xmin = total_2d_points_set[i][0].x;
                total_door_status[i].ymin = total_2d_points_set[i][0].y;
                total_door_status[i].xmax = total_2d_points_set[i][2].x;
                total_door_status[i].ymax = total_2d_points_set[i][2].y;
                total_door_status[i].raw_pcl_position.x = tvecs.at<double>(0, 0); // m
                total_door_status[i].raw_pcl_position.y = tvecs.at<double>(1, 0);
                total_door_status[i].raw_pcl_position.z = tvecs.at<double>(2, 0);
                total_door_status[i].yaw = angles[1];
                // total_door_status[i].yaw = tan(1.2 * angles[1]); // 因为角度越大，越不准，加上tan函数映射

                if(closest_idx == -1 || total_door_status[closest_idx].raw_pcl_position.z > total_door_status[i].raw_pcl_position.z)
                {
                    closest_idx = i;
                }
            }
            DrawPoints(img_calibrated, std::vector<Point>(total_2d_points_set[closest_idx].begin(),
                                                           total_2d_points_set[closest_idx].begin() + 4), Scalar(255, 0, 0), Point(0, 0), false);
            DrawPoints(img_calibrated, std::vector<Point>(total_2d_points_set[closest_idx].begin() + 4,
                                                           total_2d_points_set[closest_idx].begin() + 8) , Scalar(255, 255, 0), Point(0, 0), false);

            this->door_status.xmin = total_2d_points_set[closest_idx][0].x;
            this->door_status.ymin = total_2d_points_set[closest_idx][0].y;
            this->door_status.xmax = total_2d_points_set[closest_idx][2].x;
            this->door_status.ymax = total_2d_points_set[closest_idx][2].y;
            this->door_status.raw_pcl_position.x = total_door_status[closest_idx].raw_pcl_position.x;
            this->door_status.raw_pcl_position.y = total_door_status[closest_idx].raw_pcl_position.y;
            this->door_status.raw_pcl_position.z = total_door_status[closest_idx].raw_pcl_position.z;
            this->door_status.yaw = this->FilterYaw.run(total_door_status[closest_idx].yaw);

            this->lost_timeout /= 2;

            // std::cout << "\033[2J" << std::endl; // 清屏
            // static ros::Time last_time;
            // ros::Time current_time = ros::Time::now();
            // float current_time_sec = current_time.sec - last_time.sec;
            // float current_time_nsec = current_time.nsec / 1e9 - last_time.nsec / 1e9;
            // std::cout << "fps: " << (int)(1 / (current_time_sec + current_time_nsec)) << std::endl;
            // // std::cout << current_time_sec + current_time_nsec << " s" << std::endl;
            // last_time = ros::Time::now();

            // std::cout << "target number: " << total_2d_points_set.size() << std::endl;
            // std::cout << "closest target x: " << this->door_status.raw_pcl_position.x << std::endl;

            // std::cout << "               y: " << this->door_status.raw_pcl_position.y << std::endl;
            // std::cout << "               z: " << this->door_status.raw_pcl_position.z << std::endl;
            // std::cout << "             yaw: " << this->door_status.yaw * 57.2957795 << std::endl;
            
        }while(0);
        static ros::Time last_time = ros::Time::now();
        float fps = 1.0f / (ros::Time::now() - last_time).toSec();
        if(this->lost_timeout >= this->timeout_threshold)
        {
            putText(img_calibrated, "detected:", Point(5, 20),
                    FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2, CV_AVX);
            putText(img_calibrated, "False", Point(130, 20),
                    FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2, CV_AVX);
            putText(img_calibrated, "x: " + std::to_string(0.0), Point(5, 45),
                    FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2, CV_AVX);
            putText(img_calibrated, "y: " + std::to_string(0.0), Point(5, 70),
                    FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2, CV_AVX);
            putText(img_calibrated, "z: " + std::to_string(0.0), Point(5, 95),
                    FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2, CV_AVX);
            putText(img_calibrated, "yaw: " + std::to_string(0.0), Point(5, 120),
                    FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2, CV_AVX);
            putText(img_calibrated, "fps: " + std::to_string(fps), Point(5, 145),
                    FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2, CV_AVX);
        }
        else
        {
            putText(img_calibrated, "detected:", Point(5, 20),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 2, CV_AVX);
            putText(img_calibrated, "True", Point(130, 20),
                    FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 2, CV_AVX);
            putText(img_calibrated, "x: " + std::to_string(this->door_status.raw_pcl_position.x), Point(5, 45),
                    FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 2, CV_AVX);
            putText(img_calibrated, "y: " + std::to_string(this->door_status.raw_pcl_position.y), Point(5, 70),
                    FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 2, CV_AVX);
            putText(img_calibrated, "z: " + std::to_string(this->door_status.raw_pcl_position.z), Point(5, 95),
                    FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 2, CV_AVX);
            putText(img_calibrated, "yaw: " + std::to_string(this->door_status.yaw * 57.3), Point(5, 120),
                    FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 2, CV_AVX);
            putText(img_calibrated, "fps: " + std::to_string(fps), Point(5, 145),
                    FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 2, CV_AVX);
        }
        last_time = ros::Time::now();
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_calibrated).toImageMsg();
        msg->header.stamp = ros::Time::now();
        this->detection_image_pub.publish(*msg);
        // this->video_writer.write(img_calibrated);
        // use [rosrun image_view video_recorder image:=/uav1/detection_status/image] to save video
        // use [rosrun image_view image_view image:=/uav1/detection_status/image] to display image
        if(this->debug_switch)
        {
            imshow("door detection", img_calibrated); //show the original image
            waitKey(1);
        }
        // When everything done, release the video capture object
        // cap.release();
        // Closes all the frames
        // destroyAllWindows();
    }catch(std::exception e){
        std::cout << "getcenterImagePoint() error" << e.what() << std::endl;
    }
    return true;
}

void DoorDetection::LoopTask()
{
    // if(this->cv_ptr == nullptr)
    // {
    //     std::cout << "image empty!" << std::endl;
    //     return ;
    // }
    // GetCenterPoint();
    // GetDistanceOfSquare();
}

void DoorDetection::LoopTaskWithoutVirtual()
{
    if(!this->use_video_cap && this->cv_ptr == nullptr)
    {
        std::cout << "image empty!" << std::endl;
        return ;
    }
    PositionAttitudeEstimation();
    // GetCenterPoint();
    // GetDistanceOfSquare();
    if(this->lost_timeout >= this->timeout_threshold)
    {
        this->door_status.update = false;
    }
    else
    {
        this->door_status.update = true;
    }
    this->door_status.header.frame_id = this->nh.getNamespace() + "_camera";
    this->door_status.header.stamp = ros::Time().now();
    this->door_detection_pub.publish(this->door_status);
}

DoorDetection::DoorDetection(const ros::NodeHandle& _nh, double _period) : RosBase(_nh, _period), use_video_cap(false), lost_timeout(0), FilterYaw(50, 20)
{
    ros::NodeHandle nh("~");
    nh.param<bool>("use_video_cap", this->use_video_cap, false);
    nh.param<std::string>("topic_input", this->topic_input, "cgo3_camera/image_raw");
    nh.param<int>("timeout_threshold", this->timeout_threshold, 10);
    nh.param<float>("out_length", this->out_length, 1.5);
    nh.param<float>("out_width", this->out_width, 0.9);
    nh.param<float>("in_length", this->in_length, 1.2);
    nh.param<float>("in_width", this->in_width, 0.6);

    nh.param<int>("hsv_range1/hmin", this->hsv_min1[0], 0);
    nh.param<int>("hsv_range1/smin", this->hsv_min1[1], 0);
    nh.param<int>("hsv_range1/vmin", this->hsv_min1[2], 0);
    nh.param<int>("hsv_range1/hmax", this->hsv_max1[0], 0);
    nh.param<int>("hsv_range1/smax", this->hsv_max1[1], 0);
    nh.param<int>("hsv_range1/vmax", this->hsv_max1[2], 0);

    nh.param<int>("hsv_range2/hmin", this->hsv_min2[0], 0);
    nh.param<int>("hsv_range2/smin", this->hsv_min2[1], 0);
    nh.param<int>("hsv_range2/vmin", this->hsv_min2[2], 0);
    nh.param<int>("hsv_range2/hmax", this->hsv_max2[0], 0);
    nh.param<int>("hsv_range2/smax", this->hsv_max2[1], 0);
    nh.param<int>("hsv_range2/vmax", this->hsv_max2[2], 0);

    std::cout << "----------------hsv range----------------" << std::endl;
    std::cout << "hsv_min1: " << this->hsv_min1[0] << " " << this->hsv_min1[1] << " " << this->hsv_min1[2] << std::endl;
    std::cout << "hsv_max1: " << this->hsv_max1[0] << " " << this->hsv_max1[1] << " " << this->hsv_max1[2] << std::endl;
    std::cout << "hsv_min2: " << this->hsv_min2[0] << " " << this->hsv_min2[1] << " " << this->hsv_min2[2] << std::endl;
    std::cout << "hsv_max2: " << this->hsv_max2[0] << " " << this->hsv_max2[1] << " " << this->hsv_max2[2] << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
    nh.param<bool>("debug_switch", this->debug_switch, false);

    nh.param<bool>("use_video_writer", this->use_video_writer, false);
    if(this->use_video_writer)
    {
        time_t t = time(NULL);
        char ch[64] = {0};
        strftime(ch, sizeof(ch) - 1, "%Y-%m-%d %H:%M:%S", localtime(&t));     //年-月-日 时-分-秒
        std::string format_time(ch);
        std::string save_path;
        nh.param<std::string>("video_save_path", save_path, "/home/amov/");

        this->video_writer.open(save_path + format_time + "door.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 20.0, cv::Size(640, 480),true);
        if(!this->video_writer.isOpened())
        {
            std::cerr << "fail to initializer the video writer" << std::endl;
            exit(-1);
        }
    }

    if(this->use_video_cap)
    {
        int video_input = 0;
        nh.param<int>("video_input", video_input, 0);
        this->video_cap.open(video_input);
        video_cap.set(CAP_PROP_FRAME_HEIGHT, 480);
	    video_cap.set(CAP_PROP_FRAME_WIDTH, 640);
        if (!video_cap.isOpened())
        {
		    std::cout << "-----------------------unable to open video " << video_input << std::endl;
            exit(1);
        }
    }
    else
    {
        this->cam_image_sub = this->nh.subscribe<sensor_msgs::Image>(this->topic_input,
                                                                      1,
                                                                       &DoorDetection::CamImageCallback,
                                                                        this,
                                                                         ros::TransportHints().tcpNoDelay());
    }
    
    this->door_detection_pub = this->nh.advertise<px4_application::TargetStatus>("detection_status/door", 5);
    this->detection_image_pub = this->nh.advertise<sensor_msgs::Image>("detection_status/image", 5);
    std::string param_path;
    nh.param<std::string>("param_path", param_path, "/home/p520c/Documents/px4_ws/src/px4_application/src/detection/ost.yaml");
    FileStorage fs(param_path, FileStorage::READ);
    if(!fs.isOpened())
    {
        std::cout << "-----------------------unable to open file storage!" << std::endl;
        exit(1);
    }
    fs["camera_matrix"] >> this->camera_matrix;
    fs["distortion_coefficients"] >> this->dist_coeffs;
    std::cout << "----------------camera matrix----------------" << std::endl;
    std::cout << this->camera_matrix << std::endl;
    std::cout << this->dist_coeffs << std::endl;
    std::cout << "---------------------------------------------" << std::endl;
    /*
        在门框目标上建立坐标系
        x右，y下，z前
                    外轮廓              内轮廓
                0 -------- 3       0 -------- 1
                |          |       |          |
                |          |       |          |
                1 -------- 2       3 -------- 2
    */
    this->out_3d_points.push_back(Point3f(-this->out_length / 2.0f, -this->out_width / 2.0f, 0));
    this->out_3d_points.push_back(Point3f(-this->out_length / 2.0f, this->out_width / 2.0f, 0));
    this->out_3d_points.push_back(Point3f(this->out_length / 2.0f, this->out_width / 2.0f, 0));
    this->out_3d_points.push_back(Point3f(this->out_length / 2.0f, -this->out_width / 2.0f, 0));

    this->in_3d_points.push_back(Point3f(-this->in_length / 2.0f, -this->in_width / 2.0f, 0));
    this->in_3d_points.push_back(Point3f(this->in_length / 2.0f, -this->in_width / 2.0f, 0));
    this->in_3d_points.push_back(Point3f(this->in_length / 2.0f, this->in_width / 2.0f, 0));
    this->in_3d_points.push_back(Point3f(-this->in_length / 2.0f, this->in_width / 2.0f, 0));

    total_3d_points.insert(total_3d_points.end(), this->out_3d_points.begin(), this->out_3d_points.end());
    total_3d_points.insert(total_3d_points.end(), this->in_3d_points.begin(), this->in_3d_points.end());

    initUndistortRectifyMap(this->camera_matrix, this->dist_coeffs, Mat(),
                getOptimalNewCameraMatrix(this->camera_matrix, this->dist_coeffs,
                                           Size(640, 480), 1, Size(640, 480), 0), // 缩放比例为1
                                            Size(640, 480), CV_16SC2, this->map1, this->map2); // 生成输出映射map
    fs.release();
}

DoorDetection::~DoorDetection()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "door_detection");
    ros::NodeHandle nh;
    
    DoorDetection DoorDetection(nh, 1.0);
    // ros::spin();

    double ros_rate = 1.0;
    if(nh.getNamespace() == "/uav1")
    {
        ros::NodeHandle nh_temp("~");
        nh_temp.param<double>("leader_spin_rate/door_detection", ros_rate, 1.0);
    }
    else
    {
        ros::NodeHandle nh_temp("~");
        nh_temp.param<double>("folower_spin_rate/door_detection", ros_rate, 1.0);
    }

    std::cout << "-----------------------" << std::endl;
    std::cout << "ros rate: " << ros_rate << std::endl;
    ros::Rate loop(ros_rate);
    while(ros::ok())
    {
        ros::spinOnce();
        DoorDetection.LoopTaskWithoutVirtual();
        loop.sleep();
    }
    
    return 0;
}
