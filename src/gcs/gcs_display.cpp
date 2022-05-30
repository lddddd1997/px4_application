/** 
* @file     gcs_display.cpp
* @brief    无人机状态显示
* @details  
* @note
* @author   lddddd
*           Email: lddddd1997@gmail.com
*           Github: https://github.com/lddddd1997
* @date     2021.1.06
* @version  2.1
* @par      Edit history:
*           1.0: lddddd, 2020.5.24, .
*           2.0: lddddd, 2020.7.21, 更新节点句柄与topic的命名空间.
*           2.1: lddddd, 2021.1.06, 添加终端界面的无人机id显示.
*/

#include "gcs/gcs_display.h"

void GcsDisplay::CommandCallback(const px4_application::UavCommand::ConstPtr& _msg)
{
    this->command_reception = *_msg;
}

float GcsDisplay::GetTimePassSec(void)
{
    ros::Time current_time = ros::Time::now();
    float current_time_sec = current_time.sec - this->begin_time.sec;
    float current_time_nsec = current_time.nsec / 1e9 - this->begin_time.nsec / 1e9;
    return (current_time_sec + current_time_nsec);
}

void GcsDisplay::CommandUpdateReset(void)
{
    this->command_reception.update = false;
}

void GcsDisplay::LoopTask(void)
{
    // UavStateDisplay();
    // RvizTrajectoryDisplay();
    // RvizPoseDisplay();
    // cout << "Virtual Loop Task of Derived Class !" << endl;
}

void GcsDisplay::LoopTaskWithoutVirtual(void)
{
    UavStateDisplay();
    if(this->debug_pub)
    {
        RvizTrajectoryDisplay();
        RvizPoseDisplay();
    }
    // cout << "Virtual Loop Task of Derived Class !" << endl;
}

void GcsDisplay::Initialize(void)
{
    ros::NodeHandle nh("~");
    nh.param<int>("uav_id", this->own_id, 0);
    nh.param<bool>("debug_pub", this->debug_pub, false);
    c_id = this->own_id + '0';
    this->begin_time = ros::Time::now();
    this->current_info.uav_status.state.mode = "UNKNOWN";
    this->command_reception.task_name = "UNKNOWN";
    this->uav_command_sub = this->nh.subscribe<px4_application::UavCommand>("px4_application/uav_command",
                                                                             10,
                                                                              &GcsDisplay::CommandCallback,
                                                                               this,
                                                                                ros::TransportHints().tcpNoDelay());
    if(this->debug_pub)
    {
        this->rviz_trajectory_pub = this->nh.advertise<nav_msgs::Path>("rviz/trajectory/feedback", 10);
        this->rviz_pose_pub = this->nh.advertise<geometry_msgs::PoseStamped>("rviz/pose/feedback", 10);

        this->rviz_desired_trajectory_pub = this->nh.advertise<nav_msgs::Path>("rviz/trajectory/desired", 10);
        this->rviz_desired_pose_pub = this->nh.advertise<geometry_msgs::PoseStamped>("rviz/pose/desired", 10);
    }
}

void GcsDisplay::RvizTrajectoryDisplay(void)
{
    if(this->rviz_trajectory.poses.size() >= MAX_TRAJECTORY_BUF_SIZE)
        this->rviz_trajectory.poses.erase(this->rviz_trajectory.poses.begin()); // 删除首个插入的元素

    geometry_msgs::PoseStamped pose_stamp;
    pose_stamp.pose = this->current_info.uav_status.quat_pos;
    this->rviz_trajectory.poses.push_back(pose_stamp);
    this->rviz_trajectory.header.stamp = ros::Time::now();
    this->rviz_trajectory.header.frame_id = "world";
    this->rviz_trajectory_pub.publish(this->rviz_trajectory);


    if(this->rviz_desired_trajectory.poses.size() >= MAX_TRAJECTORY_BUF_SIZE)
        this->rviz_desired_trajectory.poses.erase(this->rviz_desired_trajectory.poses.begin()); // 删除首个插入的元素

    this->rviz_desired_trajectory.poses.push_back(pose_stamp);
    this->rviz_desired_trajectory.header.stamp = ros::Time::now();
    this->rviz_desired_trajectory.header.frame_id = "world";
    this->rviz_desired_trajectory_pub.publish(this->rviz_desired_trajectory);

}

void GcsDisplay::RvizPoseDisplay(void)
{
    this->rviz_desired_pose.header.frame_id = this->nh.getNamespace() + "_camera"; // 转化前
    this->rviz_desired_pose.header.stamp = ros::Time::now();
    this->rviz_desired_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, this->current_info.door_status.yaw, 0.0);
    this->rviz_desired_pose.pose.position.x = this->current_info.door_status.raw_pcl_position.x;
    this->rviz_desired_pose.pose.position.y = this->current_info.door_status.raw_pcl_position.y;
    this->rviz_desired_pose.pose.position.z = this->current_info.door_status.raw_pcl_position.z;
    this->rviz_desired_pose_pub.publish(this->rviz_desired_pose);

    this->rviz_pose.header.frame_id = this->current_info.tf_door_status.header.frame_id; // 转化后
    this->rviz_pose.header.stamp = ros::Time::now();
    this->rviz_pose.pose.orientation = this->current_info.tf_door_status.pose.orientation;
    this->rviz_pose.pose.position = this->current_info.tf_door_status.pose.position;
    this->rviz_pose_pub.publish(this->rviz_pose);
}

void GcsDisplay::UavStateDisplay(void)
{
    /*固定的浮点显示*/
    std::cout.setf(std::ios::fixed);
    /*setprecision(n) 设显示小数精度为n位*/
    std::cout << std::setprecision(2);
    /*左对齐*/
    std::cout.setf(std::ios::right);
    /*强制显示小数点*/
    std::cout.setf(std::ios::showpoint);
    /*强制显示符号*/
    std::cout.setf(std::ios::showpos);

    std::cout << "----------------------------------[Uav " << c_id << "]------------------------------------" << std::endl;
    std::cout << "---------------------------------State Info----------------------------------" << std::endl;
    std::cout << "Time:" << std::setw(8) << GetTimePassSec() << " [s] ";

    /*是否和飞控建立起连接*/
    std::cout << (this->current_info.uav_status.state.connected ? " [ Connected ] " : " [ Unconnected ] ");
    /*是否上锁*/
    std::cout << (this->current_info.uav_status.state.armed ? " [ Armed ] " : " [ DisArmed ] ");
    std::cout << " [ " << this->current_info.uav_status.state.mode <<" ] ";
    std::string flight_state;
    switch(this->current_info.uav_status.extended_state.landed_state)
    {
        case mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED: flight_state = "UNDEFINED"; break;
        case mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND: flight_state = "ON_GROUND"; break;
        case mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR: flight_state = "IN_FLIGHT"; break;
        case mavros_msgs::ExtendedState::LANDED_STATE_TAKEOFF: flight_state = "TAKEOFF"; break;
        case mavros_msgs::ExtendedState::LANDED_STATE_LANDING: flight_state = "LANDING"; break;
        default : flight_state = "UNDEFINED"; break;
    }

    std::cout << " [ " << flight_state << " ] " << std::endl;

    std::cout << "Estimated Status:  ";
    std::cout << "Attitude   " << (this->current_info.uav_status.estimator_status.attitude_status_flag ? "[√]   " : "[X]   ")
               << "Vel Horiz Rel " << (this->current_info.uav_status.estimator_status.velocity_horiz_status_flag ? "[√]   " : "[X]   ") 
                << "Vel Verti Rel " << (this->current_info.uav_status.estimator_status.velocity_vert_status_flag ? "[√]   " : "[X]   ") << std::endl;


    std::cout << "                   ";
    std::cout << "Accel      " << (this->current_info.uav_status.estimator_status.accel_error_status_flag ? "[X]   " : "[√]   ")
               << "Pos Verti Rel " << (this->current_info.uav_status.estimator_status.pos_vert_agl_status_flag ? "[√]   " : "[X]   ") 
                << "Pos Verti Abs " << (this->current_info.uav_status.estimator_status.pos_vert_abs_status_flag ? "[√]   " : "[X]   ") << std::endl;

    std::cout << "                   ";
    std::cout << "Gps Glitch " << (this->current_info.uav_status.estimator_status.gps_glitch_status_flag ? "[√]   " : "[X]   ")
               << "Pos Horiz Rel " << (this->current_info.uav_status.estimator_status.pos_horiz_rel_status_flag ? "[√]   " : "[X]   ") 
                << "Pos Horiz Abs " << (this->current_info.uav_status.estimator_status.pos_horiz_abs_status_flag ? "[√]   " : "[X]   ") << std::endl;

    std::cout << "                   ";
    std::cout << "Const Mode " << (this->current_info.uav_status.estimator_status.const_pos_mode_status_flag ? "[√]   " : "[X]   ")
               << "pre Horiz Rel " << (this->current_info.uav_status.estimator_status.pred_pos_horiz_rel_status_flag ? "[√]   " : "[X]   ") 
                << "pre Horiz Abs " << (this->current_info.uav_status.estimator_status.pred_pos_horiz_abs_status_flag ? "[√]   " : "[X]   ") << std::endl;

    const double RAD2DEG = 57.295779513082320876846364344191;
    std::string frame_name;
    if(this->command_reception.frame_id == px4_application::UavCommand::LOCAL)
    {
        frame_name = " [LOCAL]";
    }
    else if(this->command_reception.frame_id == px4_application::UavCommand::BODY)
    {
        frame_name = "  [BODY]";
    }
    else
    {
        frame_name = " [ERROR]";
    }
    int setw_num = 9;
    std::cout << "--------------------------------Attitude Info--------------------------------" << std::endl;
    std::cout << "Attitude Angle  [R P Y] : " 
               << std::setw(setw_num) << this->current_info.uav_status.attitude_angle.x * RAD2DEG << " [ ° ] " 
                << std::setw(setw_num) << this->current_info.uav_status.attitude_angle.y * RAD2DEG << " [ ° ] "
                 << std::setw(setw_num) << this->current_info.uav_status.attitude_angle.z * RAD2DEG << " [ ° ] " << std::endl;
    std::cout << "Attitude Rate   [R P Y] : "
               << std::setw(setw_num) << this->current_info.uav_status.attitude_rate.x  * RAD2DEG << " [°/s] "
                << std::setw(setw_num) << this->current_info.uav_status.attitude_rate.y  * RAD2DEG << " [°/s] "
                 << std::setw(setw_num) << this->current_info.uav_status.attitude_rate.z  * RAD2DEG << " [°/s] " << std::endl;
    std::cout << "-------------------------------Navigation Info-------------------------------" << std::endl;
    std::cout << "LOCAL Position  [X Y Z] : "
               << std::setw(setw_num) << this->current_info.uav_status.local_position.x << " [ m ] "
                << std::setw(setw_num) << this->current_info.uav_status.local_position.y << " [ m ] "
                 << std::setw(setw_num) << this->current_info.uav_status.local_position.z << " [ m ] " << std::endl;
    std::cout << "LOCAL Velocity  [X Y Z] : "
               << std::setw(setw_num) << this->current_info.uav_status.local_velocity.x << " [m/s] "
                << std::setw(setw_num) << this->current_info.uav_status.local_velocity.y << " [m/s] "
                 << std::setw(setw_num) << this->current_info.uav_status.local_velocity.z << " [m/s] " << std::endl;
    std::cout << "BODY  Velocity  [X Y Z] : "    //指机体坐标映射到的参考系的水平方向（即参考系到机体系，只旋转了Z轴）
               << std::setw(setw_num) << this->current_info.uav_status.body_heading_velocity.x << " [m/s] "
                << std::setw(setw_num) << this->current_info.uav_status.body_heading_velocity.y << " [m/s] "
                 << std::setw(setw_num) << this->current_info.uav_status.body_heading_velocity.z << " [m/s] " << std::endl;
    std::cout << "-------------------------------Command" << frame_name << "-------------------------------" << std::endl;
    std::cout << "Period: " << this->command_reception.period  << " [s] " << " [ " << this->command_reception.task_name << " ] ";

    std::cout << (this->command_reception.update ? " [ ACTIVE ] " : "  [ DEAD ]  ");
    switch(this->command_reception.xyz_id)
    {
        case px4_application::UavCommand::PX_PY_PZ:
        {
            std::cout << " [ PX_PY_PZ ] ";
            break;
        }
        case px4_application::UavCommand::VX_VY_VZ:
        {
            std::cout << " [ VX_VY_VZ ] ";
            break;
        }
        case px4_application::UavCommand::VX_VY_PZ:
        {
            std::cout << " [ VX_VY_PZ ] ";
            break;
        }
        case px4_application::UavCommand::VX_PY_VZ:
        {
            std::cout << " [ VX_PY_VZ ] ";
            break;
        }
        case px4_application::UavCommand::PX_VY_VZ:
        {
            std::cout << " [ PX_VY_VZ ] ";
            break;
        }
        case px4_application::UavCommand::PX_PY_VZ:
        {
            std::cout << " [ PX_PY_VZ ] ";
            break;
        }
        case px4_application::UavCommand::UX_UY_UZ:
        {
            std::cout << " [ UX_UY_UZ ] ";
            break;
        }
        default:     
        {
            std::cout << " [ ERROR ] ";
            break;
        }
    }
    switch(this->command_reception.yaw_id)
    {
        case px4_application::UavCommand::NO_YAW:
        {
            std::cout << " [ NO_YAW ] ";
            break;
        }
        case px4_application::UavCommand::YAW:
        {
            std::cout << " [ YAW ] ";
            break;
        }
        case px4_application::UavCommand::YAW_RATE:
        {
            std::cout << " [ YAW_RATE ] ";
            break;
        }
        default:     
        {
            std::cout << " [ ERROR ] ";
            break;
        }
    }
    std::cout << std::endl;
    std::cout << "Command Reception : "
               << std::setw(setw_num) << this->command_reception.x << " [X]"
                << std::setw(setw_num) << this->command_reception.y << " [Y]"
                 << std::setw(setw_num) << this->command_reception.z << " [Z]"
                  << std::setw(setw_num) << this->command_reception.yaw << " [Yaw]" << std::endl;
    std::cout << "--------------------------------Detection Info-------------------------------" << std::endl;
    std::cout << "Pixel Center : "
               << std::setw(setw_num) << (this->current_info.drone_status.xmax + this->current_info.drone_status.xmin) / 2 << " [X]"
                << std::setw(setw_num) << (this->current_info.drone_status.ymax + this->current_info.drone_status.ymin) / 2 << " [Y]" << std::endl;
    if(this->current_info.drone_status.update)
        std::cout << "Drone (true ) : ";
    else
        std::cout << "Drone (false) : ";
    std::cout << std::setw(setw_num) << this->current_info.drone_status.raw_pcl_position.x << " [X]"
               << std::setw(setw_num) << this->current_info.drone_status.raw_pcl_position.y << " [Y]"
                << std::setw(setw_num) << this->current_info.drone_status.raw_pcl_position.z << " [Z]" << std::endl;
    std::cout << "     Drone tf : ";
    std::cout << std::setw(setw_num) << this->current_info.tf_drone_status.pose.position.x << " [X]"
               << std::setw(setw_num) << this->current_info.tf_drone_status.pose.position.y << " [Y]"
                << std::setw(setw_num) << this->current_info.tf_drone_status.pose.position.z << " [Z]" << std::endl;
    if(this->current_info.door_status.update)
        std::cout << "Door  (true ) : ";
    else
        std::cout << "Door  (false) : ";
    std::cout << std::setw(setw_num) << this->current_info.door_status.raw_pcl_position.x << " [X]"
               << std::setw(setw_num) << this->current_info.door_status.raw_pcl_position.y << " [Y]"
                << std::setw(setw_num) << this->current_info.door_status.raw_pcl_position.z << " [Z]"
                 << std::setw(setw_num) << this->current_info.door_status.yaw * 57.2957795 << " [YAW]" << std::endl;
    std::cout << std::endl;

}

GcsDisplay::GcsDisplay(const ros::NodeHandle& _nh, double _period) : RosBase(_nh, _period), current_info("gcs"), MAX_TRAJECTORY_BUF_SIZE(3000)
{
    rviz_trajectory.poses.resize(MAX_TRAJECTORY_BUF_SIZE);
    rviz_desired_trajectory.poses.resize(MAX_TRAJECTORY_BUF_SIZE);
    Initialize();
}
GcsDisplay::~GcsDisplay()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gcs_display");
    ros::NodeHandle nh;
    GcsDisplay GcsDisplay(nh, 1.0);
    // ros::spin();
    
    double ros_rate = 1.0;
    if(nh.getNamespace() == "/uav1")
    {
        ros::NodeHandle nh_temp("~");
        nh_temp.param<double>("leader_spin_rate/gcs_display", ros_rate, 1.0);
    }
    else
    {
        ros::NodeHandle nh_temp("~");
        nh_temp.param<double>("folower_spin_rate/gcs_display", ros_rate, 1.0);
    }

    std::cout << "-----------------------" << std::endl;
    std::cout << "ros rate: " << ros_rate << std::endl;
    ros::Rate loop(ros_rate);
    while(ros::ok())
    {
        ros::spinOnce();
        GcsDisplay.LoopTaskWithoutVirtual();
        loop.sleep();
    }
    return 0;

}

