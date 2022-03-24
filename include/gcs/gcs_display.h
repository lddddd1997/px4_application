#ifndef PX4_APPLICATION_GCS_DISPLAY_H_
#define PX4_APPLICATION_GCS_DISPLAY_H_

#include <nav_msgs/Path.h>
#include "ros_base/ros_base.h"
#include "subscriber/status_subscriber.h"
#include "px4_application/UavCommand.h"

class GcsDisplay : public RosBase
{
public:
    GcsDisplay(const ros::NodeHandle& _nh, double _period);
    ~GcsDisplay();
    void LoopTaskWithoutVirtual(void);

private:
    ros::Time begin_time;
    ros::Subscriber uav_command_sub;

    ros::Publisher rviz_pose_pub;
    ros::Publisher rviz_desired_pose_pub;

    ros::Publisher rviz_trajectory_pub;
    ros::Publisher rviz_desired_trajectory_pub;

    bool debug_pub;
    int own_id;
    char c_id;
    const int MAX_TRAJECTORY_BUF_SIZE;

    StatusSubscriber current_info;    //无人机与目标状态
    px4_application::UavCommand command_reception;

    geometry_msgs::PoseStamped rviz_pose;
    geometry_msgs::PoseStamped rviz_desired_pose;
    
    nav_msgs::Path rviz_trajectory;
    nav_msgs::Path rviz_desired_trajectory;
    

    float GetTimePassSec(void);
    void CommandCallback(const px4_application::UavCommand::ConstPtr& _msg);

    void Initialize(void);
    void UavStateDisplay(void); // 终端地面站信息显示
    void RvizTrajectoryDisplay(void); // rviz轨迹显示
    void RvizPoseDisplay(void); // rviz位姿显示
    void CommandUpdateReset(void);

    virtual void LoopTask(void);
};

template<class T>
class RvizVisualization
{
public:
    RvizVisualization(ros::NodeHandle& nh, const std::string& topic_name)
    {
        visualization_pub = nh.advertise<T>(topic_name, 10);
    }
    ~RvizVisualization()
    {

    }
    void PublishMessage(const T& msg)
    {
        visualization_pub.publish(msg);
    }
private:
    ros::Publisher visualization_pub;
};

#endif