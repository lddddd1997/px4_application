#ifndef PX4_APPLICATION_UAV_ESTIMATOR_H_
#define PX4_APPLICATION_UAV_ESTIMATOR_H_

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "ros_base/ros_base.h"

class UavEstimator : public RosBase
{
public:
    UavEstimator(const ros::NodeHandle& _nh, double _period);
    ~UavEstimator();
    void LoopTaskWithoutVirtual(void);
private:
    ros::Publisher vision_pub; // 发布给飞控，同时要在QGC设置定位来源为vision
    ros::Subscriber ground_truth_sub; // 订阅gazebo仿真环境真值

    geometry_msgs::PoseStamped vision_uav;
    nav_msgs::Odometry ground_truth; // gazebo仿真环境真值
    int source_input; // 定位来源
    int own_id;

    void GroundTruthCallback(const nav_msgs::Odometry::ConstPtr& _msg);

    void Initialize(void);
    virtual void LoopTask(void);

    enum
    {
        truth = 0u,
        /*laser = 1u,
        optitrack = 2u,
        slam = 3u,*/
    };

};

#endif