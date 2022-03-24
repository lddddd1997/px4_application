#ifndef TRAJECTORY_UTILS
#define TRAJECTORY_UTILS

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <nav_msgs/Path.h>

template<class T>
class TrajectoryUtils
{
public:
    TrajectoryUtils();
    ~TrajectoryUtils();
    static void TrajectorySample(T& traj, double sample_time, nav_msgs::Path& path)
    {
        double total_duration = traj.getTotalDuration();
        for (double t = 0.0; t < total_duration; t += sample_time)
        {
            Eigen::Vector3d pos = traj.getPos(t);
            geometry_msgs::PoseStamped quat_pos;
            quat_pos.pose.position.x = pos(0);
            quat_pos.pose.position.y = pos(1);
            quat_pos.pose.position.z = pos(2);
            path.poses.push_back(quat_pos);
        }
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "world";
    }
private:
};

class TypeTransform
{
public:
    TypeTransform();
    ~TypeTransform();
    template<typename T>
    static inline Eigen::Vector3d RosMsg2Eigen(const T& p)
    {
        Eigen::Vector3d ev3(p.x, p.y, p.z);
        return ev3;
    }
    static inline void Eigen2RosMsg(const Eigen::Vector3d& ev3, geometry_msgs::Vector3& gv3)
    {
        gv3.x = ev3[0], gv3.y = ev3[1], gv3.z = ev3[2];
    }
private:
};

#endif

