#ifndef PX4_APPLICATION_ROS_BASE_H_
#define PX4_APPLICATION_ROS_BASE_H_

#include <ros/ros.h>
#include <iostream>

class RosBase
{
public:
    RosBase(const ros::NodeHandle& _nh, const double _period);
    virtual ~RosBase(); //防止调用不到派生类的析构函数（派生类有堆区数据的情况）
    
protected:
    ros::NodeHandle nh_;
    virtual void LoopTask() = 0;    //纯虚函数，提供接口

private:
    ros::Timer loop_timer_;
    double loop_period_;
    void LoopTimerCallback(const ros::TimerEvent& event);
};

RosBase::RosBase(const ros::NodeHandle& _nh, const double _period):nh_(_nh), loop_period_(_period)
{
    loop_timer_ = nh_.createTimer(ros::Duration(loop_period_), &RosBase::LoopTimerCallback, this);
}

RosBase::~RosBase()
{
    
}

void RosBase::LoopTimerCallback(const ros::TimerEvent& _event)
{
    LoopTask();
}

// void RosBase::LoopTask()
// {
//     std::cout << "Virtual Loop Task of Base Class !" << std::endl;
// }

#endif
