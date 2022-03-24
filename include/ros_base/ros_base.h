#ifndef PX4_APPLICATION_ROS_BASE_H_
#define PX4_APPLICATION_ROS_BASE_H_

#include <iostream>
#include <ros/ros.h>

class RosBase
{
public:
    RosBase(const ros::NodeHandle& _nh, double _period);
    virtual ~RosBase();    //防止调用不到派生类的析构函数（派生类有堆区数据的情况）
    
protected:
    ros::NodeHandle nh;
    virtual void LoopTask() = 0;    //纯虚函数，提供接口

private:
    ros::Timer loop_timer;
    double loop_period;
    
    void LoopTimerCallback(const ros::TimerEvent& event);
    virtual void Initialize(void);
};

RosBase::RosBase(const ros::NodeHandle& _nh, double _period) : nh(_nh), loop_period(_period)
{
    this->loop_timer = this->nh.createTimer(ros::Duration(this->loop_period), &RosBase::LoopTimerCallback, this);
    Initialize();
}

RosBase::~RosBase()
{
    
}

void RosBase::Initialize(void)
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
