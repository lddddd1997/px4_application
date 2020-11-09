#ifndef PX4_APPLICATION_UAV_MISSION_TEMPLATE_H_
#define PX4_APPLICATION_UAV_MISSION_TEMPLATE_H_

#include "ros_base.h"
#include "status_subscriber.h"
#include "px4_application/UavCommand.h"
#include "pid_controller.h"

class States
{
public:
    States();
    virtual ~States();
    void StateMachineSchedule(const StatusSubscriber& _current_info,
                               const ros::Publisher& _uav_command_pub,
                                px4_application::UavCommand* _command_deliver,
                                 States** _State);    //注：使用指针的指针，确保能访问到对象指针本身，因为状态转移需delete与new操作
private:
    virtual void Run(const StatusSubscriber& _current_info,
                      const ros::Publisher& _uav_command_pub,
                       px4_application::UavCommand* _command_deliver,
                        States** _State) = 0;
protected:
    geometry_msgs::Vector3 reach_point_range;
};

class Prepare : public States
{
public:
    Prepare();
    ~Prepare();
private:
    virtual void Run(const StatusSubscriber& _current_info,
                      const ros::Publisher& _uav_command_pub,
                       px4_application::UavCommand* _command_deliver,
                        States** _State);
};

class TakeOff : public States
{
public:
    TakeOff();
    ~TakeOff();
private:
    virtual void Run(const StatusSubscriber& _current_info,
                      const ros::Publisher& _uav_command_pub,
                       px4_application::UavCommand* _command_deliver,
                        States** _State);

    geometry_msgs::Vector3 takeoff_position;
    geometry_msgs::Vector3 takeoff_absolute_position_param;    //绝对起飞位置
    double takeoff_relative_height_param;    //相对起飞高度
    bool takeoff_id;    //false绝对起飞 true相对起飞
};

class Mission : public States
{
public:
    Mission();
    ~Mission();
private:
    virtual void Run(const StatusSubscriber& _current_info,
                      const ros::Publisher& _uav_command_pub,
                       px4_application::UavCommand* _command_deliver,
                        States** _State);
    geometry_msgs::Vector3 mission_position;
    int own_id;
    OtherSubscriber total_info;    //所有无人机信息
};

class ReturnHome : public States
{
public:
    ReturnHome();
    ~ReturnHome();
private:
    virtual void Run(const StatusSubscriber& _current_info,
                      const ros::Publisher& _uav_command_pub,
                       px4_application::UavCommand* _command_deliver,
                        States** _State);
    geometry_msgs::Vector3 home_position;
};

class Landing : public States
{
public:
    Landing();
    ~Landing();
private:
    virtual void Run(const StatusSubscriber& _current_info,
                      const ros::Publisher& _uav_command_pub,
                       px4_application::UavCommand* _command_deliver,
                        States** _State);
    geometry_msgs::Vector3 landing_pos_vel;
    
};

class Finished : public States
{
public:
    Finished();
    ~Finished();
private:
    virtual void Run(const StatusSubscriber& _current_info,
                      const ros::Publisher& _uav_command_pub,
                       px4_application::UavCommand* _command_deliver,
                        States** _State);
};

class UavMission : public RosBase 
{
public:
    UavMission(const ros::NodeHandle& _nh, double _period);
    ~UavMission();
    
private:

    ros::Publisher uav_command_pub;

    px4_application::UavCommand command_deliver;
    StatusSubscriber current_info;    //本无人机和目标状态

    States* UavState;

    void Initialize(void);

    virtual void LoopTask(void);
};

#endif