#ifndef PX4_APPLICATION_PID_CONTROLLER_H_
#define PX4_APPLICATION_PID_CONTROLLER_H_

#include <ros/ros.h>
#include "math_utils.h"

struct PidParameters
{
    PidParameters();
    ~PidParameters();
    float p;
    float i;
    float d;
    float ff;    //前馈增益
    float error_max;
    float integral_max;
    float output_max;
};

PidParameters::PidParameters() : p(0.0), i(0.0), d(0.0), ff(0.0), error_max(0.0), integral_max(0.0), output_max(0.0)
{
    
}

PidParameters::~PidParameters()
{

}

class PidController
{
public:
    enum Type
    {
        NORMAL = 0u,    //普通的PID
        DIFF_FIRST = 1u,    //微分先行
        DIFF_FEED = 2U    //微分先行加前馈
    };
public:
    PidController(Type _pid_type);
    ~PidController();
    float GetDeltaTime(const ros::Time& _last_time);
    void SetParameters(const PidParameters& _Param);
    void ResetIntegral(void);
    float ControlOutput(float _expectation, float _feedback);

private:
    PidParameters Param_;
    ros::Time last_time_;
    float expect_prev_;
    float error_;
    float error_prev_;
    float integral_;
    float delta_time_;
    float output_;
    Type pid_type_;
};

PidController::PidController(Type _pid_type) : last_time_(ros::Time::now()), expect_prev_(0.0), error_(0.0), error_prev_(0.0), 
                                                integral_(0.0), delta_time_(0.01), output_(0.0), pid_type_(_pid_type)
{

}

PidController::~PidController()
{

}

float PidController::GetDeltaTime(const ros::Time& _last_time)
{
    ros::Time current_time = ros::Time::now();
    float current_time_sec = current_time.sec - _last_time.sec;
    float current_time_nsec = current_time.nsec / 1e9 - _last_time.nsec / 1e9;
    return (current_time_sec + current_time_nsec);
}

void PidController::SetParameters(const PidParameters& _Param)
{
    Param_ = _Param;
}

void PidController::ResetIntegral(void)
{
    integral_ = 0.0;
}

float PidController::ControlOutput(float _expectation, float _feedback)
{
    error_ = _expectation - _feedback;
    error_ = MathUtils::Constrain(error_, Param_.error_max);
    delta_time_ = GetDeltaTime(last_time_);
    last_time_ = ros::Time::now();
    switch(pid_type_)
    {
        case NORMAL:
        {
            output_ = Param_.p * error_ + Param_.i * integral_ + Param_.d;
            break;
        }
        case DIFF_FIRST:
        {
            output_ = Param_.p * error_ + Param_.i * integral_ + Param_.d ;
            break;
        }
        case DIFF_FEED:
        {
            output_ = Param_.p * error_ + Param_.i * integral_ + Param_.d + Param_.ff * (_expectation - expect_prev_) / delta_time_;
            break;
        }
        default:
        {
            output_ = 0.0;
            break;
        }
    }
    error_prev_ = error_;
    expect_prev_ = _expectation;
}

#endif
