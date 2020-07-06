#ifndef PX4_APPLICATION_PID_CONTROLLER_H_
#define PX4_APPLICATION_PID_CONTROLLER_H_

#include <ros/ros.h>
#include "math_utils.h"

struct PidParameters
{
    PidParameters();
    ~PidParameters();
    float kp;
    float ki;
    float kd;
    float ff;    //前馈增益
    float error_max;
    float integral_max;
    float output_max;
};

PidParameters::PidParameters() : kp(0.0), ki(0.0), kd(0.0), ff(0.0), error_max(0.0), integral_max(0.0), output_max(0.0)
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
    void SetParameters(const PidParameters& _param);
    void ResetIntegral(void);
    float ControlOutput(float _expectation, float _feedback);
    void PrintParameters(void);
private:
    PidParameters param_;
    ros::Time last_time_;
    float expect_prev_;
    float feedback_prev_;
    float error_;
    float error_prev_;
    float integral_;
    float delta_time_;
    float output_;
    Type pid_type_;
    float GetDeltaTime(const ros::Time& _last_time);
};

PidController::PidController(Type _pid_type) : last_time_(ros::Time::now()), expect_prev_(0.0), feedback_prev_(0.0), error_(0.0), error_prev_(0.0), 
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

void PidController::SetParameters(const PidParameters& _param)
{
    param_ = _param;
}

void PidController::ResetIntegral(void)
{
    integral_ = 0.0;
}

float PidController::ControlOutput(float _expectation, float _feedback)
{
    error_ = _expectation - _feedback;
    error_ = MathUtils::Constrain(error_, param_.error_max);
    delta_time_ = GetDeltaTime(last_time_);

    integral_ += error_ * delta_time_;
    integral_ = MathUtils::Constrain(integral_, param_.integral_max);

    switch(pid_type_)
    {
        case NORMAL:
        {
            output_ = param_.kp * error_ 
                     + param_.ki * integral_ 
                      + param_.kd * (error_ - error_prev_) / delta_time_;
            break;
        }
        case DIFF_FIRST:
        {
            output_ = param_.kp * error_ 
                     + param_.ki * integral_ 
                      + param_.kd * (_feedback - feedback_prev_) / delta_time_;
            break;
        }
        case DIFF_FEED:
        {
            output_ = param_.kp * error_ 
                     + param_.ki * integral_ 
                      + param_.kd * (_feedback - feedback_prev_) / delta_time_ 
                       + param_.ff * (_expectation - expect_prev_) / delta_time_;
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
    feedback_prev_ = _feedback;
    last_time_ = ros::Time::now();

    output_ = MathUtils::Constrain(output_, param_.output_max);
    return output_;
}

void PidController::PrintParameters(void)
{
    // std::cout.setf(std::ios::fixed);
    // std::cout<<std::setprecision(1);
    std::cout << "kp:  " << this->param_.kp << std::endl;
    std::cout << "ki:  " << this->param_.ki << std::endl;
    std::cout << "kd:  " << this->param_.kd << std::endl;
    std::cout << "ff:  " << this->param_.ff << std::endl;
    std::cout << "error_max:  " << this->param_.error_max << std::endl;
    std::cout << "integral_max:  " << this->param_.integral_max << std::endl;
    std::cout << "output_max:  " << this->param_.output_max << std::endl << std::endl;
}

#endif
