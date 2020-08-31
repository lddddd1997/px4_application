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
    PidParameters param;
    ros::Time last_time;
    float expect_prev;
    float feedback_prev;
    float error;
    float error_prev;
    float integral;
    float delta_time;
    float output;
    Type pid_type;
    float GetDeltaTime(const ros::Time& _last_time);
};

PidController::PidController(Type _pid_type) : last_time(ros::Time::now()), expect_prev(0.0), feedback_prev(0.0), error(0.0), error_prev(0.0), 
                                                integral(0.0), delta_time(0.01), output(0.0), pid_type(_pid_type)
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
    this->param = _param;
}

void PidController::ResetIntegral(void)
{
    this->integral = 0.0;
}

float PidController::ControlOutput(float _expectation, float _feedback)
{
    this->error = _expectation - _feedback;
    this->error = MathUtils::Constrain(this->error, this->param.error_max);
    this->delta_time = GetDeltaTime(this->last_time);

    this->integral += this->error * this->delta_time;
    this->integral = MathUtils::Constrain(this->integral, this->param.integral_max);

    switch(this->pid_type)
    {
        case NORMAL:
        {
            this->output = this->param.kp * this->error 
                          + this->param.ki * this->integral 
                           + this->param.kd * (this->error - this->error_prev) / this->delta_time;
            break;
        }
        case DIFF_FIRST:
        {
            this->output = this->param.kp * this->error 
                          + this->param.ki * this->integral 
                           + this->param.kd * (_feedback - this->feedback_prev) / this->delta_time;
            break;
        }
        case DIFF_FEED:
        {
            this->output = this->param.kp * this->error 
                          + this->param.ki * this->integral 
                           + this->param.kd * (_feedback - this->feedback_prev) / this->delta_time 
                            + this->param.ff * (_expectation - this->expect_prev) / this->delta_time;
            break;
        }
        default:
        {
            this->output = 0.0;
            break;
        }
    }
    this->error_prev = this->error;
    this->expect_prev = _expectation;
    this->feedback_prev = _feedback;
    this->last_time = ros::Time::now();

    this->output = MathUtils::Constrain(this->output, this->param.output_max);
    return this->output;
}

void PidController::PrintParameters(void)
{
    // std::cout.setf(std::ios::fixed);
    // std::cout<<std::setprecision(1);
    std::cout << "kp:  " << this->param.kp << std::endl;
    std::cout << "ki:  " << this->param.ki << std::endl;
    std::cout << "kd:  " << this->param.kd << std::endl;
    std::cout << "ff:  " << this->param.ff << std::endl;
    std::cout << "error_max:  " << this->param.error_max << std::endl;
    std::cout << "integral_max:  " << this->param.integral_max << std::endl;
    std::cout << "output_max:  " << this->param.output_max << std::endl << std::endl;
}

#endif
