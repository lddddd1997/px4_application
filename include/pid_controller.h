#ifndef PX4_APPLICATION_PID_CONTROLLER_H_
#define PX4_APPLICATION_PID_CONTROLLER_H_

struct PidParameters
{
    PidParameters();
    ~PidParameters();
    float p;
    float i;
    float d;
    float f;
};

PidParameters::PidParameters() : p(0.0), i(0.0), d(0.0), f(0.0)
{

}

PidParameters::~PidParameters()
{

}

class PidController
{
public:
    PidController();
    ~PidController();
    enum Type
    {
        NORMAL = 0u,    //普通的PID
        FEEDFORWARD = 1u,    //微分先行
        DIFF_FEED = 2U    //前馈加微分先行
    };
    void SetParameters(const PidParameters& _Param);
    float ControlOutput(float _expectation, float _feedback, Type _type);

private:
    PidParameters Param_;
    float error_;
    float integral_;
    float integral_max_;
    float output_max_;
};

PidController::PidController()
{

}

PidController::~PidController()
{

}

void PidController::SetParameters(const PidParameters& _Param)
{
    Param_ = _Param;
}

float PidController::ControlOutput(float _expectation, float _feedback, Type _type)
{
    error_ = _expectation - _feedback;

}

#endif
