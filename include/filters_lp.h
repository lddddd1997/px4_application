#ifndef PX4_APPLICATION_FLITERSLP_H_
#define PX4_APPLICATION_FLITERSLP_H_

#include "math_utils.h"

class FiltersLP
{
public:
    FiltersLP() : PI(3.1415926535897932384626433832795)
    {

    }
    ~FiltersLP()
    {

    }

protected:
    double k[3];

    bool IIRCalculateParamters(double _sample_freq, double _cutoff_freq, double _cp)
    {
        if ((_cutoff_freq <= 0.0001) || (_sample_freq <= 1.99 * _cutoff_freq))    // no filtering
        {
            return false;
        }
        double cos_PI_cp = cos( PI * _cp );
        
        double fr = _sample_freq / _cutoff_freq;
        double ohm = tan(PI / fr);
        double ohm2 = ohm * ohm;
        double c = 1.0 + 2.0 * cos_PI_cp * ohm + ohm2;
        double inv_c = 1.0 / c;
        this->k[0] = ohm2 * inv_c;
        this->k[1] = 2.0 * (ohm2-1.0) * inv_c;
        this->k[2] = (1.0 - 2.0 * cos_PI_cp * ohm + ohm2) * inv_c;
        return true;
    }
private:
    const double PI;
};

class Butterworth2 : public FiltersLP
{
private:
    bool available;	
    double in[2];
    double out[2];
public:
    bool IsAvailable()
    {
        return this->available;
    }
    void AvailableReset()
    {
        this->available = false;
    }

    bool SetCutoffFrequency(double _fs, double _fc)
    {
        this->available = IIRCalculateParamters(_fs, _fc, 1.0 / 4);
        return this->available;
    }
    void SetCutoffFrequencyFrom(const Butterworth2& _Filter)
    {
        this->k[0] = _Filter.k[0];
        this->k[1] = _Filter.k[1];
        this->k[2] = _Filter.k[2];
        this->available = _Filter.available;		
    }

    void InitialValue(double _value)
    {
        this->in[0] = this->in[1] = _value;
        this->out[0] = this->out[1] = _value;
    }

    double GetResult()
    {
        return this->out[0];
    }

    double run(double _newdata)
    {
        if(this->available)
        {
            double out_1_2 = this->out[1];
            this->out[1] = this->out[0];
            this->out[0] = this->k[0]*(_newdata + 2 * this->in[0] + this->in[1]) - this->k[1] * this->out[1] - this->k[2] * out_1_2;			
            this->in[1] = this->in[0];
            this->in[0] = _newdata;
        }
        else
            InitialValue(_newdata);
        return this->out[0];
    }

    double AddOffset(double _offset)
    {
        this->in[0] += _offset;
        this->in[1] += _offset;
        this->out[0] += _offset;
        this->out[1] += _offset;				
        return this->out[0];
    }

    Butterworth2()
    {
        this->available = false;
        this->InitialValue(0);
    }

    Butterworth2(double _fs, double _fc)
    {
        SetCutoffFrequency(_fs, _fc);
        this->InitialValue(0);
    }
};

#endif
