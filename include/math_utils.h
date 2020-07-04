#ifndef PX4_APPLICATION_MATH_UTILS_H_
#define PX4_APPLICATION_MATH_UTILS_H_

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>


class MathUtils
{
public:
    MathUtils();
    ~MathUtils();
    template<typename T>
    static T Constrain(T _data, T _max);
    template<typename T>
    static T Constrain(T _data, T _max, T _min);
    static void Quaternion2Euler(const geometry_msgs::Quaternion &quat, geometry_msgs::Vector3& euler);
    
private:
};

MathUtils::MathUtils()
{

}


MathUtils::~MathUtils()
{
    
}

template<typename T>
T MathUtils::Constrain(T _data, T _max)
{
    if(abs(_data)>_max)
    {
        return (_data > 0.0f) ? _max : -_max;
    }
    else
    {
        return _data;
    }
}

template<class T>
T MathUtils::Constrain(T _data, T _max, T _min)
{
    if(_data > _max)
    {
        return _max;
    }
    else if (_data < _min)
    {
        return _min;
    }
    else
    {
        return _data;
    }
}

void MathUtils::Quaternion2Euler(const geometry_msgs::Quaternion &quat, geometry_msgs::Vector3& euler)
{
    euler.x = atan2(2.0 * (quat.z * quat.y + quat.w * quat.x), 1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y));
    euler.y = asin(2.0 * (quat.y * quat.w - quat.z * quat.x));
    euler.z = atan2(2.0 * (quat.z * quat.w + quat.x * quat.y), 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));

}

#endif
