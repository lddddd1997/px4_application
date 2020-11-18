#ifndef PX4_APPLICATION_MATH_UTILS_H_
#define PX4_APPLICATION_MATH_UTILS_H_

#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <fstream>

class MathUtils
{
public:
    MathUtils();
    ~MathUtils();
    template<typename T>
    static T Constrain(T _data, T _max);
    template<typename T>
    static T Constrain(T _data, T _max, T _min);
    static void Quaternion2Euler(const geometry_msgs::Quaternion& _quat, geometry_msgs::Vector3& _euler);
    static void BodyHeading2Local(const geometry_msgs::Vector3& _body, geometry_msgs::Vector3& _local, double _yaw);
    static void Local2BodyHeading(const geometry_msgs::Vector3& _local, geometry_msgs::Vector3& _body, double _yaw);
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

void MathUtils::Quaternion2Euler(const geometry_msgs::Quaternion& _quat, geometry_msgs::Vector3& _euler)
{
    _euler.x = atan2(2.0 * (_quat.z * _quat.y + _quat.w * _quat.x), 1.0 - 2.0 * (_quat.x * _quat.x + _quat.y * _quat.y));
    _euler.y = asin(2.0 * (_quat.y * _quat.w - _quat.z * _quat.x));
    _euler.z = atan2(2.0 * (_quat.z * _quat.w + _quat.x * _quat.y), 1.0 - 2.0 * (_quat.y * _quat.y + _quat.z * _quat.z));
}

void MathUtils::BodyHeading2Local(const geometry_msgs::Vector3& _body, geometry_msgs::Vector3& _local, double _yaw)
{
    _local.x = _body.x * cos(_yaw) - _body.y * sin(_yaw);
    _local.y = _body.x * sin(_yaw) + _body.y * cos(_yaw);
    _local.z = _body.z;
}

void MathUtils:: Local2BodyHeading(const geometry_msgs::Vector3& _local, geometry_msgs::Vector3& _body, double _yaw)
{
    _body.x = _local.x * cos(_yaw) + _local.y * sin(_yaw);
    _body.y = -_local.x * sin(_yaw) + _local.y * cos(_yaw);
    _body.z = _local.z;
}

class FunctionUtils
{
public:
    FunctionUtils();
    ~FunctionUtils();
    static void DataFileWrite(const geometry_msgs::Vector3& _data_0, const geometry_msgs::Vector3& _data_1, std::string _file_name);
private:
};

void FunctionUtils::DataFileWrite(const geometry_msgs::Vector3& _data_0, const geometry_msgs::Vector3& _data_1, std::string _file_name)
{
    std::ofstream out_file;
    out_file.open(_file_name, std::ios_base::app);

    out_file << _data_0.x << "\t" << _data_0.y << "\t" << _data_0.z << "\t"
             << _data_1.x << "\t" << _data_1.y << "\t" << _data_1.z << "\n";
    out_file.close();
}

#endif
