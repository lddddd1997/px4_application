/** 
* @file     gcs_setting.cpp
* @brief    无人机模式设置
* @details  
* @note
* @author   lddddd
*           Email: lddddd1997@gmail.com
*           Github: https://github.com/lddddd1997
* @date     2020.7.21
* @version  2.0
* @par      Edit history:
*           1.0: lddddd, 2020.5.24, .
*           2.0: lddddd, 2020.7.21, 更新节点句柄与topic的命名空间.
*/

#include "gcs_setting.h"

void GcsSetting::StateCallback(const mavros_msgs::State::ConstPtr& _msg)
{
    this->current_state_uav = *_msg;
}

void GcsSetting::LoopTask(void)
{
    // std::cout << "Virtual Loop Task of Derived Class !" << std::endl;
}

void GcsSetting::Initialize(void)
{
    this->control_mode = "POSCTL";
    this->armed_cmd = false;
    this->uav_state_sub = this->nh.subscribe<mavros_msgs::State>("mavros/state",
                                                                  10, &GcsSetting::StateCallback,
                                                                   this,
                                                                    ros::TransportHints().tcpNoDelay());
    this->uav_set_mode_client = this->nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    this->uav_arming_client = this->nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
}

void GcsSetting::ModeSelect(void)
{
    int select_flag;
    int mode_flag;
    uint8_t timeout_count;
    std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>gcs control<<<<<<<<<<<<<<<<<<<<<<<<<<<<< " << std::endl;
    std::cout << "Input the selection:  0 for set mode,1 for arm or disarm..." << std::endl;
    std::cin >> select_flag;

    if(select_flag == 0)
    {
        std::cout << "Input the mode:  0 for set _OFFBOARD_,1 for _POSCTL_,2 for _AUTO.RTL_,3 for _AUTO.LAND_" << std::endl;
        std::cin >> mode_flag;
        switch(mode_flag)
        {
            case 0: this->control_mode = "OFFBOARD"; break;

            case 1: this->control_mode = "POSCTL"; break;

            case 2: this->control_mode = "AUTO.RTL"; break;

            case 3: this->control_mode = "AUTO.LAND"; break;

            default:std::cout << "Input error,please retry..." << std::endl; std::cout << std::endl; break;
        }
        if(mode_flag < 4)    //暂时设置四种模式
        {
            timeout_count= 0;
            std::cout << "Setting to " << this->control_mode << " Mode..." << std::endl;
            ros::spinOnce();
            while(this->current_state_uav.mode != this->control_mode)
            {
                this->uav_mode_cmd.request.custom_mode = this->control_mode;
                this->uav_set_mode_client.call(this->uav_mode_cmd);

                ros::spinOnce();
                ros::Rate(10).sleep();
                timeout_count++;
                if(timeout_count > 20)    //超时则退出
                    break;
            }
            if(timeout_count > 20)
                std::cout << "Set to " << this->control_mode << " Mode Timeout!!!" << std::endl;
            else
                std::cout << "Set to " << this->control_mode << " Mode Susscess!!!" << std::endl;
            std::cout << std::endl;
        } 
    }
    else if(select_flag == 1)
    {
        std::cout << "Input the CMD:  0 for Disarmed,1 for Armed" << std::endl;
        std::cin >> mode_flag;
        switch(mode_flag)
        {
            case 0: this->armed_cmd = false; break;

            case 1: this->armed_cmd = true; break;
            default:std::cout << "Input error,please retry..." << std::endl; break;
        }
        if(mode_flag < 2)
        {
            timeout_count = 0;
            if(this->armed_cmd)
                std::cout << "Uav Armed..." << std::endl;
            else
                std::cout << "Uav Disarmed..." << std::endl;
            ros::spinOnce();
            while(this->current_state_uav.armed != this->armed_cmd)
            {
                this->uav_arm_cmd.request.value = this->armed_cmd;
                this->uav_arming_client.call(this->uav_arm_cmd);

                ros::spinOnce();
                ros::Rate(10).sleep();
                timeout_count++;
                if(timeout_count > 20)    //超时则退出
                    break;
            }
            if(timeout_count > 20)
            {
                if(this->armed_cmd)
                    std::cout << "Armed Timeout!!!" << std::endl;
                else
                    std::cout << "Disarmed Timeout!!!" << std::endl;
            }
            else
            {
                if(this->armed_cmd)
                    std::cout << "Armed Susscess!!!" << std::endl;
                else
                    std::cout << "Disarmed Susscess!!!" << std::endl;
            }
            std::cout << std::endl;
        }
    }
    else
    {
        std::cout << "Input error,please retry..." << std::endl;
        std::cout << std::endl;
    }
        
}

GcsSetting::GcsSetting(const ros::NodeHandle& _nh, double _period) : RosBase(_nh, _period)
{
    Initialize();
}

GcsSetting::~GcsSetting()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gcs_setting");
    ros::NodeHandle nh;
    GcsSetting GcsSetting(nh, 0.1);
    ros::Rate rate(10.0);

    while(ros::ok())
    {
        ros::spinOnce();
        GcsSetting.ModeSelect();
        rate.sleep();
    }

    return 0;
}
