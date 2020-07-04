/** 
* @file     gcs_setting.cpp
* @brief    无人机模式设置
* @details  
* @note
* @author   lddddd
*           Email: lddddd1997@gmail.com
* @date     2020.5.24
* @version  1.0
* @par      Edit history:
*           1.0: lddddd, 2020.5.24, .
*/
#include "gcs_setting.h"

// void GcsSetting::LoopTimerCallback(const ros::TimerEvent& _event)
// {

// }

void GcsSetting::UavStateCallback(const mavros_msgs::State::ConstPtr& _msg)
{
    current_state_uav_ = *_msg;
}

void GcsSetting::LoopTask(void)
{
    // std::cout << "Virtual Loop Task of Derived Class !" << std::endl;
}

void GcsSetting::Initialize(void)
{
    control_mode_ = "POSCTL";
    armed_cmd_ = false;
    
    // loop_timer_ = nh_.createTimer(ros::Duration(loop_period_), &GcsSetting::LoopTimerCallback, this);
    uav_state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state",
                                                        10, &GcsSetting::UavStateCallback,
                                                         this,
                                                          ros::TransportHints().tcpNoDelay());
    uav_set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    uav_arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
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
            case 0: control_mode_ = "OFFBOARD"; break;

            case 1: control_mode_ = "POSCTL"; break;

            case 2: control_mode_ = "AUTO.RTL"; break;

            case 3: control_mode_ = "AUTO.LAND"; break;

            default:std::cout << "Input error,please retry..." << std::endl; std::cout << std::endl; break;
        }
        if(mode_flag < 4)    //暂时设置四种模式
        {
            timeout_count= 0;
            std::cout << "Setting to " << control_mode_ << " Mode..." << std::endl;
            ros::spinOnce();
            while(current_state_uav_.mode != control_mode_)
            {
                uav_mode_cmd_.request.custom_mode = control_mode_;
                uav_set_mode_client_.call(uav_mode_cmd_);

                ros::spinOnce();
                ros::Rate(10).sleep();
                timeout_count++;
                if(timeout_count > 20)    //超时则退出
                    break;
            }
            if(timeout_count > 20)
                std::cout << "Set to " << control_mode_ << " Mode Timeout!!!" << std::endl;
            else
                std::cout << "Set to " << control_mode_ << " Mode Susscess!!!" << std::endl;
            std::cout << std::endl;
        } 
    }
    else if(select_flag == 1)
    {
        std::cout << "Input the CMD:  0 for Disarmed,1 for Armed" << std::endl;
        std::cin >> mode_flag;
        switch(mode_flag)
        {
            case 0: armed_cmd_ = false; break;

            case 1: armed_cmd_ = true; break;
            default:std::cout << "Input error,please retry..." << std::endl; break;
        }
        if(mode_flag < 2)
        {
            timeout_count = 0;
            if(armed_cmd_)
                std::cout << "Uav Armed..." << std::endl;
            else
                std::cout << "Uav Disarmed..." << std::endl;
            ros::spinOnce();
            while(current_state_uav_.armed != armed_cmd_)
            {
                uav_arm_cmd_.request.value = armed_cmd_;
                uav_arming_client_.call(uav_arm_cmd_);

                ros::spinOnce();
                ros::Rate(10).sleep();
                timeout_count++;
                if(timeout_count > 20)    //超时则退出
                    break;
            }
            if(timeout_count > 20)
            {
                if(armed_cmd_)
                    std::cout << "Armed Timeout!!!" << std::endl;
                else
                    std::cout << "Disarmed Timeout!!!" << std::endl;
            }
            else
            {
                if(armed_cmd_)
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
    ros::NodeHandle nh("~");
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
