/** 
* @file     app_visual_formation.cpp
* @brief    无人机视觉编队程序
* @details  
* @note
* @author   lddddd
*           Email: lddddd1997@gmail.com
*           Github: https://github.com/lddddd1997
* @date     2021.4.7
* @version  1.0
* @par      Edit history:
*           1.0: lddddd, 2021.4.7 .
*/

#include "application/app_visual_formation.h"

void UavMission::LoopTask(void)
{
    // this->UavState->StateMachineSchedule(this->current_info,
    //                                       this->uav_command_pub,
    //                                        &this->command_deliver,
    //                                         &this->UavState);    //运行状态机调度
}

void UavMission::LoopTaskWithoutVirtual(void)
{
    this->UavState->StateMachineSchedule(this->current_info,
                                          this->uav_command_pub,
                                           &this->command_deliver,
                                            &this->UavState);    //运行状态机调度
}

void UavMission::Initialize(void)
{
    this->uav_command_pub = this->nh.advertise<px4_application::UavCommand>("px4_application/uav_command", 10);

    this->UavState = new Prepare;    //初始为准备状态
}

UavMission::UavMission(const ros::NodeHandle& _nh, double _period) : RosBase(_nh, _period), current_info("mission")
{
    Initialize();
}

UavMission::~UavMission()
{
    if(this->UavState != NULL)
    {
        delete this->UavState;
        this->UavState = NULL;
    }
}

/**
* @name         void States::StateMachineSchedule(const StatusSubscriber& _current_info,
                                                   const ros::Publisher& _uav_command_pub,
                                                    px4_application::UavCommand* _command_deliver,
                                                     States** _State);
* @brief        简易状态机调度（基类）
* @param[in]    无人机与目标状态：_current_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL
*/
void States::StateMachineSchedule(const StatusSubscriber& _current_info,
                                   const ros::Publisher& _uav_command_pub,
                                    px4_application::UavCommand* _command_deliver,
                                     States** _State)
{
    Run(_current_info, _uav_command_pub, _command_deliver, _State);
}

States::States()
{
    ros::NodeHandle nh("~");
    nh.param<double>("range/x", this->reach_point_range.x, 0.1);
    nh.param<double>("range/y", this->reach_point_range.y, 0.1);
    nh.param<double>("range/z", this->reach_point_range.z, 0.1);
}

States::~States()
{
    
}

/**
* @name         void Prepare::Run(const StatusSubscriber& _current_info,
                                   const ros::Publisher& _uav_command_pub,
                                    px4_application::UavCommand* _command_deliver,
                                     States** _State)
* @brief        准备任务接口（派生类）
* @param[in]    无人机与目标状态：_current_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL  
*/
void Prepare::Run(const StatusSubscriber& _current_info,
                   const ros::Publisher& _uav_command_pub,
                    px4_application::UavCommand* _command_deliver,
                     States** _State)
{

    if(!(_current_info.uav_status.estimator_status.attitude_status_flag
         && _current_info.uav_status.estimator_status.velocity_horiz_status_flag
          && _current_info.uav_status.estimator_status.velocity_vert_status_flag))
    {
        _command_deliver->header.stamp = ros::Time::now();
        _command_deliver->command_type = px4_application::UavCommand::NORMAL;
        _command_deliver->period = 0.05;
        _command_deliver->update = true;
        _command_deliver->xyz_id = px4_application::UavCommand::UX_UY_UZ;
        _command_deliver->task_name = "Prepare";
        _command_deliver->yaw = _current_info.uav_status.attitude_angle.z;    //设置初始航向，起飞状态也会设置，在此设置是为了防止忽略起飞状态，直接跳到其他状态
        _uav_command_pub.publish(*_command_deliver);
        return ;
    }
    delete *_State;
    *_State = new Mission;    //状态转移
}

Prepare::Prepare()
{
    std::cout << "[ Prepare ]" << std::endl;
    ROS_ERROR("Waiting for state estimation to complete..." );
}

Prepare::~Prepare()
{
    std::cout << "Prepare state transition..." << std::endl;
}

/**
* @name         void TakeOff::Run(const StatusSubscriber& _current_info,
                                   const ros::Publisher& _uav_command_pub,
                                    px4_application::UavCommand* _command_deliver,
                                     States** _State)
* @brief        起飞任务接口（派生类）
* @param[in]    无人机与目标状态：_current_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL
*/
void TakeOff::Run(const StatusSubscriber& _current_info,
                   const ros::Publisher& _uav_command_pub,
                    px4_application::UavCommand* _command_deliver,
                     States** _State)
{
    // if(!(_current_info.uav_status.estimator_status.attitude_status_flag
    //      && _current_info.uav_status.estimator_status.velocity_horiz_status_flag
    //       && _current_info.uav_status.estimator_status.velocity_vert_status_flag))
    // {
    //     ROS_INFO_STREAM_THROTTLE( 1, "Waiting for state estimation to complete..." );
    //     return ;
    // }

    if(_current_info.uav_status.extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
    {
        if(this->takeoff_id)
        {
            this->takeoff_position.x = _current_info.uav_status.local_position.x;
            this->takeoff_position.y = _current_info.uav_status.local_position.y;
            this->takeoff_position.z = _current_info.uav_status.local_position.z + this->takeoff_relative_height_param;
        }
        else
        {
            this->takeoff_position.x = this->takeoff_absolute_position_param.x;
            this->takeoff_position.y = this->takeoff_absolute_position_param.y;
            this->takeoff_position.z = this->takeoff_absolute_position_param.z;
        }
        _command_deliver->yaw = _current_info.uav_status.attitude_angle.z;    //设置为初始航向
    }

    if(!(abs(_current_info.uav_status.local_position.x - this->takeoff_position.x) < this->reach_point_range.x &&
          abs(_current_info.uav_status.local_position.y - this->takeoff_position.y) < this->reach_point_range.y &&
           abs(_current_info.uav_status.local_position.z - this->takeoff_position.z) < this->reach_point_range.z))    //认为起飞未完成
    {
        _command_deliver->header.stamp = ros::Time::now();    //发送时间戳
        _command_deliver->command_type = px4_application::UavCommand::NORMAL;
        _command_deliver->period = 0.05;    //发送指令的周期，暂未用到
        _command_deliver->update = true;    //指令是否更新
        _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_PZ;    //x y z的控制模式P对应位置控制，V对应速度控制，U未定义
        _command_deliver->yaw_id = px4_application::UavCommand::YAW;    //是否进行航向控制
        _command_deliver->frame_id = px4_application::UavCommand::LOCAL;    //控制坐标系选择
        _command_deliver->x = this->takeoff_position.x;    //x的指令信息
        _command_deliver->y = this->takeoff_position.y;    //y的指令信息
        _command_deliver->z = this->takeoff_position.z;    //z的指令信息
        // _command_deliver->yaw = 0;    //航向的指令信息
        _command_deliver->task_name = "TakeOff";    //任务名为TakeOff
        _uav_command_pub.publish(*_command_deliver);    //发送至飞控
        return ;
    }

    delete *_State;
    *_State = new Mission;    //切换到任务状态
}

TakeOff::TakeOff()
{
    ros::NodeHandle nh("~");
    nh.param<bool>("take_off/id", this->takeoff_id, true);
    nh.param<double>("take_off/x", this->takeoff_absolute_position_param.x, 0.0);
    nh.param<double>("take_off/y", this->takeoff_absolute_position_param.y, 0.0);
    nh.param<double>("take_off/z", this->takeoff_absolute_position_param.z, 1.0);
    nh.param<double>("take_off/h", this->takeoff_relative_height_param, 1.0);

    std::cout << "[ Take off ]" << std::endl;
}

TakeOff::~TakeOff()
{
    std::cout << "Takeoff state transition..." << std::endl;
}

void Mission::FlatTargetCallback(const px4_application::FlatTarget::ConstPtr& _msg)
{
    this->flat_target = *_msg;
}

/**
* @name         void Mission::Run(const StatusSubscriber& _current_info,
                                    const ros::Publisher& _uav_command_pub,
                                     px4_application::UavCommand* _command_deliver,
                                      States** _State)
* @brief        任务接口（派生类）
* @param[in]    无人机与目标状态：_current_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL
*/

void Mission::Run(const StatusSubscriber& _current_info,
                    const ros::Publisher& _uav_command_pub,
                     px4_application::UavCommand* _command_deliver,
                      States** _State)
{
    switch(this->own_id)
    {
        /*1号无人机任务*/
        case OtherSubscriber::uav_1 + 1:
        {
            if(_current_info.uav_status.state.mode != "OFFBOARD")
                _command_deliver->z = _current_info.uav_status.local_position.z;    //设定高度

    static int switch_num = 0;
    double enu_yaw = tf::getYaw(_current_info.tf_door_status.pose.orientation) + 1.57;
    if(_current_info.door_status.update)
    {
        
        double tan_yaw = tan(enu_yaw);
        double sin_yaw = sin(enu_yaw);
        double cos_yaw = cos(enu_yaw);
        
        double a = tan_yaw; // 门框中心点向外延伸的直线斜率
        double b = _current_info.tf_door_status.pose.position.y - a * _current_info.tf_door_status.pose.position.x; // 门框中心点向外延伸的直线截距
        double c = _current_info.uav_status.local_position.y + _current_info.uav_status.local_position.x / a; // 门框中心点向外延伸的直线垂直的直线的截距

        double nearest_x = (a * c - a * b) / (a * a + 1); // 当前位置到门框中心点向外延伸的直线的最近点
        double nearest_y = (a * a * c + b) / (a * a + 1);

        // if(_current_info.uav_status.state.mode != "OFFBOARD")
        //     this->route.col(0) = TypeTransform::RosMsg2Eigen(_current_info.uav_status.local_position);
        // else
        //     this->route.col(0) = TypeTransform::RosMsg2Eigen(this->flat_target.position);

        // wapoints为无人机距离门框中心点向外延伸直线的最近的点的前面3m和6m
        this->route.col(1) = Eigen::Vector3d(nearest_x + 3.0 * cos_yaw, nearest_y + 3.0 * sin_yaw, _current_info.tf_door_status.pose.position.z);
        this->route.col(2) = Eigen::Vector3d(nearest_x + 6.0 * cos_yaw, nearest_y + 6.0 * sin_yaw, _current_info.tf_door_status.pose.position.z);
    }
    else if(switch_num == 0 && abs(_current_info.uav_status.local_position.x - _current_info.tf_door_after_status.pose.position.x) < 0.5 &&
             abs(_current_info.uav_status.local_position.y - _current_info.tf_door_after_status.pose.position.y) < 0.5 && // 通过门框后的点，改变下一个航迹点
              _current_info.tf_door_after_status.pose.position.x != _current_info.tf_door_before_status.pose.position.x && // 初始未更新则不改变
               _current_info.tf_door_after_status.pose.position.y != _current_info.tf_door_before_status.pose.position.y)
    {
        // if(_current_info.uav_status.state.mode != "OFFBOARD")
        //     this->route.col(0) = TypeTransform::RosMsg2Eigen(_current_info.uav_status.local_position);
        // else
        //     this->route.col(0) = TypeTransform::RosMsg2Eigen(this->flat_target.position);

        this->route.col(1) = this->ref_door_pose.col(this->door_index);
        this->door_index++;
        if(this->door_index >= this->ref_door_pose.cols())
            this->door_index = 0;
        this->route.col(2) = this->ref_door_pose.col(this->door_index);
        switch_num++;
    }
    // if(_current_info.uav_status.state.mode != "OFFBOARD") // 初始点为当前位置
    //    this->route.col(0) = TypeTransform::RosMsg2Eigen(_current_info.uav_status.local_position);
    // else
    //     this->route.col(0) = TypeTransform::RosMsg2Eigen(this->flat_target.position);
    this->route.col(0) = TypeTransform::RosMsg2Eigen(_current_info.uav_status.local_position);

    if(abs(_current_info.uav_status.local_position.x - _current_info.tf_door_after_status.pose.position.x) > 0.5 ||
             abs(_current_info.uav_status.local_position.y - _current_info.tf_door_after_status.pose.position.y) > 0.5)
        switch_num = 0;

    Eigen::Matrix3d iS, fS; // 初始和终止状态pva
    iS.setZero(), fS.setZero();

    iS.col(0) << this->route.leftCols<1>();
    // iS.col(1) << TypeTransform::RosMsg2Eigen(_current_info.uav_status.local_velocity);
    // iS.col(2) << TypeTransform::RosMsg2Eigen(_current_info.uav_status.local_acceleration);

    // iS.col(0) << TypeTransform::RosMsg2Eigen(this->flat_target.position); // p
    iS.col(1) << TypeTransform::RosMsg2Eigen(this->flat_target.velocity); // v
    iS.col(2) << TypeTransform::RosMsg2Eigen(this->flat_target.acceleration); // a
    fS.col(0) << this->route.rightCols<1>();
    
    // 8字形轨迹测试
    /*this->route.resize(3, 13);
    this->route.col(0) = TypeTransform::RosMsg2Eigen(_current_info.uav_status.local_position);
    this->route.col(1) = Eigen::Vector3d(5.0, 5.0, 2.0);
    this->route.col(2) = Eigen::Vector3d(5.0, -5.0, 2.5);
    this->route.col(3) = Eigen::Vector3d(-5.0, 5.0, 3.0);
    this->route.col(4) = Eigen::Vector3d(-5.0, -5.0, 3.5);

    this->route.col(5) = Eigen::Vector3d(5.0, 5.0, 4.0);
    this->route.col(6) = Eigen::Vector3d(5.0, -5.0, 4.5);
    this->route.col(7) = Eigen::Vector3d(-5.0, 5.0, 5.0);
    this->route.col(8) = Eigen::Vector3d(-5.0, -5.0, 5.5);

    this->route.col(9) = Eigen::Vector3d(5.0, 5.0, 6.0);
    this->route.col(10) = Eigen::Vector3d(5.0, -5.0, 6.5);
    this->route.col(11) = Eigen::Vector3d(-5.0, 5.0, 7.0);
    this->route.col(12) = Eigen::Vector3d(-5.0, -5.0, 7.5);
    iS.col(0) << this->route.leftCols<1>();
    iS.col(1) << Eigen::Vector3d(0.0, 0.0, 0.0);
    iS.col(2) << Eigen::Vector3d(0.0, 0.0, 0.0);
    fS.col(0) << this->route.rightCols<1>();*/

    // 圆形轨迹测试
    /*this->route.resize(3, 25);
    this->route.col(0) = TypeTransform::RosMsg2Eigen(_current_info.uav_status.local_position);
    for(int i = 15, j = 1; i <= 360; i += 15, j++)
        this->route.col(j) = Eigen::Vector3d(5.0 + -5.0 * cos(i / 57.3), -5.0 * sin(i / 57.3), 3.0);
    // this->route.col(1) = Eigen::Vector3d(1.465, -3.535, 3.0);
    // this->route.col(2) = Eigen::Vector3d(5.0, -5.0, 3.0);
    // this->route.col(3) = Eigen::Vector3d(8.535, -3.535, 3.0);
    // this->route.col(4) = Eigen::Vector3d(10.0, 0.0, 3.0);
    // this->route.col(5) = Eigen::Vector3d(8.535, 3.535, 3.0);
    // this->route.col(6) = Eigen::Vector3d(5.0, 5.0, 3.0);
    // this->route.col(7) = Eigen::Vector3d(1.465, 3.535, 3.0);
    // this->route.col(8) = Eigen::Vector3d(0.0, 0.0, 3.0);

    iS.col(0) << this->route.leftCols<1>();
    iS.col(1) << Eigen::Vector3d(0.0, 0.0, 0.0);
    iS.col(2) << Eigen::Vector3d(0.0, 0.0, 0.0);
    fS.col(0) << this->route.rightCols<1>();*/

    this->jerk_opt.reset(iS, fS, this->route.cols() - 1);
    this->jerk_opt.generate(this->route.block(0, 1, 3, this->route.cols() - 2), this->jerk_opt.allocateTime(this->route, this->vel, this->acc));
    this->jerk_opt.getTraj(this->min_jerk_traj);

    this->cycles++;
    ros::Time curr_time = ros::Time::now();
    if(_current_info.door_status.update || switch_num == 1 || _current_info.uav_status.state.mode != "OFFBOARD") // 重新生成跟踪轨迹
    {
        // 发送给traj_publisher模块生成轨迹
        Eigen::VectorXd jerk_durs = this->min_jerk_traj.getDurations();
        px4_application::BoundaryConditions ref_traj;
        for(int i = 0; i < this->min_jerk_traj.getPieceNum(); i++)
        {
            px4_application::BoundaryCondition bd;
            bd.duration = jerk_durs[i];
            
            TypeTransform::Eigen2RosMsg(this->min_jerk_traj.getJuncPos(i), bd.p0);
            TypeTransform::Eigen2RosMsg(this->min_jerk_traj.getJuncVel(i), bd.v0);
            TypeTransform::Eigen2RosMsg(this->min_jerk_traj.getJuncAcc(i), bd.a0);
            
            TypeTransform::Eigen2RosMsg(this->min_jerk_traj.getJuncPos(i + 1), bd.pt);
            TypeTransform::Eigen2RosMsg(this->min_jerk_traj.getJuncVel(i + 1), bd.vt);
            TypeTransform::Eigen2RosMsg(this->min_jerk_traj.getJuncAcc(i + 1), bd.at);
            
            ref_traj.bound_conds.push_back(bd);
        }

        this->traj_duration = this->min_jerk_traj.getTotalDuration();
        ref_traj.header.stamp = ros::Time::now();
        
        if(this->cycles >= 5) // 至少100ms才发布一次
        {
            this->ref_traj_pub.publish(ref_traj);
            this->cycles = 0;
        }
        
        switch_num++;
    }
    
    // 轨迹显示
    // nav_msgs::Path rviz_min_jerk_path;
    // TrajectoryUtils<min_jerk::Trajectory>::TrajectorySample(this->min_jerk_traj, 0.1, rviz_min_jerk_path);
    // this->min_jerk_traj_visualization.PublishMessage(rviz_min_jerk_path);
    
    // 加速度显示
    geometry_msgs::Vector3 acc;
    acc.x = _current_info.uav_status.local_acceleration.x;
    acc.y = _current_info.uav_status.local_acceleration.y;
    acc.z = _current_info.uav_status.local_acceleration.z;
    acc_visualization.PublishMessage(acc);
    
    // 航点显示
    geometry_msgs::PoseArray rviz_waypoints;
    /*for(int i = 0; i < this->route.cols(); i++)
    {
        geometry_msgs::Pose quat_pos;
        quat_pos.position.x = this->route(0, i);
        quat_pos.position.y = this->route(1, i);
        quat_pos.position.z = this->route(2, i);
        rviz_waypoints.poses.push_back(quat_pos);
        
    }*/
    // 门框三点显示
    geometry_msgs::Pose quat_pos;
    quat_pos.orientation = _current_info.tf_door_before_status.pose.orientation;
    quat_pos.position = _current_info.tf_door_before_status.pose.position;
    rviz_waypoints.poses.push_back(quat_pos);

    quat_pos.orientation = _current_info.tf_door_status.pose.orientation;
    quat_pos.position = _current_info.tf_door_status.pose.position;
    rviz_waypoints.poses.push_back(quat_pos);

    quat_pos.orientation = _current_info.tf_door_after_status.pose.orientation;
    quat_pos.position = _current_info.tf_door_after_status.pose.position;
    rviz_waypoints.poses.push_back(quat_pos);

    rviz_waypoints.header.stamp = ros::Time::now();
    rviz_waypoints.header.frame_id = "world";
    this->waypoints_visualization.PublishMessage(rviz_waypoints);



    // minimum snap轨迹生成
    /*Eigen::Matrix<double, 3, 4> iSS, fSS; // 初始和终止状态pvaj
    iSS << iS, Eigen::MatrixXd::Zero(3, 1);
    fSS << fS, Eigen::MatrixXd::Zero(3, 1);
    this->snap_opt.reset(iSS, fSS, this->route.cols() - 1);
    this->snap_opt.generate(this->route.block(0, 1, 3, this->route.cols() - 2), this->snap_opt.allocateTime(this->route, this->vel, this->acc));
    this->snap_opt.getTraj(this->min_snap_traj);


    nav_msgs::Path rviz_min_snap_path;
    TrajectoryUtils<min_snap::Trajectory>::TrajectorySample(this->min_snap_traj, 0.1, rviz_min_snap_path);
    this->min_snap_traj_visualization.PublishMessage(rviz_min_snap_path);*/




            _command_deliver->header.stamp = ros::Time::now();
            _command_deliver->command_type = px4_application::UavCommand::TRAJECTORY; // 轨迹跟踪模式，控制指令由traj_publisher节点发布给uav_control
            _command_deliver->period = 0.05;
            _command_deliver->update = _current_info.door_status.update;
            _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_PZ;
            _command_deliver->yaw_id = px4_application::UavCommand::YAW;
            _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
            _command_deliver->x = _current_info.tf_door_status.pose.position.x;
            _command_deliver->y = _current_info.tf_door_status.pose.position.y;
            _command_deliver->z = _current_info.tf_door_status.pose.position.z;
            _command_deliver->yaw = enu_yaw;
            // _command_deliver->yaw = lock_yaw_;
            _command_deliver->task_name = "1_Mission";
            _uav_command_pub.publish(*_command_deliver);
    
            // /*任务指令*/
            /*Eigen::Vector3d target_pos = this->min_jerk_traj.getPos(0.5);
            Eigen::Vector3d target_vel = this->min_jerk_traj.getVel(0.5);


            if(_current_info.uav_status.state.mode != "OFFBOARD")
            {
                _command_deliver->x = _current_info.uav_status.local_position.x;
                _command_deliver->y = _current_info.uav_status.local_position.y;
                _command_deliver->z = _current_info.uav_status.local_position.z;    //设定高度
            }
            _command_deliver->header.stamp = ros::Time::now();
            _command_deliver->command_type = px4_application::UavCommand::NORMAL;
            _command_deliver->period = 0.05;
            _command_deliver->update = true;
            _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_PZ;
            _command_deliver->yaw_id = px4_application::UavCommand::YAW;
            _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
            // _command_deliver->x = target_pos[0];
            // _command_deliver->y = target_pos[1];
            // _command_deliver->z = target_pos[2];
            _command_deliver->yaw = enu_yaw;
            _command_deliver->task_name = "1_Mission";
            _uav_command_pub.publish(*_command_deliver);*/
            return ;
        }
        break;
        /*2号无人机任务*/
        case OtherSubscriber::uav_2 + 1:
        {
            if(_current_info.uav_status.state.mode != "OFFBOARD")
            {
                _command_deliver->z = _current_info.uav_status.local_position.z;    //设定高度
                _command_deliver->yaw = _current_info.uav_status.attitude_angle.z; //设定航向
            }
            
            double leader_yaw = total_info.uav_status[OtherSubscriber::uav_1].attitude_angle.z;
            if(total_info.uav_status[OtherSubscriber::uav_1].attitude_angle.z == 0)
                leader_yaw = _command_deliver->yaw;
            double tan_yaw = tan(leader_yaw);
            double sin_yaw = sin(leader_yaw);
            double cos_yaw = cos(leader_yaw);

                // 加速度显示
            geometry_msgs::Vector3 acc;
            acc.x = _current_info.uav_status.local_acceleration.x;
            acc.y = _current_info.uav_status.local_acceleration.y;
            acc.z = _current_info.uav_status.local_acceleration.z;
            acc_visualization.PublishMessage(acc);

            // 通信编队，保持在3m后
            // this->leader_status.position.x = total_info.uav_status[OtherSubscriber::uav_1].local_position.x + 3.0 - 3.0 * cos_yaw; // +3.0为1号机相对于2号机的初始摆放偏差
            // this->leader_status.position.y = total_info.uav_status[OtherSubscriber::uav_1].local_position.y - 3.0 * sin_yaw;
            // this->leader_status.position.z = total_info.uav_status[OtherSubscriber::uav_1].local_position.z;
            // this->leader_status.velocity.x = total_info.uav_status[OtherSubscriber::uav_1].local_velocity.x;
            // this->leader_status.velocity.y = total_info.uav_status[OtherSubscriber::uav_1].local_velocity.y;
            // this->leader_status.velocity.z = total_info.uav_status[OtherSubscriber::uav_1].local_velocity.z;
            // 视觉编队
            if(_current_info.drone_status.update)
            {
                this->leader_status.position.x = _current_info.tf_drone_status.pose.position.x - 3.0 * cos_yaw;
                this->leader_status.position.y = _current_info.tf_drone_status.pose.position.y - 3.0 * sin_yaw;
                this->leader_status.position.z = _current_info.tf_drone_status.pose.position.z;
            }
            this->leader_status.yaw = leader_yaw;
            this->leader_status_pub.publish(leader_status);
            
            /*任务指令*/
            _command_deliver->header.stamp = ros::Time::now();
            _command_deliver->command_type = px4_application::UavCommand::TRAJECTORY; // 轨迹跟踪模式，控制指令由form_publisher节点发布给uav_control
            _command_deliver->period = 0.05;
            _command_deliver->update = false;
            _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_PZ;
            _command_deliver->yaw_id = px4_application::UavCommand::YAW;
            _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
            _command_deliver->x = _current_info.tf_drone_status.pose.position.x;
            _command_deliver->y = _current_info.tf_drone_status.pose.position.y;
            _command_deliver->z = _current_info.tf_drone_status.pose.position.z;
            _command_deliver->yaw = this->leader_status.yaw;
            _command_deliver->task_name = "2_Mission";
            _uav_command_pub.publish(*_command_deliver);
            
            // _command_deliver->header.stamp = ros::Time::now();
            // _command_deliver->command_type = px4_application::UavCommand::NORMAL; // 轨迹跟踪模式，控制指令由form_publisher节点发布给uav_control
            // _command_deliver->period = 0.05;
            // _command_deliver->update = false;
            // _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_PZ;
            // _command_deliver->yaw_id = px4_application::UavCommand::YAW;
            // _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
            // _command_deliver->x = leader_status.position.x;
            // _command_deliver->y = leader_status.position.y;
            // _command_deliver->z = leader_status.position.z;
            // _command_deliver->yaw = leader_status.yaw;
            // _command_deliver->task_name = "2_Mission";
            // _uav_command_pub.publish(*_command_deliver);
            return ;
        }
        break;
        default: break;
    }

    // delete *_State; 
    // *_State = new ReturnHome;
}

Mission::Mission() : // min_jerk_traj_visualization(nh_private, "trajectory_visualization/min_jerk"),
                    // min_snap_traj_visualization(nh_private, "trajectory_visualization/min_snap"),
                       waypoints_visualization(nh_private, "waypoints_visualization"),
                        acc_visualization(nh_private, "acc_visualization")
{
    ros::NodeHandle nh("~");
    nh.param<int>("uav_id", this->own_id, 0);

    nh.param<double>("contraint/vel_max", this->vel, 2.0);
    nh.param<double>("contraint/acc_max", this->acc, 2.0);
    std::cout << "trajectory max vel: " << this->vel << std::endl;
    std::cout << "trajectory max acc: " << this->acc << std::endl;

    if(this->nh_private.getNamespace() == "/uav1")
    {
        this->ref_traj_pub = this->nh_private.advertise<px4_application::BoundaryConditions>("reference/boundary_conds/min_jerk", 10);
        this->flat_target_sub = this->nh_private.subscribe<px4_application::FlatTarget>("reference/flat_setpoint",
                                                                                         1,
                                                                                          &Mission::FlatTargetCallback,
                                                                                           this,
                                                                                            ros::TransportHints().tcpNoDelay());
    }
    // this->ref_door_pose.resize(3, 4);
    // // this->ref_door_pose.col(0) << Eigen::Vector3d(9.2, 2.3, 1.5);
    // // this->ref_door_pose.col(1) << Eigen::Vector3d(12.0, 9.7, 1.5);
    // // this->ref_door_pose.col(2) << Eigen::Vector3d(2.1, 14.5, 1.5);
    // // this->ref_door_pose.col(3) << Eigen::Vector3d(-0.9, 4.4, 1.5);
    // this->ref_door_pose.col(0) << Eigen::Vector3d(11.1, 0.7, 1.5); // 实物要修改！
    // this->ref_door_pose.col(1) << Eigen::Vector3d(13.4, 13.7, 1.5);
    // this->ref_door_pose.col(2) << Eigen::Vector3d(3.3, 15.3, 1.5);
    // this->ref_door_pose.col(3) << Eigen::Vector3d(0.4, 4.9, 1.5);
    int door_nums = 3;
    nh.param<int>("door_ref_info/nums", door_nums, 4);
    this->ref_door_pose.resize(3, door_nums);
    this->ref_door_pose.setZero();
    for(int i = 0; i < door_nums; i++)
    {
        geometry_msgs::Vector3 ref_pose;
        std::string param_name = "door_ref_info/door" + std::to_string(i);
        nh.param<double>(param_name + "/x", ref_pose.x, 0.0); // 相对于领航无人机位置点的参考门框位置
        nh.param<double>(param_name + "/y", ref_pose.y, 0.0);
        nh.param<double>(param_name + "/z", ref_pose.z, 0.0);
        this->ref_door_pose.col(i) << TypeTransform::RosMsg2Eigen(ref_pose);
    }
    std::cout << "-------------------------" << std::endl;
    std::cout << "number of reference door : " << door_nums << std::endl;
    for(int i = 0; i < ref_door_pose.cols(); i++)
    {
        std::cout << "door" << i << ": " << ref_door_pose.col(i)[0] << "[x], "
         << ref_door_pose.col(i)[1] << "[y], " << ref_door_pose.col(i)[2] << "[z]" << std::endl;
    }
    std::cout << "-------------------------" << std::endl;

    this->route.resize(3, 3);
    this->route.setZero();
    this->route.col(1) = this->ref_door_pose.col(0);
    this->route.col(2) = this->ref_door_pose.col(1);

    if(this->nh_private.getNamespace() == "/uav2")
    {
        this->leader_status_pub = this->nh_private.advertise<px4_application::LeaderStatus>("leader_status/local", 10);
    }

    std::cout << "[ Mission ]" << std::endl;
}

Mission::~Mission()
{
    std::cout << "Mission state transition..." << std::endl;
}

/**
* @name         void ReturnHome::Run(const StatusSubscriber& _current_info,
                                      const ros::Publisher& _uav_command_pub,
                                       px4_application::UavCommand* _command_deliver,
                                        States** _State)
* @brief        返航任务接口（派生类）
* @param[in]    无人机与目标状态：_current_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL
*/
void ReturnHome::Run(const StatusSubscriber& _current_info,
                      const ros::Publisher& _uav_command_pub,
                       px4_application::UavCommand* _command_deliver,
                        States** _State)
{
    if(!(abs(_current_info.uav_status.local_position.x - this->home_position.x) < this->reach_point_range.x &&
          abs(_current_info.uav_status.local_position.y - this->home_position.y) < this->reach_point_range.y &&
           abs(_current_info.uav_status.local_position.z - this->home_position.z) < this->reach_point_range.z))
    {
        _command_deliver->header.stamp = ros::Time::now();
        _command_deliver->command_type = px4_application::UavCommand::NORMAL;
        _command_deliver->period = 0.05;
        _command_deliver->update = true;
        _command_deliver->xyz_id = px4_application::UavCommand::PX_PY_PZ;
        _command_deliver->yaw_id = px4_application::UavCommand::YAW;
        _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
        _command_deliver->x = this->home_position.x;
        _command_deliver->y = this->home_position.y;
        _command_deliver->z = this->home_position.z;
        // _command_deliver->yaw = lock_yaw_;
        _command_deliver->task_name = "Return";
        _uav_command_pub.publish(*_command_deliver);
        return ;
    }

    delete *_State;
    *_State = new Landing;
}

ReturnHome::ReturnHome()
{
    ros::NodeHandle nh("~");
    nh.param<double>("home/x", this->home_position.x, 0.0);
    nh.param<double>("home/y", this->home_position.y, 0.0);
    nh.param<double>("home/z", this->home_position.z, 5.0);

    std::cout << "[ ReturnHome ]" << std::endl;
}

ReturnHome::~ReturnHome()
{
    std::cout << "ReturnHome state transition..." << std::endl;
}

/**
* @name         void Landing::Run(const StatusSubscriber& _current_info,
                                   const ros::Publisher& _uav_command_pub,
                                    px4_application::UavCommand* _command_deliver,
                                     States** _State)
* @brief        降落任务接口（派生类）
* @param[in]    无人机与目标状态：_current_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL
*/
void Landing::Run(const StatusSubscriber& _current_info,
                   const ros::Publisher& _uav_command_pub,
                    px4_application::UavCommand* _command_deliver,
                     States** _State)
{
    if(!(_current_info.uav_status.extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND && !_current_info.uav_status.state.armed))    //未检测到着陆与上锁
    {
        _command_deliver->header.stamp = ros::Time::now();
        _command_deliver->command_type = px4_application::UavCommand::NORMAL;
        _command_deliver->period = 0.05;
        _command_deliver->update = true;
        _command_deliver->xyz_id = px4_application::UavCommand::VX_VY_VZ;
        _command_deliver->yaw_id = px4_application::UavCommand::YAW;
        _command_deliver->frame_id = px4_application::UavCommand::LOCAL;
        _command_deliver->x = this->landing_pos_vel.x;
        _command_deliver->y = this->landing_pos_vel.y;
        _command_deliver->z = this->landing_pos_vel.z;
        // _command_deliver->yaw = lock_yaw_;
        _command_deliver->task_name = "Landing";
        _uav_command_pub.publish(*_command_deliver);
        return ;
    }
    
    delete *_State;
    *_State = new Finished;
}

Landing::Landing()
{
    ros::NodeHandle nh("~");
    nh.param<double>("landing/x", this->landing_pos_vel.x, 0.0);
    nh.param<double>("landing/y", this->landing_pos_vel.y, 0.0);
    nh.param<double>("landing/vz", this->landing_pos_vel.z, -0.5);

    std::cout << "[ Landing ]" << std::endl;
}

Landing::~Landing()
{
    std::cout << "Landing state transition..." << std::endl;
}

/**
* @name         void Finished::Run(const StatusSubscriber& _current_info,
                                    const ros::Publisher& _uav_command_pub,
                                     px4_application::UavCommand* _command_deliver,
                                      States** _State)
* @brief        完成任务接口（派生类）
* @param[in]    无人机与目标状态：_current_info
* @param[in]    指令发布器：_uav_command_pub
* @param[in]    指令信息：_command_deliver
* @param[in]    状态机：_State
* @param[out]   NULL
*/
void Finished::Run(const StatusSubscriber& _current_info,
                    const ros::Publisher& _uav_command_pub,
                     px4_application::UavCommand* _command_deliver,
                      States** _State)
{
    _command_deliver->header.stamp = ros::Time::now();
    _command_deliver->command_type = px4_application::UavCommand::NORMAL;
    _command_deliver->period = 0.05;
    _command_deliver->update = false;
    _command_deliver->xyz_id = px4_application::UavCommand::UX_UY_UZ;
    _command_deliver->task_name = "Finished";
    _uav_command_pub.publish(*_command_deliver);
    // return ;

    // delete *_State;
    // *_State = NULL;

    // *_State = new TakeOff;
}

Finished::Finished()
{

    std::cout << "[ Finished ]" << std::endl;
}

Finished::~Finished()
{
    std::cout << "Finished state transition..." << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "app_visual_formation");
    ros::NodeHandle nh;
    UavMission UavMission(nh, 1.0);
    // ros::spin();

    double ros_rate = 1.0;
    if(nh.getNamespace() == "/uav1")
    {
        ros::NodeHandle nh_temp("~");
        nh_temp.param<double>("leader_spin_rate/visual_formation", ros_rate, 1.0);
    }
    else
    {
        ros::NodeHandle nh_temp("~");
        nh_temp.param<double>("folower_spin_rate/visual_formation", ros_rate, 1.0);
    }

    std::cout << "-----------------------" << std::endl;
    std::cout << "ros rate: " << ros_rate << std::endl;
    ros::Rate loop(ros_rate);
    while(ros::ok())
    {
        ros::spinOnce();
        UavMission.LoopTaskWithoutVirtual();
        loop.sleep();
    }
    return 0;
}

