
// 要求:
// 1,不用写成类,用普通的ros文件 main函数写就行
// 2,配套的是P450,outdoor(GPS),目前可考虑仅支持仿真,不考虑真实情况
// 3,根据agent_num参数来确定发布多少个话题(位置控制 XYZ_POS): "/uav" + std::to_string(this->agent_id_) + "/prometheus/command"
// 4,简易阵型即可,可以使用我们之前写好的支持任意个飞机的阵型h文件
// 5,在本程序中可以尝试加入等待避障或者相关简易避障策略
// 6,本程序和 swarm_command这个自定义消息无关

// 大致效果
// roslaunch prometheus_gazebo sitl_outdoor_4uav.launch
// roslaunch prometheus_uav_control uav_control_main_outdoor_4uav.launch

// roslaunch prometheus_demo formation_control.launch

// 确定启动程序没问题,遥控器一键解锁,一键切换至hover_control,遥控器正常1控多
// 切换至command_control,飞机悬停等待本程序的指令
// 飞机可以切换3个左右阵型,实现机动
// 降落
// 本程序相关教学:
// 1, 多个sdf的编写,端口如何与px4对应
// 2, uav_control中相关的集群接口
// 3, ...
#include "swarm_control.h"

SwarmControl::SwarmControl(ros::NodeHandle &nh)
{
    // 【参数】编号，从1开始编号
    nh.param<int>("agent_id", this->agent_id_, 1);
    nh.param<float>("takeoff_height", this->takeoff_height_, 1.0);
    nh.param<float>("setmode_timeout", this->setmode_timeout_, 2.0);
    //无人机集群控制模式切换为stop,无人机相应的模式;1: LAND 2: RETURN
    nh.param<int>("stop_mode", this->stop_mode_,1);

    this->node_name_ = "[swarm_control_uav_" + std::to_string(this->agent_id_) + "]";

    //【订阅】本机状态信息（来自本机估计节点）
    this->uav_state_sub_ = nh.subscribe<prometheus_msgs::UAVState>("/uav" + std::to_string(this->agent_id_) + "/prometheus/state",
                                                                   1,
                                                                   &SwarmControl::uavStateCb, this);

    //【订阅】集群控制指令(来自地面站 -> 通信节点)
    this->swarm_command_sub_ = nh.subscribe<prometheus_msgs::SwarmCommand>("/prometheus/swarm_command",
                                                                           1,
                                                                           &SwarmControl::swarmCommandCb, this);

    //【发布】底层控制指令(-> uav_control.cpp)
    this->uav_cmd_pub_ = nh.advertise<prometheus_msgs::UAVCommand>("/uav" + std::to_string(this->agent_id_) + "/prometheus/command", 1);

    //【发布】mavros接口调用指令(-> uav_control.cpp)
    this->mavros_interface_pub_ = nh.advertise<prometheus_msgs::MavrosInterface>("/uav" + std::to_string(this->agent_id_) + "/prometheus/mavros_interface", 1);

    //【定时器】打印调试
    this->debug_timer_ = nh.createTimer(ros::Duration(10.0), &SwarmControl::debugCb, this);

    this->vel_ctrl_param_.APF_R = 1.3; // 不能大于阵型间隔，不然无法收敛
    this->vel_ctrl_param_.APF_r = 0.4;
    this->vel_ctrl_param_.k_aij = 0.1;
    this->vel_ctrl_param_.k_p = 1.5;
    this->vel_ctrl_param_.k_gamma = 1.5;

    this->last_process_time_ = ros::Time::now();
    this->mainloop_rate_ = 10.0;

    this->swarm_command_.header.stamp = ros::Time::now();
    this->swarm_command_.Swarm_CMD = prometheus_msgs::SwarmCommand::Ready;
    this->swarm_command_.swarm_size = 1.0;
    this->swarm_command_.swarm_shape = prometheus_msgs::SwarmCommand::One_column;
    this->swarm_command_.target_area_x_min = 0.0;
    this->swarm_command_.target_area_y_min = 0.0;
    this->swarm_command_.target_area_x_max = 0.0;
    this->swarm_command_.target_area_y_max = 0.0;
    this->swarm_command_.attack_target_pos[0] = 0.0;
    this->swarm_command_.attack_target_pos[1] = 0.0;
    this->swarm_command_.attack_target_pos[2] = 0.0;

    printfParam();

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout << setprecision(SWARM_CONTROL_BOSHEN97_NUM_POINT);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout << GREEN << this->node_name_ << " init! " << TAIL << endl;
}

void SwarmControl::mainLoop()
{
    ros::Time now_time = ros::Time::now();
    // 本函数的实际执行频率，mainloop_rate = 100，即100Hz
    if ((now_time - this->last_process_time_).toSec() < (1.0 / this->mainloop_rate_))
    {
        return;
    }
    this->last_process_time_ = now_time;
    switch (this->swarm_command_.Swarm_CMD)
    {

    // 【Ready】 准备
    case prometheus_msgs::SwarmCommand::Ready:

        break;

    // 【Init】 初始化
    case prometheus_msgs::SwarmCommand::Init:

        // check UAV state, todo...
        // 检查解锁状态、GPS状态、模式状态等等
        // 打印提示消息
        
        break;
    // 【Start】 起飞
    case prometheus_msgs::SwarmCommand::Start:

        if(this->swarm_command_last_.Swarm_CMD == this->swarm_command_.Swarm_CMD)
        {
            break;
        }

        this->uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Move;
        this->uav_command_.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
        this->uav_command_.position_ref[0] = 0;
        this->uav_command_.position_ref[1] = 0;
        this->uav_command_.position_ref[2] = this->takeoff_height_; 
        this->uav_cmd_pub_.publish(this->uav_command_);
        sleep(1);
        this->mavros_interface_.type = prometheus_msgs::MavrosInterface::SET_MODE;
        this->mavros_interface_.mode = "OFFBOARD";
        this->mavros_interface_pub_.publish(this->mavros_interface_);
        sleep(1);
        this->mavros_interface_.type = prometheus_msgs::MavrosInterface::ARMING;
        this->mavros_interface_.arming = true;
        this->mavros_interface_pub_.publish(this->mavros_interface_);
        sleep(2);

        break;

    // 【Hold】 悬停
    case prometheus_msgs::SwarmCommand::Hold:

        this->uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
        this->uav_cmd_pub_.publish(this->uav_command_);

        break;

    // 【Stop】 降落,停止
    case prometheus_msgs::SwarmCommand::Stop:

        if(this->swarm_command_last_.Swarm_CMD == this->swarm_command_.Swarm_CMD)
        {
            break;
        }

        if(this->stop_mode_ == 1)
        {
            this->mavros_interface_.type = prometheus_msgs::MavrosInterface::SET_MODE;
            this->mavros_interface_.mode = "AUTO.LAND";
            this->mavros_interface_pub_.publish(this->mavros_interface_);
        }
        else if(this->stop_mode_ == 2)
        {
            this->mavros_interface_.type = prometheus_msgs::MavrosInterface::SET_MODE;
            this->mavros_interface_.mode = "AUTO.RTL";
            this->mavros_interface_pub_.publish(this->mavros_interface_);
        }
        
        break;

    // 【Formation】 班组跟随，无人机无人车以编队飞行，或者直接跟随离它最近的士兵
    // 需要士兵位置信息（或者使用视觉信息跟随）
    case prometheus_msgs::SwarmCommand::Formation:

        //阵型（位置控制）
        formationWithPositionControl();

        //阵型（速度控制）
        //formation_with_velocity_control();

        break;

    // 【Follow】 班组跟随，无人机无人车以编队飞行，或者直接跟随离它最近的士兵
    // 需要士兵位置信息（或者使用视觉信息跟随）
    case prometheus_msgs::SwarmCommand::Follow:
        // todo ...
        break;

    // 【Search】 对目标区域进行搜索
    case prometheus_msgs::SwarmCommand::Search:
        // todo ...
        break;

    // 【Attack】 对目标位置进行打击
    case prometheus_msgs::SwarmCommand::Attack:
        // todo ...
        break;
    }
    this->swarm_command_last_ = this->swarm_command_;
}

void SwarmControl::swarmCommandCb(const prometheus_msgs::SwarmCommand::ConstPtr &msg)
{
    this->swarm_command_ = *msg;

    // 指令预处理
    switch (this->swarm_command_.Swarm_CMD)
    {
    // 【Init】 准备
    case prometheus_msgs::SwarmCommand::Ready:
        this->mainloop_rate_ = 10.0;
        break;
    // 【Init】 准备
    case prometheus_msgs::SwarmCommand::Init:
        this->mainloop_rate_ = 10.0;
        break;
    // 【Start】 起飞
    case prometheus_msgs::SwarmCommand::Start:
        this->mainloop_rate_ = 10.0;
        break;

    // 【Hold】 悬停
    case prometheus_msgs::SwarmCommand::Hold:
        this->mainloop_rate_ = 10.0;
        break;

    // 【Stop】 降落,停止
    case prometheus_msgs::SwarmCommand::Stop:
        this->mainloop_rate_ = 10.0;
        break;

    // 【Formation】
    // 确定阵型量，follow_in_formation将发布agent指令
    case prometheus_msgs::SwarmCommand::Formation:
        this->mainloop_rate_ = 20.0;
        // 虚拟领队位置
        this->virtual_leader_pos_[0] = this->swarm_command_.leader_pos[0];
        this->virtual_leader_pos_[1] = this->swarm_command_.leader_pos[1];
        this->virtual_leader_pos_[2] = this->swarm_command_.leader_pos[2];
        this->virtual_leader_vel_[0] = this->swarm_command_.leader_vel[0];
        this->virtual_leader_vel_[1] = this->swarm_command_.leader_vel[1];

        //队形切换
        this->formation_separation_ = FormationUtils::getFormationSeparation(this->swarm_command_.swarm_shape, this->swarm_command_.swarm_size, this->swarm_command_.swarm_num);

    }
}

void SwarmControl::uavStateCb(const prometheus_msgs::UAVState::ConstPtr &msg)
{
    this->uav_state_ = *msg;
    this->uav_pos_ = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
    this->uav_vel_ = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
}

/*void SwarmControl::formationWithVelocityControl()
{
    　//平面阵型，xy控制速度，z轴高度定高
    　//一阶积分器分布式编队控制算法，可追踪时变轨迹，此处采用双向环的拓扑结构，且仅部分无人机可收到地面站发来的虚拟领队消息
    　//参考文献：Multi-Vehicle consensus with a time-varying reference state 公式(11) - (12)
    　//目前该算法有一定控制偏差，可能是由于参数选取不是最佳导致的

    float yita = 1 / ((float)this->uav_num_ * this->vel_ctrl_param_.k_aij + this->vel_ctrl_param_.k_p);

    Eigen::Vector2d vel_des;

    //收敛项
    vel_des[0] = yita * this->vel_ctrl_param_.k_p * (this->virtual_leader_vel_[0] - this->vel_ctrl_param_.k_gamma * (this->uav_pos_[0] - this->virtual_leader_pos_[0] - this->formation_separation_(this->agent_id_ - 1, 0)));
    vel_des[1] = yita * this->vel_ctrl_param_.k_p * (this->virtual_leader_vel_[1] - this->vel_ctrl_param_.k_gamma * (this->uav_pos_[1] - this->virtual_leader_pos_[1] - this->formation_separation_(this->agent_id_ - 1, 1)));

    //协同项
    for (int i = 1; i <= this->uav_num_; i++)
    {
        if (i == this->agent_id_)
        {
            continue;
        }

        vel_des[0] += -yita * this->vel_ctrl_param_.k_aij * (this->all_uav_vel_[i][0] - this->vel_ctrl_param_.k_gamma * ((this->uav_pos_[0] - this->all_uav_pos_[i][0]) - (this->formation_separation_(this->agent_id_ - 1, 0) - this->formation_separation_(i - 1, 0))));
        vel_des[1] += -yita * this->vel_ctrl_param_.k_aij * (this->all_uav_vel_[i][1] - this->vel_ctrl_param_.k_gamma * ((this->uav_pos_[1] - this->all_uav_pos_[i][1]) - (this->formation_separation_(this->agent_id_ - 1, 1) - this->formation_separation_(i - 1, 1))));
    }

    Eigen::Vector3d dv;
    //计算APF项（APF还是只能作为一个局部策略）
    for (int i = 1; i <= this->uav_num_; i++)
    {
        if (i == this->agent_id_)
        {
            continue;
        }

        float distance = (this->uav_pos_ - this->all_uav_vel_[i]).norm();

        if (distance > this->vel_ctrl_param_.APF_R)
        {
            dv.setZero();
        }
        else if (distance > this->vel_ctrl_param_.APF_r)
        {
            dv = 4 * (this->vel_ctrl_param_.APF_R * this->vel_ctrl_param_.APF_R - this->vel_ctrl_param_.APF_r * this->vel_ctrl_param_.APF_r) * (distance * distance - this->vel_ctrl_param_.APF_R * this->vel_ctrl_param_.APF_R) / pow(distance * distance - this->vel_ctrl_param_.APF_r * this->vel_ctrl_param_.APF_r, 3) * (this->uav_pos_ - this->all_uav_vel_[i]);
        }
        else
        {
            dv.setZero();
        }

        vel_des[0] -= 0.2 * dv[0];
        vel_des[1] -= 0.2 * dv[1];
    }

    this->uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Move;
    this->uav_command_.Move_mode = prometheus_msgs::UAVCommand::XY_VEL_Z_POS;
    this->uav_command_.velocity_ref[0] = vel_des[0];
    this->uav_command_.velocity_ref[1] = vel_des[1];
    this->uav_command_.position_ref[2] = 0.0 + this->formation_separation_(this->agent_id_ - 1, 2);
    this->uav_command_.yaw_ref = 0.0;
    this->uav_cmd_pub_.publish(this->uav_command_);
}*/

void SwarmControl::formationWithPositionControl()
{
    this->uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Move;
    this->uav_command_.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
    this->uav_command_.position_ref[0] = this->virtual_leader_pos_[0] + this->formation_separation_(this->agent_id_ - 1, 0);
    this->uav_command_.position_ref[1] = this->virtual_leader_pos_[1] + this->formation_separation_(this->agent_id_ - 1, 1);
    this->uav_command_.position_ref[2] = this->virtual_leader_pos_[2] + this->formation_separation_(this->agent_id_ - 1, 2);
    this->uav_command_.yaw_ref = 0.0;
    this->uav_cmd_pub_.publish(this->uav_command_);
}

void SwarmControl::debugCb(const ros::TimerEvent &e)
{
    cout << GREEN << ">>>>>>>>>>>>>>>>>>>>>>>> Swarm Control Node  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << TAIL << endl;

    cout << GREEN << "[ UAV ]"
         << "agent_id : " << this->agent_id_ << TAIL << endl;
    cout << GREEN << "uav_pos [X Y Z] : " << this->uav_state_.position[0] << " [ m ] " << this->uav_state_.position[1] << " [ m ] " << this->uav_state_.position[2] << " [ m ] " << TAIL << endl;
    cout << GREEN << "uav_pos [X Y Z] : " << this->uav_state_.velocity[0] << " [m/s] " << this->uav_state_.velocity[1] << " [m/s] " << this->uav_state_.velocity[2] << " [m/s] " << TAIL << endl;

    if (this->swarm_command_.Swarm_CMD == prometheus_msgs::SwarmCommand::Init)
    {
        cout << GREEN << "SwarmCommand : [ Init ]" << TAIL << endl;
    }
    else if (this->swarm_command_.Swarm_CMD == prometheus_msgs::SwarmCommand::Start)
    {
        cout << GREEN << "SwarmCommand : [ Start ]" << TAIL << endl;
    }
    else if (this->swarm_command_.Swarm_CMD == prometheus_msgs::SwarmCommand::Hold)
    {
        cout << GREEN << "SwarmCommand : [ Hold ]" << TAIL << endl;
    }
    else if (this->swarm_command_.Swarm_CMD == prometheus_msgs::SwarmCommand::Stop)
    {
        cout << GREEN << "SwarmCommand : [ Stop ]" << TAIL << endl;
    }
    else if (this->swarm_command_.Swarm_CMD == prometheus_msgs::SwarmCommand::Formation)
    {
        cout << GREEN << "SwarmCommand : [ Formation ]" << TAIL << endl;
    }
    else if (this->swarm_command_.Swarm_CMD == prometheus_msgs::SwarmCommand::Follow)
    {
        cout << GREEN << "SwarmCommand : [ Follow ]" << TAIL << endl;
    }
    else if (this->swarm_command_.Swarm_CMD == prometheus_msgs::SwarmCommand::Search)
    {
        cout << GREEN << "SwarmCommand : [ Search ]" << TAIL << endl;
    }
    else if (this->swarm_command_.Swarm_CMD == prometheus_msgs::SwarmCommand::Attack)
    {
        cout << GREEN << "SwarmCommand : [ Attack ]" << TAIL << endl;
    }
}

void SwarmControl::printfParam()
{
    cout << ">>>>>>>>>>>>>>>> Swarm Control Param <<<<<<<<<<<<<<<<" << endl;

    cout << "agent_id   : " << this->agent_id_ << endl;
}