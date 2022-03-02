#include "swarm_control.h"

SwarmControl::SwarmControl(ros::NodeHandle &nh)
{
    // 【参数】编号，从1开始编号
    nh.param<int>("agent_id", agent_id, 1);

    int uav_id = agent_id;
    node_name = "[swarm_control_uav_" + std::to_string(uav_id) + "]";

    //【订阅】本机状态信息（来自本机估计节点）
    uav_state_sub = nh.subscribe<prometheus_msgs::UAVState>("/uav" + std::to_string(uav_id) + "/prometheus/state",
                                                            1,
                                                            &SwarmControl::uav_state_cb, this);

    //【订阅】所有状态信息（来自其他无人机 -> 通信节点）
    all_uav_state_sub = nh.subscribe<prometheus_msgs::MultiUAVState>("/prometheus/all_uav_state",
                                                                     1,
                                                                     &SwarmControl::all_uav_state_cb, this);

    //【订阅】集群控制指令(来自地面站 -> 通信节点)
    swarm_command_sub = nh.subscribe<prometheus_msgs::SwarmCommand>("/prometheus/swarm_command",
                                                                    1,
                                                                    &SwarmControl::swarm_command_cb, this);

    //【发布】底层控制指令(-> uav_control.cpp)
    uav_cmd_pub = nh.advertise<prometheus_msgs::UAVCommand>("/uav" + std::to_string(uav_id) + "/prometheus/command", 1);

    //【定时器】打印调试
    debug_timer = nh.createTimer(ros::Duration(10.0), &SwarmControl::debug_cb, this);

    vel_ctrl_param.APF_R = 1.3; // 不能大于阵型间隔，不然无法收敛
    vel_ctrl_param.APF_r = 0.4;
    vel_ctrl_param.k_aij = 0.1;
    vel_ctrl_param.k_p = 1.5;
    vel_ctrl_param.k_gamma = 1.5;

    last_process_time = ros::Time::now();
    mainloop_rate = 10.0;

    swarm_command.header.stamp = ros::Time::now();
    swarm_command.Swarm_CMD = prometheus_msgs::SwarmCommand::Init;
    swarm_command.swarm_size = 1.0;
    swarm_command.swarm_shape = prometheus_msgs::SwarmCommand::One_column;
    swarm_command.target_area_x_min = 0.0;
    swarm_command.target_area_y_min = 0.0;
    swarm_command.target_area_x_max = 0.0;
    swarm_command.target_area_y_max = 0.0;
    swarm_command.attack_target_pos[0] = 0.0;
    swarm_command.attack_target_pos[1] = 0.0;
    swarm_command.attack_target_pos[2] = 0.0;

    for (int i = 1; i <= MAX_AGENT_NUM; i++)
    {
        all_uav_pos[i] = Eigen::Vector3d(100.0, 100.0, 100.0);
        all_uav_vel[i] = Eigen::Vector3d(0.0, 0.0, 0.0);
    }

    printf_param();

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout << setprecision(NUM_POINT);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout << GREEN << node_name << " init! " << TAIL << endl;
}

void SwarmControl::mainloop()
{
    ros::Time now_time = ros::Time::now();
    // 本函数的实际执行频率，mainloop_rate = 100，即100Hz
    if ((now_time - last_process_time).toSec() < (1.0 / mainloop_rate))
    {
        return;
    }
    last_process_time = now_time;

    switch (swarm_command.Swarm_CMD)
    {
    // 【Init】 准备
    case prometheus_msgs::SwarmCommand::Init:

        // check UAV state, todo...
        // 检查解锁状态、GPS状态、模式状态等等
        // 打印提示消息
        checkUAVState();

        break;
    // 【Start】 起飞
    case prometheus_msgs::SwarmCommand::Start:

        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
        uav_cmd_pub.publish(uav_command);

        break;

    // 【Hold】 悬停
    case prometheus_msgs::SwarmCommand::Hold:

        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
        uav_cmd_pub.publish(uav_command);

        break;

    // 【Stop】 降落,停止
    case prometheus_msgs::SwarmCommand::Stop:

        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Land;
        uav_cmd_pub.publish(uav_command);

        break;

    // 【Follow】 班组跟随，无人机无人车以编队飞行，或者直接跟随离它最近的士兵
    // 需要士兵位置信息（或者使用视觉信息跟随）
    case prometheus_msgs::SwarmCommand::Formation:

        // 判断是否碰撞 1 hold, 2 land
        // 这个仅仅作为防护措施，不会避开
        collision_flag = check_collision();
        if (collision_flag == 1)
        {
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
            uav_cmd_pub.publish(uav_command);
        }
        else if (collision_flag == 2)
        {
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
            uav_cmd_pub.publish(uav_command);
        }
        else
        {
            // 无危险，则继续跟随阵型（位置控制）
            formation_with_position_control();

            // 无危险，则继续跟随阵型（速度控制）
            // formation_with_velocity_control();
        }

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
    swarm_command_last = swarm_command;
}

int SwarmControl::check_collision()
{
    // 规则： ID越小，优先通过权限越高
    // 最坏情况： 1号飞机可以飞行，其他飞机全部卡住悬停
    // 遍历ID小于自己的agent
    for (int i = 1; i < agent_id; i++)
    {
        float dis_to_nei = (uav_pos - all_uav_pos[i]).norm();

        if (dis_to_nei < safe_dis_to_hold)
        {
            cout << RED << node_name << " is too near to uav" << i << ", distance is " << dis_to_nei << " [m] !" << TAIL << endl;
            return 1;
        }
        else if (dis_to_nei < safe_dis_to_land)
        {
            cout << RED << node_name << " is too near to uav" << i << ", distance is " << dis_to_nei << " [m] !" << TAIL << endl;
            return 2;
        }
    }

    // if(agent_id == 1)
    // {
    //     for(int i = 2; i < uav_num; i++)
    //     {
    //         float dis_to_nei = (uav_pos - all_uav_pos[i]).norm();

    //         if(dis_to_nei < safe_dis_to_land)
    //         {
    //             cout << RED << node_name <<" is too near to uav" << i << ", distance is "<<dis_to_nei<<" [m], STOP!"<< TAIL << endl;
    //             return 2;
    //         }
    //     }
    // }

    return 0;
}

void SwarmControl::swarm_command_cb(const prometheus_msgs::SwarmCommand::ConstPtr &msg)
{
    swarm_command = *msg;

    // 指令预处理
    switch (swarm_command.Swarm_CMD)
    {
    // 【Init】 准备
    case prometheus_msgs::SwarmCommand::Init:
        mainloop_rate = 10.0;
        break;
    // 【Start】 起飞
    case prometheus_msgs::SwarmCommand::Start:
        mainloop_rate = 10.0;
        break;

    // 【Hold】 悬停
    case prometheus_msgs::SwarmCommand::Hold:
        mainloop_rate = 10.0;
        break;

    // 【Stop】 降落,停止
    case prometheus_msgs::SwarmCommand::Stop:
        mainloop_rate = 10.0;
        break;

    // 【Formation】
    // 确定阵型量，follow_in_formation将发布agent指令
    case prometheus_msgs::SwarmCommand::Formation:
        // 弹性阵型，可伸缩阵型（指定点变为一个区域），todo
        mainloop_rate = 20.0;

        // 虚拟领队位置
        virtual_leader_pos[0] = swarm_command.leader_pos[0];
        virtual_leader_pos[1] = swarm_command.leader_pos[1];
        virtual_leader_vel[0] = swarm_command.leader_vel[0];
        virtual_leader_vel[1] = swarm_command.leader_vel[1];

        // 计算agnet期望位置（将阵型变成自动生成，不限制死固定数量，todo）
        // 阵型优化
        formation_separation = formation_utils::get_formation_separation(swarm_command.swarm_shape, swarm_command.swarm_size, uav_num);
        break;
    }
}

void SwarmControl::uav_state_cb(const prometheus_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;

    uav_pos = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
    uav_vel = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
}

void SwarmControl::all_uav_state_cb(const prometheus_msgs::MultiUAVState::ConstPtr &msg)
{
    uav_num = msg->uav_num;

    for (int i = 1; i <= uav_num; i++)
    {
        uav_states[i] = msg->uav_state_all[i];
        all_uav_pos[i] = Eigen::Vector3d(msg->uav_state_all[i].position[0], msg->uav_state_all[i].position[1], msg->uav_state_all[i].position[2]);
        all_uav_vel[i] = Eigen::Vector3d(msg->uav_state_all[i].velocity[0], msg->uav_state_all[i].velocity[1], msg->uav_state_all[i].velocity[2]);
    }
}

void SwarmControl::formation_with_velocity_control()
{
    //　平面阵型，xy控制速度，z轴高度定高
    //　一阶积分器分布式编队控制算法，可追踪时变轨迹，此处采用双向环的拓扑结构，且仅部分无人机可收到地面站发来的虚拟领队消息
    //　参考文献：Multi-Vehicle consensus with a time-varying reference state 公式(11) - (12)
    //　目前该算法有一定控制偏差，可能是由于参数选取不是最佳导致的

    float yita = 1 / ((float)uav_num * vel_ctrl_param.k_aij + vel_ctrl_param.k_p);

    Eigen::Vector2d vel_des;

    // 收敛项
    vel_des[0] = yita * vel_ctrl_param.k_p * (virtual_leader_vel[0] - vel_ctrl_param.k_gamma * (uav_pos[0] - virtual_leader_pos[0] - formation_separation(agent_id - 1, 0)));
    vel_des[1] = yita * vel_ctrl_param.k_p * (virtual_leader_vel[1] - vel_ctrl_param.k_gamma * (uav_pos[1] - virtual_leader_pos[1] - formation_separation(agent_id - 1, 1)));

    // 协同项
    for (int i = 1; i <= uav_num; i++)
    {
        if (i == agent_id)
        {
            continue;
        }

        vel_des[0] += -yita * vel_ctrl_param.k_aij * (all_uav_vel[i][0] - vel_ctrl_param.k_gamma * ((uav_pos[0] - all_uav_pos[i][0]) - (formation_separation(agent_id - 1, 0) - formation_separation(i - 1, 0))));
        vel_des[1] += -yita * vel_ctrl_param.k_aij * (all_uav_vel[i][1] - vel_ctrl_param.k_gamma * ((uav_pos[1] - all_uav_pos[i][1]) - (formation_separation(agent_id - 1, 1) - formation_separation(i - 1, 1))));
    }

    Eigen::Vector3d dv;
    // 计算APF项（APF还是只能作为一个局部策略）
    for (int i = 1; i <= uav_num; i++)
    {
        if (i == agent_id)
        {
            continue;
        }

        float distance = (uav_pos - all_uav_vel[i]).norm();

        if (distance > vel_ctrl_param.APF_R)
        {
            dv.setZero();
        }
        else if (distance > vel_ctrl_param.APF_r)
        {
            dv = 4 * (vel_ctrl_param.APF_R * vel_ctrl_param.APF_R - vel_ctrl_param.APF_r * vel_ctrl_param.APF_r) * (distance * distance - vel_ctrl_param.APF_R * vel_ctrl_param.APF_R) / pow(distance * distance - vel_ctrl_param.APF_r * vel_ctrl_param.APF_r, 3) * (uav_pos - all_uav_vel[i]);
        }
        else
        {
            dv.setZero();
        }

        vel_des[0] -= 0.2 * dv[0];
        vel_des[1] -= 0.2 * dv[1];
    }

    uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
    uav_command.Move_mode = prometheus_msgs::UAVCommand::XY_VEL_Z_POS;
    uav_command.velocity_ref[0] = vel_des[0];
    uav_command.velocity_ref[1] = vel_des[1];
    uav_command.position_ref[2] = 0.0 + formation_separation(agent_id - 1, 2);
    uav_command.yaw_ref = 0.0;
    uav_cmd_pub.publish(uav_command);
}

void SwarmControl::formation_with_position_control()
{
    uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
    uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
    uav_command.position_ref[0] = virtual_leader_pos[0] + formation_separation(agent_id - 1, 0);
    uav_command.position_ref[1] = virtual_leader_pos[1] + formation_separation(agent_id - 1, 1);
    uav_command.position_ref[2] = 0.0 + formation_separation(agent_id - 1, 2);
    uav_command.yaw_ref = 0.0;
    uav_cmd_pub.publish(uav_command);
}

void SwarmControl::debug_cb(const ros::TimerEvent &e)
{
    cout << GREEN << ">>>>>>>>>>>>>>>>>>>>>>>> Swarm Control Node  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << TAIL << endl;

    cout << GREEN << "[ UAV ]"
         << "agent_id : " << agent_id << TAIL << endl;
    cout << GREEN << "uav_pos [X Y Z] : " << uav_state.position[0] << " [ m ] " << uav_state.position[1] << " [ m ] " << uav_state.position[2] << " [ m ] " << TAIL << endl;
    cout << GREEN << "uav_pos [X Y Z] : " << uav_state.velocity[0] << " [m/s] " << uav_state.velocity[1] << " [m/s] " << uav_state.velocity[2] << " [m/s] " << TAIL << endl;

    if (swarm_command.Swarm_CMD == prometheus_msgs::SwarmCommand::Init)
    {
        cout << GREEN << "SwarmCommand : [ Init ]" << TAIL << endl;
    }
    else if (swarm_command.Swarm_CMD == prometheus_msgs::SwarmCommand::Start)
    {
        cout << GREEN << "SwarmCommand : [ Start ]" << TAIL << endl;
    }
    else if (swarm_command.Swarm_CMD == prometheus_msgs::SwarmCommand::Hold)
    {
        cout << GREEN << "SwarmCommand : [ Hold ]" << TAIL << endl;
    }
    else if (swarm_command.Swarm_CMD == prometheus_msgs::SwarmCommand::Stop)
    {
        cout << GREEN << "SwarmCommand : [ Stop ]" << TAIL << endl;
    }
    else if (swarm_command.Swarm_CMD == prometheus_msgs::SwarmCommand::Formation)
    {
        cout << GREEN << "SwarmCommand : [ Formation ]" << TAIL << endl;
    }
    else if (swarm_command.Swarm_CMD == prometheus_msgs::SwarmCommand::Follow)
    {
        cout << GREEN << "SwarmCommand : [ Follow ]" << TAIL << endl;
    }
    else if (swarm_command.Swarm_CMD == prometheus_msgs::SwarmCommand::Search)
    {
        cout << GREEN << "SwarmCommand : [ Search ]" << TAIL << endl;
    }
    else if (swarm_command.Swarm_CMD == prometheus_msgs::SwarmCommand::Attack)
    {
        cout << GREEN << "SwarmCommand : [ Attack ]" << TAIL << endl;
    }
}

void SwarmControl::printf_param()
{
    cout << ">>>>>>>>>>>>>>>> Swarm Control Param <<<<<<<<<<<<<<<<" << endl;

    cout << "agent_id   : " << agent_id << endl;
}

void SwarmControl::checkUAVState()
{
    // todo
}