#include "swarm_control_topic.hpp"

// 真机：集群模式-无人机 或者 集群模式-无人车 构造函数
SwarmControl::SwarmControl(ros::NodeHandle &nh, Communication *communication, SwarmMode mode, int id, int swarm_num)
{
    this->communication_ = communication;
    is_simulation_ = false;
    init(nh, mode, swarm_num);
    // 【发布】集群控制指令
    this->swarm_command_pub_ = nh.advertise<prometheus_msgs::SwarmCommand>("/prometheus/swarm_command", 1000);
    // 【订阅】集群控制指令
    this->swarm_command_sub_ = nh.subscribe("/prometheus/swarm_command", 10, &SwarmControl::swarmCmdCb, this);
    if (mode == SwarmMode::ONLY_UAV)
    {
        std::cout << "真机：集群模式-无人机 开启" << std::endl;
        // 【发布】连接是否失效
        this->communication_state_pub_ = nh.advertise<std_msgs::Bool>("/uav" + std::to_string(id) + "/prometheus/communication_state", 10);
        // 【发布】所有无人机状态
        this->all_uav_state_pub_ = nh.advertise<prometheus_msgs::MultiUAVState>("/prometheus/all_uav_state", 1000);
    }
    else if (mode == SwarmMode::ONLY_UGV)
    {
        std::cout << "真机：集群模式-无人车 开启" << std::endl;
        // 【发布】连接是否失效
        this->communication_state_pub_ = nh.advertise<std_msgs::Bool>("/ugv" + std::to_string(id) + "/prometheus/communication_state", 10);
        // 【发布】所有无人车状态
        this->all_ugv_state_pub_ = nh.advertise<prometheus_msgs::MultiUGVState>("/prometheus/all_ugv_state", 1000);
    }
    else
    {
        std::cout << "集群模式错误： " << (int)mode << std::endl;
        return;
    }
}

// 真机：集群模式-机车协同 构造函数
SwarmControl::SwarmControl(ros::NodeHandle &nh, Communication *communication, SwarmMode mode, RobotType type, int id, int swarm_uav_num, int swarm_ugv_num)
{
    if (mode != SwarmMode::UAV_AND_UGV)
    {
        std::cout << "集群模式错误： " << (int)mode << std::endl;
        return;
    }
    this->communication_ = communication;
    is_simulation_ = false;
    init(nh, mode, swarm_uav_num, swarm_ugv_num);

    // 【发布】集群控制指令
    this->swarm_command_pub_ = nh.advertise<prometheus_msgs::SwarmCommand>("/prometheus/swarm_command", 1000);
    // 该通信节点运行的机器类型  无人机或者无人车
    // 运行在无人机上
    if (type == RobotType::ROBOT_TYPE_UAV)
    {
        // 【发布】连接是否失效
        this->communication_state_pub_ = nh.advertise<std_msgs::Bool>("/uav" + std::to_string(id) + "/prometheus/communication_state", 10);
    }
    // 运行在无人车上
    else if (type == RobotType::ROBOT_TYPE_UGV)
    {
        // 【发布】连接是否失效
        this->communication_state_pub_ = nh.advertise<std_msgs::Bool>("/ugv" + std::to_string(id) + "/prometheus/communication_state", 10);
    }
    // 【发布】所有无人机状态
    this->all_uav_state_pub_ = nh.advertise<prometheus_msgs::MultiUAVState>("/prometheus/all_uav_state", 1000);
    // 【订阅】集群控制指令
    this->swarm_command_sub_ = nh.subscribe("/prometheus/swarm_command", 10, &SwarmControl::swarmCmdCb, this);
    // 【发布】所有无人车状态
    this->all_ugv_state_pub_ = nh.advertise<prometheus_msgs::MultiUGVState>("/prometheus/all_ugv_state", 1000);
}

// 仿真：无人机或者无人车集群、机车协同
SwarmControl::SwarmControl(ros::NodeHandle &nh, Communication *communication, SwarmMode mode, RobotType type, int swarm_uav_num, int swarm_ugv_num)
{
    if (type != RobotType::ROBOT_TYPE_SIMULATION)
    {
        std::cout << "参数错误" << std::endl;
        return;
    }
    this->communication_ = communication;
    is_simulation_ = true;
    if (mode == SwarmMode::ONLY_UAV)
    {
        std::cout << "仿真：集群模式-无人机 开启" << std::endl;
        init(nh, mode, swarm_uav_num);
        for (int i = 1; i <= swarm_uav_num; i++)
        {
            // 连接是否失效
            ros::Publisher simulation_communication_state = nh.advertise<std_msgs::Bool>("/uav" + std::to_string(i) + "/prometheus/communication_state", 10);
            simulation_communication_state_pub.push_back(simulation_communication_state);
        }

        // 【发布】所有无人机状态
        this->all_uav_state_pub_ = nh.advertise<prometheus_msgs::MultiUAVState>("/prometheus/all_uav_state", 1000);
        // 【发布】集群控制指令
        this->swarm_command_pub_ = nh.advertise<prometheus_msgs::SwarmCommand>("/prometheus/swarm_command", 1000);
        // 【订阅】集群控制指令
        this->swarm_command_sub_ = nh.subscribe("/prometheus/swarm_command", 10, &SwarmControl::swarmCmdCb, this);
    }
    else if (mode == SwarmMode::ONLY_UGV)
    {
        std::cout << "仿真：集群模式-无人车 开启" << std::endl;
        init(nh, mode, swarm_ugv_num);
        for (int i = 1; i <= swarm_ugv_num; i++)
        {
            // 连接是否失效
            ros::Publisher simulation_communication_state = nh.advertise<std_msgs::Bool>("/ugv" + std::to_string(i) + "/prometheus/communication_state", 10);
            simulation_communication_state_pub.push_back(simulation_communication_state);
        }

        // 【发布】所有无人车状态
        this->all_ugv_state_pub_ = nh.advertise<prometheus_msgs::MultiUGVState>("/prometheus/all_ugv_state", 1000);
        // 【发布】集群控制指令
        this->swarm_command_pub_ = nh.advertise<prometheus_msgs::SwarmCommand>("/prometheus/swarm_command", 1000);
        // 【订阅】集群控制指令
        this->swarm_command_sub_ = nh.subscribe("/prometheus/swarm_command", 10, &SwarmControl::swarmCmdCb, this);
    }
    else if (mode == SwarmMode::UAV_AND_UGV)
    {
        std::cout << "仿真：集群模式-机车协同 开启" << std::endl;
        init(nh, mode, swarm_uav_num, swarm_ugv_num);
        for (int i = 1; i <= swarm_uav_num; i++)
        {
            // 连接是否失效
            ros::Publisher simulation_communication_state = nh.advertise<std_msgs::Bool>("/uav" + std::to_string(i) + "/prometheus/communication_state", 10);
            simulation_communication_state_pub.push_back(simulation_communication_state);
        }
        for (int i = 1; i <= swarm_ugv_num; i++)
        {
            // 连接是否失效
            ros::Publisher simulation_communication_state = nh.advertise<std_msgs::Bool>("/ugv" + std::to_string(i) + "/prometheus/communication_state", 10);
            simulation_communication_state_pub.push_back(simulation_communication_state);
        }

        // 【发布】所有无人机状态
        this->all_uav_state_pub_ = nh.advertise<prometheus_msgs::MultiUAVState>("/prometheus/all_uav_state", 1000);
        // 【发布】集群控制指令
        this->swarm_command_pub_ = nh.advertise<prometheus_msgs::SwarmCommand>("/prometheus/swarm_command", 1000);
        // 【订阅】集群控制指令
        this->swarm_command_sub_ = nh.subscribe("/prometheus/swarm_command", 10, &SwarmControl::swarmCmdCb, this);
        // 【发布】所有无人车状态
        this->all_ugv_state_pub_ = nh.advertise<prometheus_msgs::MultiUGVState>("/prometheus/all_ugv_state", 1000);
    }
}

SwarmControl::~SwarmControl()
{
    // delete this->communication_;
    // this->communication_ = nullptr;
}

void SwarmControl::init(ros::NodeHandle &nh, SwarmMode mode, int swarm_num)
{
    nh.param<std::string>("ground_station_ip", udp_ip, "127.0.0.1");
    nh.param<std::string>("multicast_udp_ip", multicast_udp_ip, "224.0.0.88");

    if (mode == SwarmMode::ONLY_UAV)
    {
        for (int i = 1; i <= swarm_num; ++i)
        {
            struct UAVState uav_state;
            uav_state.uav_id = i;
            // uav_state.state = UAVState::State::unknown;
            uav_state.location_source = UAVState::LocationSource::MOCAP;
            uav_state.gps_status = 0;
            uav_state.mode = "";
            uav_state.connected = false;
            uav_state.armed = false;
            uav_state.odom_valid = false;
            uav_state.gps_num = 0;
            for (int j = 0; j < 3; j++)
            {
                uav_state.position[j] = 0;
                uav_state.velocity[j] = 0;
                uav_state.attitude[j] = 0;
                uav_state.attitude_rate[j] = 0;
            }
            uav_state.latitude = 0;
            uav_state.longitude = 0;
            uav_state.altitude = 0;

            uav_state.attitude_q.x = 0;
            uav_state.attitude_q.y = 0;
            uav_state.attitude_q.z = 0;
            uav_state.attitude_q.w = 0;
            uav_state.battery_state = 0;
            uav_state.battery_percetage = 0;
            this->multi_uav_state_.uav_state_all.push_back(uav_state);
        }
    }
    else if (mode == SwarmMode::ONLY_UGV)
    {
        for (int i = 1; i <= swarm_num; ++i)
        {
            struct UGVState ugv_state;
            ugv_state.ugv_id = i;
            ugv_state.secs = 0;
            ugv_state.nsecs = 0;
            ugv_state.battery = 0;
            for (int j = 0; j < 3; j++)
            {
                ugv_state.position[j] = 0;
                ugv_state.velocity[j] = 0;
                ugv_state.attitude[j] = 0;
            }
            ugv_state.attitude_q.x = 0;
            ugv_state.attitude_q.y = 0;
            ugv_state.attitude_q.z = 0;
            ugv_state.attitude_q.w = 0;
            this->multi_ugv_state_.ugv_state_all.push_back(ugv_state);
        }
    }
}

void SwarmControl::init(ros::NodeHandle &nh, SwarmMode mode, int swarm_uav_num, int swarm_ugv_num)
{
    nh.param<std::string>("ground_station_ip", udp_ip, "127.0.0.1");
    nh.param<std::string>("multicast_udp_ip", multicast_udp_ip, "224.0.0.88");

    for (int i = 1; i <= swarm_uav_num; ++i)
    {
        struct UAVState uav_state;
        uav_state.uav_id = i;
        // uav_state.state = UAVState::State::unknown;
        uav_state.location_source = UAVState::LocationSource::MOCAP;
        uav_state.gps_status = 0;
        uav_state.mode = "";
        uav_state.connected = false;
        uav_state.armed = false;
        uav_state.odom_valid = false;
        uav_state.gps_num = 0;
        for (int j = 0; j < 3; j++)
        {
            uav_state.position[j] = 0;
            uav_state.velocity[j] = 0;
            uav_state.attitude[j] = 0;
            uav_state.attitude_rate[j] = 0;
        }
        uav_state.latitude = 0;
        uav_state.longitude = 0;
        uav_state.altitude = 0;

        uav_state.attitude_q.x = 0;
        uav_state.attitude_q.y = 0;
        uav_state.attitude_q.z = 0;
        uav_state.attitude_q.w = 0;
        uav_state.battery_state = 0;
        uav_state.battery_percetage = 0;
        this->multi_uav_state_.uav_state_all.push_back(uav_state);
    }
    for (int i = 1; i <= swarm_ugv_num; ++i)
    {
        struct UGVState ugv_state;
        ugv_state.ugv_id = i;
        ugv_state.secs = 0;
        ugv_state.nsecs = 0;
        ugv_state.battery = 0;
        for (int j = 0; j < 3; j++)
        {
            ugv_state.position[j] = 0;
            ugv_state.velocity[j] = 0;
            ugv_state.attitude[j] = 0;
        }
        ugv_state.attitude_q.x = 0;
        ugv_state.attitude_q.y = 0;
        ugv_state.attitude_q.z = 0;
        ugv_state.attitude_q.w = 0;
        this->multi_ugv_state_.ugv_state_all.push_back(ugv_state);
    }
}

// 更新全部无人机数据
void SwarmControl::updateAllUAVState(struct UAVState uav_state)
{
    // 更新数据
    for (int i = 0; i < this->multi_uav_state_.uav_state_all.size(); i++)
    {
        if (this->multi_uav_state_.uav_state_all[i].uav_id == uav_state.uav_id)
        {
            this->multi_uav_state_.uav_state_all[i] = uav_state;
            break;
        }
    }
}

// 更新全部无人车数据
void SwarmControl::updateAllUGVState(struct UGVState ugv_state)
{
    // 更新数据
    for (int i = 0; i < this->multi_ugv_state_.ugv_state_all.size(); i++)
    {
        if (this->multi_ugv_state_.ugv_state_all[i].ugv_id == ugv_state.ugv_id)
        {
            this->multi_ugv_state_.ugv_state_all[i] = ugv_state;
            break;
        }
    }
}

// 【发布】集群控制指令
void SwarmControl::swarmCommandPub(struct SwarmCommand swarm_command)
{
    // 进行发布
    prometheus_msgs::SwarmCommand m_swarm_command;
    m_swarm_command.source = swarm_command.source;
    m_swarm_command.Swarm_CMD = swarm_command.Swarm_CMD;
    m_swarm_command.swarm_location_source = swarm_command.swarm_location_source;
    m_swarm_command.swarm_num = swarm_command.swarm_num;
    for (int i = 0; i < 2; i++)
    {
        m_swarm_command.leader_pos[i] = swarm_command.leader_pos[i];
        m_swarm_command.leader_vel[i] = swarm_command.leader_vel[i];
    }
    m_swarm_command.leader_pos[2] = swarm_command.leader_pos[2];

    m_swarm_command.swarm_size = swarm_command.swarm_size;
    m_swarm_command.swarm_shape = swarm_command.swarm_shape;
    m_swarm_command.target_area_x_min = swarm_command.target_area_x_min;
    m_swarm_command.target_area_y_min = swarm_command.target_area_y_min;
    m_swarm_command.target_area_x_max = swarm_command.target_area_x_max;
    m_swarm_command.target_area_y_max = swarm_command.target_area_y_max;
    for (int i = 0; i < 3; i++)
    {
        m_swarm_command.attack_target_pos[i] = swarm_command.attack_target_pos[i];
    };
    for (int i = 0; i < swarm_command.formation_poses.size(); i++)
    {
        geometry_msgs::Point point;
        point.x = swarm_command.formation_poses[i].x;
        point.y = swarm_command.formation_poses[i].y;
        point.z = swarm_command.formation_poses[i].z;
        m_swarm_command.formation_poses.push_back(point);
    }
    this->swarm_command_pub_.publish(m_swarm_command);
}

// 【发布】连接是否失效
void SwarmControl::communicationStatePub(bool communication)
{
    std_msgs::Bool communication_state;
    communication_state.data = communication;
    this->communication_state_pub_.publish(communication_state);
}

void SwarmControl::communicationStatePub(bool communication, int id)
{
    std_msgs::Bool communication_state;
    communication_state.data = communication;
    // this->communication_state_pub_.publish(communication_state);
    this->simulation_communication_state_pub[id].publish(communication_state);
}

// 【发布】所有无人机状态
void SwarmControl::allUAVStatePub(struct MultiUAVState m_multi_uav_state)
{
    prometheus_msgs::MultiUAVState multi_uav_state;
    multi_uav_state.uav_num = 0;

    for (auto it = m_multi_uav_state.uav_state_all.begin(); it != m_multi_uav_state.uav_state_all.end(); it++)
    {
        prometheus_msgs::UAVState uav_state;
        uav_state.uav_id = (*it).uav_id;
        // uav_state.state = (*it).state;
        uav_state.mode = (*it).mode;
        uav_state.connected = (*it).connected;
        uav_state.armed = (*it).armed;
        uav_state.odom_valid = (*it).odom_valid;
        uav_state.location_source = (*it).location_source;
        uav_state.gps_status = (*it).gps_status;
        uav_state.gps_num = (*it).gps_num;
        for (int i = 0; i < 3; i++)
        {
            uav_state.position[i] = (*it).position[i];
            uav_state.velocity[i] = (*it).velocity[i];
            uav_state.attitude[i] = (*it).attitude[i];
            uav_state.attitude_rate[i] = (*it).attitude_rate[i];
        };
        uav_state.latitude = (*it).latitude;
        uav_state.longitude = (*it).longitude;
        uav_state.altitude = (*it).altitude;

        uav_state.attitude_q.x = (*it).attitude_q.x;
        uav_state.attitude_q.y = (*it).attitude_q.y;
        uav_state.attitude_q.z = (*it).attitude_q.z;
        uav_state.attitude_q.w = (*it).attitude_q.w;
        uav_state.battery_state = (*it).battery_state;
        uav_state.battery_percetage = (*it).battery_percetage;

        multi_uav_state.uav_num++;
        multi_uav_state.uav_state_all.push_back(uav_state);
    }

    // 发布话题
    this->all_uav_state_pub_.publish(multi_uav_state);
}

// 【发布】所有无人车状态
void SwarmControl::allUGVStatePub(struct MultiUGVState m_multi_ugv_state)
{
    prometheus_msgs::MultiUGVState multi_ugv_state;
    multi_ugv_state.swarm_num_ugv = 0;

    for (auto it = m_multi_ugv_state.ugv_state_all.begin(); it != m_multi_ugv_state.ugv_state_all.end(); it++)
    {
        prometheus_msgs::UGVState ugv_state;
        ugv_state.ugv_id = (*it).ugv_id;
        ugv_state.battery = (*it).battery;
        for (int i = 0; i < 3; i++)
        {
            ugv_state.position[i] = (*it).position[i];
            ugv_state.velocity[i] = (*it).velocity[i];
            ugv_state.attitude[i] = (*it).attitude[i];
        };
        ugv_state.attitude_q.x = (*it).attitude_q.x;
        ugv_state.attitude_q.y = (*it).attitude_q.y;
        ugv_state.attitude_q.z = (*it).attitude_q.z;
        ugv_state.attitude_q.w = (*it).attitude_q.w;

        multi_ugv_state.swarm_num_ugv++;
        multi_ugv_state.ugv_state_all.push_back(ugv_state);
    }

    // 发布话题
    this->all_ugv_state_pub_.publish(multi_ugv_state);
}

void SwarmControl::swarmCmdCb(const prometheus_msgs::SwarmCommand::ConstPtr &msg)
{
    struct SwarmCommand swarm_command;
    swarm_command.source = msg->source;
    swarm_command.Swarm_CMD = msg->Swarm_CMD;
    swarm_command.swarm_location_source = msg->swarm_location_source;
    swarm_command.swarm_num = msg->swarm_num;
    for (int i = 0; i < 2; i++)
    {
        swarm_command.leader_pos[i] = msg->leader_pos[i];
        swarm_command.leader_vel[i] = msg->leader_vel[i];
    }
    swarm_command.leader_pos[2] = msg->leader_pos[2];

    swarm_command.swarm_size = msg->swarm_size;
    swarm_command.swarm_shape = msg->swarm_shape;
    swarm_command.target_area_x_min = msg->target_area_x_min;
    swarm_command.target_area_y_min = msg->target_area_y_min;
    swarm_command.target_area_x_max = msg->target_area_x_max;
    swarm_command.target_area_y_max = msg->target_area_y_max;
    for (int i = 0; i < 3; i++)
    {
        swarm_command.attack_target_pos[i] = msg->attack_target_pos[i];
    };
    for (int i = 0; i < swarm_command.formation_poses.size(); i++)
    {
        struct Point point;
        point.x = msg->formation_poses[i].x;
        point.y = msg->formation_poses[i].y;
        point.z = msg->formation_poses[i].z;
        swarm_command.formation_poses.push_back(point);
    }
    this->communication_->sendMsgByUdp(this->communication_->encodeMsg(Send_Mode::UDP, swarm_command), multicast_udp_ip);
}
