#include "communication_bridge.hpp"
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
std::mutex g_m;
std::mutex g_uav_basic;
// boost::shared_mutex g_m;

CommunicationBridge::CommunicationBridge(ros::NodeHandle &nh) : Communication()
{
    // 是否仿真 1 为是  0为否
    nh.param<int>("is_simulation", this->is_simulation_, 1);
    // 集群数量  非集群模式值为0
    nh.param<int>("swarm_num", this->swarm_num_, 0);
    // 集群小车  非集群模式值为0
    nh.param<int>("swarm_ugv_num", this->swarm_ugv_num_, 0);

    // 集群模式下数据更新超时多久进行反馈
    nh.param<int>("swarm_data_update_timeout", this->swarm_data_update_timeout_, 5);

    nh.param<int>("udp_port", UDP_PORT, 8889);
    nh.param<int>("tcp_port", TCP_PORT, 55555);
    nh.param<int>("tcp_heartbeat_port", TCP_HEARTBEAT_PORT, 55556);
    // nh.param<int>("rviz_port", RVIZ_PORT, 8890);
    nh.param<int>("ROBOT_ID", ROBOT_ID, 1);
    nh.param<int>("uav_id", uav_id, 0);
    nh.param<int>("ugv_id", ugv_id, 0);
    nh.param<std::string>("ground_station_ip", udp_ip, "127.0.0.1");
    nh.param<std::string>("multicast_udp_ip", multicast_udp_ip, "224.0.0.88");
    nh.param<int>("try_connect_num", try_connect_num, 3);

    bool autoload;
    nh.param<bool>("autoload", autoload, false);
    if (autoload)
    {
        nh.param<std::string>("uav_control_start", OPENUAVBASIC, "");
        nh.param<std::string>("close_uav_control", CLOSEUAVBASIC, "");
        nh.param<std::string>("ugv_control_start", OPENUGVBASIC, "");
        nh.param<std::string>("close_ugv_control", CLOSEUGVBASIC, "");
        nh.param<std::string>("swarm_control_start", OPENSWARMCONTROL, "");
        nh.param<std::string>("close_swarm_control", CLOSESWARMCONTROL, "");
    }

    this->nh_ = nh;

    Communication::init(ROBOT_ID, UDP_PORT, TCP_PORT, TCP_HEARTBEAT_PORT);

    // thread_recCommunicationBridgeiver
    boost::thread recv_thd(&CommunicationBridge::serverFun, this);
    recv_thd.detach();        // 后台
    ros::Duration(1).sleep(); // wait

    // thread_receiver
    boost::thread recv_multicast_thd(&CommunicationBridge::multicastUdpFun, this);
    recv_multicast_thd.detach();
    ros::Duration(1).sleep(); // wait

    boost::thread to_ground_station_thd(&CommunicationBridge::toGroundStationFun, this);
    to_ground_station_thd.detach();
    ros::Duration(1).sleep(); // wait

    heartbeat_check_timer = nh.createTimer(ros::Duration(1.0), &CommunicationBridge::checkHeartbeatState, this);
}

CommunicationBridge::~CommunicationBridge()
{
    if (this->uav_basic_ != NULL)
        delete this->uav_basic_;
    if (this->ugv_basic_ != NULL)
        delete this->ugv_basic_;
    if (this->autonomous_landing_ != NULL)
        delete this->autonomous_landing_;
    if (this->object_tracking_ != NULL)
        delete this->object_tracking_;
    if (this->swarm_control_ != NULL)
        delete this->swarm_control_;
}

// TCP服务端
void CommunicationBridge::serverFun()
{
    int valread;
    if (waitConnectionFromGroundStation(TCP_PORT) < 0)
    {
        ROS_ERROR("[bridge_node]Socket recever creation error!");
        exit(EXIT_FAILURE);
    }

    while (true)
    {
        // 等待连接队列中抽取第一个连接，创建一个与s同类的新的套接口并返回句柄。
        if ((recv_sock = accept(server_fd, (struct sockaddr *)NULL, NULL)) < 0)
        {
            perror("accept");
            exit(EXIT_FAILURE);
        }

        // recv函数从TCP连接的另一端接收数据
        valread = recv(recv_sock, tcp_recv_buf, BUF_LEN, 0);

        if (valread <= 0)
        {
            ROS_ERROR("Received message length <= 0, maybe connection has lost");
            close(recv_sock);
            continue;
        }

        // std::lock_guard<std::mutex> lg(g_m);

        std::cout << "tcp valread: " << valread << std::endl;
        // char *ptr = tcp_recv_buf;
        pubMsg(decodeMsg(tcp_recv_buf, Send_Mode::TCP));
        close(recv_sock);
    }
}

void CommunicationBridge::recvData(struct UAVState uav_state)
{
    if (this->swarm_control_ != NULL)
    {
        // 融合到所有无人机状态然后发布话题
        this->swarm_control_->updateAllUAVState(uav_state);
        // 发布话题
        this->swarm_control_->allUAVStatePub(this->swarm_control_->getMultiUAVState());
    }
}
void CommunicationBridge::recvData(struct UAVCommand uav_cmd)
{
    // 非仿真情况 只有一个UAV
    if (this->is_simulation_ == 0)
    {
        if (this->uav_basic_ == NULL)
        {
            return;
        }
        this->uav_basic_->uavCmdPub(uav_cmd);
    }
    // 仿真情况下 可能存在多个UAV 找到对应ID进行发布对应的控制命令
    else
    {
        auto it = this->swarm_control_simulation_.find(recv_id);
        if (it != this->swarm_control_simulation_.end())
        {
            (*it).second->uavCmdPub(uav_cmd);
        }
    }
}
void CommunicationBridge::recvData(struct SwarmCommand swarm_command)
{
    if (this->swarm_control_ == NULL)
    {
        return;
    }
    // 发布话题
    this->swarm_control_->swarmCommandPub(swarm_command);
}
void CommunicationBridge::recvData(struct ConnectState connect_state)
{
    if (this->is_simulation_ == 0)
        return;
    if (!connect_state.state || connect_state.num < this->swarm_num_)
        // 触发降落信号
        this->swarm_control_->communicationStatePub(connect_state.state, connect_state.num);
}
void CommunicationBridge::recvData(struct GimbalControl gimbal_control)
{
    if (this->gimbal_basic_ == NULL)
    {
        return;
    }
    this->gimbal_basic_->gimbalControlPub(gimbal_control);
}
void CommunicationBridge::recvData(struct GimbalService gimbal_service)
{
    if (this->autonomous_landing_ == NULL)
    {
        return;
    }
    if (gimbal_service.service == gimbal_service.search)
        this->autonomous_landing_->gimbalSearchServer(gimbal_service.data);
    else if (gimbal_service.service == gimbal_service.record_video)
        this->autonomous_landing_->gimbalRecordVideoServer(gimbal_service.data);
    else if (gimbal_service.service == gimbal_service.track_mode)
        this->autonomous_landing_->gimbalTrackModeServer(gimbal_service.data);
}
void CommunicationBridge::recvData(struct GimbalParamSet param_set)
{
    if (this->autonomous_landing_ == NULL)
    {
        return;
    }
    this->autonomous_landing_->gimbalParamSetServer(param_set);
}
void CommunicationBridge::recvData(struct WindowPosition window_position)
{
    if (this->gimbal_basic_ == NULL)
    {
        return;
    }
    // 如果udp_msg数据不为空 则向udo端口发送数据。否则发布ros话题
    if (!window_position.udp_msg.empty())
    {
        std::cout << "udp_msg size :" << window_position.udp_msg.size() << std::endl;
        sendMsgByUdp(window_position.udp_msg.size(), window_position.udp_msg.c_str(), "127.0.0.1", SERV_PORT);
    }
    else
    {
        this->gimbal_basic_->gimbalWindowPositionPub(window_position);
    }
}
void CommunicationBridge::recvData(struct UGVCommand ugv_command)
{
    // 非仿真情况 只有一个UAV
    if (this->is_simulation_ == 0)
    {
        if (this->ugv_basic_ == NULL)
        {
            return;
        }
        this->ugv_basic_->ugvCmdPub(ugv_command);
    }
    // 仿真情况下 可能存在多个UAV 找到对应ID进行发布对应的控制命令
    else
    {
        auto it = this->swarm_ugv_control_simulation_.find(Communication::getRecvID() - swarm_num_);
        if (it != this->swarm_ugv_control_simulation_.end())
        {
            (*it).second->ugvCmdPub(ugv_command);
        }
    }
}
void CommunicationBridge::recvData(struct UGVState ugv_state)
{
    if (this->swarm_control_ != NULL && this->swarm_ugv_num_ != 0)
    {
        // 融合到所有无人车状态然后发布话题
        this->swarm_control_->updateAllUGVState(ugv_state);
        // 发布话题
        this->swarm_control_->allUGVStatePub(this->swarm_control_->getMultiUGVState());
    }
}
void CommunicationBridge::recvData(struct ImageData image_data)
{
    createImage(image_data);
}
void CommunicationBridge::recvData(struct UAVSetup uav_setup)
{
    if (this->uav_basic_ == NULL)
    {
        return;
    }
    this->uav_basic_->uavSetupPub(uav_setup);
}
void CommunicationBridge::recvData(struct ModeSelection mode_selection)
{
    modeSwitch(mode_selection);
}
void CommunicationBridge::recvData(struct ParamSettings param_settings)
{
    if (param_settings.params.size() == 0)
    {
        switch (param_settings.param_module)
        {
        case ParamSettings::ParamModule::UAVCONTROL:
            sendControlParam();
            break;
        case ParamSettings::ParamModule::UAVCOMMUNICATION:
            sendCommunicationParam();
            break;
        case ParamSettings::ParamModule::SWARMCONTROL:
            sendSwarmParam();
            break;
        case ParamSettings::ParamModule::UAVCOMMANDPUB:
            sendCommandPubParam();
            break;
        default:
            break;
        }
        return;
    }
    for (int i = 0; i < param_settings.params.size(); i++)
    {
        bool is = false;
        // 根据不同类型将string转为对应类型
        if (param_settings.params[i].type == param_settings.params[i].INT)
        {
            int value = atoi(param_settings.params[i].param_value.c_str());
            // this->nh_.setParam(param_settings.params[i].param_name,value);
            is = setParam(param_settings.params[i].param_name, value);
        }
        else if (param_settings.params[i].type == param_settings.params[i].FLOAT)
        {
            float value = atof(param_settings.params[i].param_value.c_str());
            // this->nh_.setParam(param_settings.params[i].param_name,value);
            is = setParam(param_settings.params[i].param_name, value);
        }
        else if (param_settings.params[i].type == param_settings.params[i].DOUBLE)
        {
            double value = stod(param_settings.params[i].param_value.c_str());
            // this->nh_.setParam(param_settings.params[i].param_name,value);
            is = setParam(param_settings.params[i].param_name, value);
        }
        else if (param_settings.params[i].type == param_settings.params[i].STRING)
        {
            // this->nh_.setParam(param_settings.params[i].param_name,param_settings.params[i].param_value);
            is = setParam(param_settings.params[i].param_name, param_settings.params[i].param_value);
        }
        else if (param_settings.params[i].type == param_settings.params[i].BOOLEAN)
        {
            bool value = param_settings.params[i].param_value == "true" ? true : false;
            is = setParam(param_settings.params[i].param_name, value);
        }
        // 反馈消息 表示、设置成功与否 textinfo
        std::string info = is ? "param settings success!" : "param settings failed!";
        sendTextInfo(is ? TextInfo::MessageTypeGrade::MTG_INFO : TextInfo::MessageTypeGrade::MTG_ERROR, info);
    }
    if (param_settings.param_module == ParamSettings::ParamModule::UAVCOMMUNICATION)
    {
        nh_.getParam("ground_station_ip", udp_ip);
        nh_.getParam("multicast_udp_ip", multicast_udp_ip);
        nh_.getParam("ROBOT_ID", ROBOT_ID);
        nh_.getParam("swarm_num", swarm_num_);
        nh_.getParam("is_simulation", is_simulation_);
        nh_.getParam("swarm_data_update_timeout", swarm_data_update_timeout_);

        bool autoload = false;
        nh_.getParam("autoload", autoload);
        if (autoload)
        {
            nh_.getParam("uav_control_start", OPENUAVBASIC);
            nh_.getParam("close_uav_control", CLOSEUAVBASIC);
            nh_.getParam("swarm_control_start", OPENSWARMCONTROL);
            nh_.getParam("close_swarm_control", CLOSESWARMCONTROL);
        }
        else
        {
            OPENUAVBASIC = "";
            CLOSEUAVBASIC = "";
            OPENSWARMCONTROL = "";
            CLOSESWARMCONTROL = "";
        }

        if (this->is_heartbeat_ready_ == false)
            this->is_heartbeat_ready_ = true;
    }
}
void CommunicationBridge::recvData(struct MultiBsplines multi_bsplines)
{
    if (this->ego_planner_ == NULL)
    {
        return;
    }
    this->ego_planner_->swarmTrajPub(multi_bsplines);
}
void CommunicationBridge::recvData(struct Bspline bspline)
{
    if (this->ego_planner_ == NULL)
    {
        return;
    }
    this->ego_planner_->oneTrajPub(bspline);
}
// 此处为 地面站-->机载端 机载端<->机载端
void CommunicationBridge::recvData(struct CustomDataSegment_1 custom_data_segment)
{
    // 自定义
}

void CommunicationBridge::recvData(struct Goal goal)
{
    if (this->ego_planner_ != NULL)
    {
        this->ego_planner_->goalPub(goal);
    }
}

// 根据协议中MSG_ID的值，将数据段数据转化为正确的结构体
void CommunicationBridge::pubMsg(int msg_id)
{
    switch (msg_id)
    {
    case MsgId::UAVSTATE:
        recvData(Communication::getUAVState());
        break;
    case MsgId::SWARMCOMMAND:
        recvData(Communication::getSwarmCommand());
        break;
    case MsgId::CONNECTSTATE:
        // 集群仿真下有效
        recvData(Communication::getConnectState());
        break;
    case MsgId::GIMBALCONTROL:
        recvData(Communication::getGimbalControl());
        break;
    case MsgId::GIMBALSERVICE:
        recvData(Communication::getGimbalService());
        break;
    case MsgId::GIMBALPARAMSET:
        recvData(Communication::getGimbalParamSet());
        break;
    case MsgId::WINDOWPOSITION:
        recvData(Communication::getWindowPosition());
        break;
    case MsgId::UGVCOMMAND:
        recvData(Communication::getUGVCommand());
        break;
    case MsgId::UGVSTATE:
        recvData(Communication::getUGVState());
        break;
    case MsgId::IMAGEDATA:
        recvData(Communication::getImageData());
        break;
    case MsgId::UAVCOMMAND:
        recvData(Communication::getUAVCommand());
        break;
    case MsgId::UAVSETUP:
        recvData(Communication::getUAVSetup());
        break;
    case MsgId::MODESELECTION:
        recvData(Communication::getModeSelection());
        break;
    case MsgId::PARAMSETTINGS:
        recvData(Communication::getParamSettings());
        break;
    case MsgId::BSPLINE:
        recvData(Communication::getBspline());
        break;
    case MsgId::MULTIBSPLINES:
        recvData(Communication::getMultiBsplines());
        break;
    case MsgId::CUSTOMDATASEGMENT_1:
        recvData(Communication::getCustomDataSegment_1());
        break;
    case MsgId::GOAL:
        recvData(Communication::getGoal());
        break;
    default:
        break;
    }
}

void CommunicationBridge::createImage(struct ImageData image_data)
{
    std::ofstream os(image_data.name);
    os << image_data.data;
    os.close();
    std::cout << "image_data" << std::endl;
}

void CommunicationBridge::modeSwitch(struct ModeSelection mode_selection)
{
    if (mode_selection.mode == ModeSelection::Mode::REBOOTNX)
    {
        system(REBOOTNXCMD);
    }
    else if (mode_selection.mode == ModeSelection::Mode::EXITNX)
    {
        system(EXITNXCMD);
    }

    if (mode_selection.use_mode == ModeSelection::UseMode::UM_CREATE)
    {
        createMode(mode_selection);
    }
    else if (mode_selection.use_mode == ModeSelection::UseMode::UM_DELETE)
    {
        deleteMode(mode_selection);
    }
}

void CommunicationBridge::createMode(struct ModeSelection mode_selection)
{
    if (mode_selection.mode == ModeSelection::Mode::UAVBASIC)
    {
        // 仿真模式 允许同一通信节点创建多个飞机的话题
        if (this->is_simulation_ == 1)
        {
            for (int i = 0; i < mode_selection.selectId.size(); i++)
            {
                // 判断是否已经存在
                if (!this->swarm_control_simulation_.empty())
                {
                    if (this->swarm_control_simulation_.find(mode_selection.selectId[i]) != this->swarm_control_simulation_.end())
                    {
                        sendTextInfo(TextInfo::MessageTypeGrade::MTG_INFO, "UAV" + to_string(mode_selection.selectId[i]) + " duplicate connections!!!");
                        continue;
                    }
                }

                // 创建并存入
                this->swarm_control_simulation_[mode_selection.selectId[i]] = new UAVBasic(this->nh_, mode_selection.selectId[i], (Communication *)this);
                // 如果id与通信节点相同则存入uav_basic_
                if (ROBOT_ID == mode_selection.selectId[i])
                {
                    if (this->uav_basic_ != NULL)
                    {
                        delete this->uav_basic_;
                        this->uav_basic_ = NULL;
                    }
                    this->uav_basic_ = this->swarm_control_simulation_[mode_selection.selectId[i]];

                    // 打开
                    system(OPENUAVBASIC.c_str());
                }
                sendTextInfo(TextInfo::MessageTypeGrade::MTG_INFO, "Simulation UAV" + to_string(mode_selection.selectId[i]) + " connection succeeded!!!");
            }
        }
        // 真机模式 同一通信节点只能创建一个飞机的话题
        else
        {
            for (int i = 0; i < mode_selection.selectId.size(); i++)
            {
                // 如果id与通信节点相同则存入uav_basic_
                if (mode_selection.selectId[i] == ROBOT_ID)
                {
                    this->is_heartbeat_ready_ = true;
                    if (this->uav_basic_ == NULL)
                    {
                        this->uav_basic_ = new UAVBasic(this->nh_, uav_id, (Communication *)this);
                        sendTextInfo(TextInfo::MessageTypeGrade::MTG_INFO, "UAV" + to_string(ROBOT_ID) + " connection succeeded!!!");

                        // 打开
                        system(OPENUAVBASIC.c_str());
                    }
                    else
                        sendTextInfo(TextInfo::MessageTypeGrade::MTG_INFO, "UAV" + to_string(ROBOT_ID) + " duplicate connections!!!");
                }
                else
                {
                    sendTextInfo(TextInfo::MessageTypeGrade::MTG_WARN, "Robot" + to_string(mode_selection.selectId[i]) + " connection failed, The ground station ID is inconsistent with the communication node ID.");
                    return;
                }
            }
        }
        this->is_heartbeat_ready_ = true;
        // 加载单机参数
        sendControlParam();
        // 加载通信模块参数
        sendCommunicationParam();
        // 加载轨迹控制参数
        sendCommandPubParam();
    }
    else if (mode_selection.mode == ModeSelection::Mode::UGVBASIC)
    {
        if (this->is_simulation_)
        {
            for (int i = 0; i < mode_selection.selectId.size(); i++)
            {
                //
                int id = mode_selection.selectId[i] - swarm_num_;
                if (id < 0)
                {
                    sendTextInfo(TextInfo::MessageTypeGrade::MTG_INFO, "Please check the parameters --- swarm_num");
                    return;
                }

                // 判断是否已经存在
                if (!this->swarm_ugv_control_simulation_.empty())
                {
                    if (this->swarm_ugv_control_simulation_.find(id) != this->swarm_ugv_control_simulation_.end())
                    {
                        sendTextInfo(TextInfo::MessageTypeGrade::MTG_INFO, "UGV" + to_string(id) + " duplicate connections!!!");
                        continue;
                    }
                }
                //

                this->swarm_ugv_control_simulation_[id] = new UGVBasic(this->nh_, id, (Communication *)this);
                // 创建并存入
                // this->swarm_ugv_control_simulation_[mode_selection.selectId[i]] = new UGVBasic(this->nh_, mode_selection.selectId[i], (Communication *)this);
                // 如果id与通信节点相同则存入uav_basic_
                if (ROBOT_ID == mode_selection.selectId[i])
                {
                    if (this->ugv_basic_ != NULL)
                    {
                        delete this->ugv_basic_;
                        this->ugv_basic_ = NULL;
                    }
                    this->ugv_basic_ = this->swarm_ugv_control_simulation_[id];

                    // 打开
                    system(OPENUGVBASIC.c_str());
                }
                sendTextInfo(TextInfo::MessageTypeGrade::MTG_INFO, "Simulation UGV" + to_string(id) + " connection succeeded!!!");
            }
        }
        else
        {
            for (int i = 0; i < mode_selection.selectId.size(); i++)
            {
                // 如果id与通信节点相同则存入uav_basic_
                if (mode_selection.selectId[i] == ROBOT_ID)
                {
                    this->is_heartbeat_ready_ = true;
                    if (this->ugv_basic_ == NULL)
                    {
                        this->ugv_basic_ = new UGVBasic(this->nh_, ugv_id, (Communication *)this);
                        sendTextInfo(TextInfo::MessageTypeGrade::MTG_INFO, "UGV" + to_string(ugv_id) + " connection succeeded!!!");

                        // 打开
                        system(OPENUGVBASIC.c_str());
                    }
                    else
                        sendTextInfo(TextInfo::MessageTypeGrade::MTG_INFO, "UGV" + to_string(ugv_id) + " duplicate connections!!!");
                }
                else
                {
                    sendTextInfo(TextInfo::MessageTypeGrade::MTG_WARN, "Robot" + to_string(mode_selection.selectId[i]) + " connection failed, The ground station ID is inconsistent with the communication node ID.");
                    return;
                }
            }
        }
        this->is_heartbeat_ready_ = true;
    }
    // 集群模式
    else if (mode_selection.mode == ModeSelection::Mode::SWARMCONTROL)
    {
        if (this->swarm_num_ != mode_selection.selectId.size() && this->swarm_ugv_num_ != mode_selection.selectId.size() && this->swarm_num_ + this->swarm_ugv_num_ != mode_selection.selectId.size())
        {
            sendTextInfo(TextInfo::MessageTypeGrade::MTG_WARN, "Switching mode failed, The number of ground stations is inconsistent with the number of communication nodes.");
        }
        else if (this->is_simulation_ != mode_selection.is_simulation)
        {
            sendTextInfo(TextInfo::MessageTypeGrade::MTG_WARN, "Switching mode failed, The ground station and communication node are simulation mode and real machine mode respectively.");
        }
        // 仿真模式
        else if (this->is_simulation_ == 1)
        {
            for (int i = 0; i < mode_selection.selectId.size(); i++)
            {
                // 按照顺序来判断 先无人机后无人车
                if (i < swarm_num_)
                {
                    if (this->swarm_control_simulation_.count(mode_selection.selectId[i]) == 0)
                    {
                        sendTextInfo(TextInfo::MessageTypeGrade::MTG_WARN, "Switching mode failed, UAV" + to_string(mode_selection.selectId[i]) + " non-existent, " + "please check whether it is connected.");
                        return;
                    }
                }
                else
                {
                    int id = mode_selection.selectId[i] - swarm_num_;
                    if (this->swarm_ugv_control_simulation_.count(id) == 0)
                    {
                        sendTextInfo(TextInfo::MessageTypeGrade::MTG_WARN, "Switching mode failed, UGV" + to_string(mode_selection.selectId[i]) + " non-existent, " + "please check whether it is connected.");
                        return;
                    }
                }

                // if (this->swarm_control_simulation_.count(mode_selection.selectId[i]) == 0 && this->swarm_ugv_control_simulation_.count(mode_selection.selectId[i]) == 0)
                // {
                //     sendTextInfo(TextInfo::MessageTypeGrade::MTG_WARN, "Switching mode failed, Robot" + to_string(mode_selection.selectId[i]) + " non-existent, " + "please check whether it is connected.");
                //     return;
                // }
            }
            if (this->swarm_control_ == NULL)
            {
                // 如果 预设无人机数量和地面站创建的无人机数量一致 并且无人车为空 说明只有无人机
                if (this->swarm_num_ == this->swarm_control_simulation_.size() && this->swarm_ugv_control_simulation_.empty())
                {
                    // 无人机集群
                    this->swarm_control_ = new SwarmControl(this->nh_, (Communication *)this, SwarmMode::ONLY_UAV, RobotType::ROBOT_TYPE_SIMULATION, this->swarm_num_, this->swarm_ugv_num_);
                }
                // 如果 预设无人车数量和地面站创建的无人车数量一致 并且无人机为空  说明只有无人车
                else if (this->swarm_ugv_num_ == this->swarm_ugv_control_simulation_.size() && this->swarm_control_simulation_.empty())
                {
                    // 无人车集群
                    this->swarm_control_ = new SwarmControl(this->nh_, (Communication *)this, SwarmMode::ONLY_UGV, RobotType::ROBOT_TYPE_SIMULATION, this->swarm_num_, this->swarm_ugv_num_);
                }
                // 如果 预设无人车数量、无人机数量和地面站创建的无人机车数量一致并且不为空 说明该集群为车机协同
                else if (this->swarm_ugv_num_ == this->swarm_ugv_control_simulation_.size() && this->swarm_num_ == this->swarm_control_simulation_.size() && !this->swarm_ugv_control_simulation_.empty() && !this->swarm_control_simulation_.empty())
                {
                    // 车机协同
                    this->swarm_control_ = new SwarmControl(this->nh_, (Communication *)this, SwarmMode::UAV_AND_UGV, RobotType::ROBOT_TYPE_SIMULATION, this->swarm_num_, this->swarm_ugv_num_);
                }
                sendTextInfo(TextInfo::MessageTypeGrade::MTG_INFO, "Mode switching succeeded, current swarm control simulation mode.");
                system(OPENSWARMCONTROL.c_str());
            }
        }
        else // 真机
        {
            if (std::find(mode_selection.selectId.begin(), mode_selection.selectId.end(), ROBOT_ID) == mode_selection.selectId.end())
            {
                sendTextInfo(TextInfo::MessageTypeGrade::MTG_WARN, "Switching mode failed, UAV" + to_string(ROBOT_ID) + " non-existent, " + "please check whether it is connected.");
                return;
            }

            if (this->swarm_control_ == NULL)
            {
                // 开启集群的数量跟无人机数量一致 并且无人车数量为0 进入 无人机集群
                if (this->swarm_num_ == mode_selection.swarm_num && this->swarm_ugv_num_ == 0)
                {
                    this->swarm_control_ = new SwarmControl(this->nh_, (Communication *)this, SwarmMode::ONLY_UAV, ROBOT_ID, this->swarm_num_);
                }
                // 开启集群的数量跟无人车数量一致 并且无人机数量为0 进入 无人车集群
                else if (this->swarm_ugv_num_ == mode_selection.swarm_num && this->swarm_num_ == 0)
                {
                    this->swarm_control_ = new SwarmControl(this->nh_, (Communication *)this, SwarmMode::ONLY_UGV, ROBOT_ID, this->swarm_ugv_num_);
                }
                // 开启集群的数量跟无人车和无人机总数量一致 进入 机车协同
                else if (this->swarm_num_ + this->swarm_ugv_num_ == mode_selection.swarm_num)
                {
                    if (this->uav_basic_ != NULL)
                        this->swarm_control_ = new SwarmControl(this->nh_, (Communication *)this, SwarmMode::UAV_AND_UGV, RobotType::ROBOT_TYPE_UAV, ROBOT_ID, this->swarm_num_, this->swarm_ugv_num_);
                    else if (this->ugv_basic_ != NULL)
                        this->swarm_control_ = new SwarmControl(this->nh_, (Communication *)this, SwarmMode::UAV_AND_UGV, RobotType::ROBOT_TYPE_UGV, ROBOT_ID, this->swarm_num_, this->swarm_ugv_num_);
                }

                sendTextInfo(TextInfo::MessageTypeGrade::MTG_INFO, "Mode switching succeeded, current swarm control mode.");
                system(OPENSWARMCONTROL.c_str());
            }
        }
        // 加载集群参数
        sendSwarmParam();
    }
    else if (mode_selection.mode == ModeSelection::Mode::AUTONOMOUSLANDING)
    {
        if (this->ugv_basic_ != NULL)
        {
            sendTextInfo(TextInfo::MessageTypeGrade::MTG_WARN, "Switching mode failed, because user type UGV.");
            return;
        }
        if (this->uav_basic_ != NULL)
        {
            if (this->gimbal_basic_ == NULL)
            {
                this->gimbal_basic_ = new GimbalBasic(this->nh_, (Communication *)this);
            }
            // 自主降落
            if (this->autonomous_landing_ == NULL)
            {
                this->autonomous_landing_ = new AutonomousLanding(this->nh_, (Communication *)this);
            }
            sendTextInfo(TextInfo::MessageTypeGrade::MTG_INFO, "Mode switching succeeded, current autonomous landing mode.");
            system(OPENAUTONOMOUSLANDING);
        }
    }
    // 目标识别与跟踪模式
    else if (mode_selection.mode == ModeSelection::Mode::OBJECTTRACKING)
    {
        if (this->uav_basic_ != NULL)
        {
            if (this->gimbal_basic_ == NULL)
            {
                this->gimbal_basic_ = new GimbalBasic(this->nh_, (Communication *)this);
            }
            if (this->object_tracking_ == NULL)
            {
                this->object_tracking_ = new ObjectTracking(this->nh_, (Communication *)this);
            }
            sendTextInfo(TextInfo::MessageTypeGrade::MTG_INFO, "Mode switching succeeded, current objectTracking mode.");
            system(OPENOBJECTTRACKING);
        }
    }
    else if (mode_selection.mode == ModeSelection::Mode::CUSTOMMODE)
    {
        system(mode_selection.cmd.c_str());
    }
    else if (mode_selection.mode == ModeSelection::Mode::EGOPLANNER)
    {
        if (this->trajectoy_control_ != NULL)
        {
            delete this->trajectoy_control_;
            this->trajectoy_control_ = NULL;
        }
        if (this->ego_planner_ == NULL)
        {
            this->ego_planner_ = new EGOPlannerSwarm(this->nh_);
        }
        sendTextInfo(TextInfo::MessageTypeGrade::MTG_INFO, "Mode switching succeeded, current EGO planner swarm mode.");
        system(OPENEGOPLANNER);
    }
    else if (mode_selection.mode == ModeSelection::Mode::TRAJECTOYCONTROL)
    {
        if (this->ego_planner_ != NULL)
        {
            delete this->ego_planner_;
            this->ego_planner_ = NULL;
        }
        if (this->trajectoy_control_ == NULL)
        {
            this->trajectoy_control_ = new EGOPlannerSwarm(this->nh_, ROBOT_ID, udp_ip);
        }
        sendTextInfo(TextInfo::MessageTypeGrade::MTG_INFO, "Mode switching succeeded, current trajectoy control mode.");
    }
    this->current_mode_ = mode_selection.mode;
}

void CommunicationBridge::deleteMode(struct ModeSelection mode_selection)
{
    struct TextInfo text_info;
    text_info.MessageType = TextInfo::MessageTypeGrade::MTG_INFO;
    text_info.sec = ros::Time::now().sec;

    if (mode_selection.mode == ModeSelection::Mode::UAVBASIC)
    {
        if (this->is_simulation_ == 1)
        {
            for (int i = 0; i < mode_selection.selectId.size(); i++)
            {
                std::lock_guard<std::mutex> lg_uav_basic(g_uav_basic);
                if (this->swarm_control_simulation_.find(mode_selection.selectId[i]) != this->swarm_control_simulation_.end())
                {
                    delete this->swarm_control_simulation_[mode_selection.selectId[i]];
                    this->swarm_control_simulation_.erase(this->swarm_control_simulation_.find(mode_selection.selectId[i]));

                    if (ROBOT_ID == mode_selection.selectId[i])
                    {
                        this->is_heartbeat_ready_ = false;
                        // delete this->uav_basic_;
                        this->uav_basic_ = NULL;
                        system(CLOSEUAVBASIC.c_str());
                    }
                }
                sendTextInfo(TextInfo::MessageTypeGrade::MTG_INFO, "Simulation UAV" + to_string(mode_selection.selectId[i]) + " disconnect!!!");
            }
        }
        else
        {
            if (std::find(mode_selection.selectId.begin(), mode_selection.selectId.end(), ROBOT_ID) != mode_selection.selectId.end())
            {
                if (this->uav_basic_ != NULL)
                {
                    delete this->uav_basic_;
                    this->uav_basic_ = NULL;
                    system(CLOSEUAVBASIC.c_str());
                    sendTextInfo(TextInfo::MessageTypeGrade::MTG_INFO, "UAV" + to_string(ROBOT_ID) + " disconnect!!!");
                }
            }
        }
        this->is_heartbeat_ready_ = false;
    }
    else if (mode_selection.mode == ModeSelection::Mode::UGVBASIC)
    {
        if (this->is_simulation_ == 1)
        {
            for (int i = 0; i < mode_selection.selectId.size(); i++)
            {
                // std::lock_guard<std::mutex> lg_uav_basic(g_uav_basic);
                if (this->swarm_ugv_control_simulation_.find(mode_selection.selectId[i]) != this->swarm_ugv_control_simulation_.end())
                {
                    delete this->swarm_ugv_control_simulation_[mode_selection.selectId[i]];
                    this->swarm_ugv_control_simulation_.erase(this->swarm_ugv_control_simulation_.find(mode_selection.selectId[i]));

                    if (ROBOT_ID == mode_selection.selectId[i])
                    {
                        this->is_heartbeat_ready_ = false;
                        // delete this->uav_basic_;
                        this->ugv_basic_ = NULL;
                        system(CLOSEUGVBASIC.c_str());
                    }
                }
                sendTextInfo(TextInfo::MessageTypeGrade::MTG_INFO, "Simulation UGV" + to_string(mode_selection.selectId[i]) + " disconnect!!!");
            }
        }
        else
        {
            if (std::find(mode_selection.selectId.begin(), mode_selection.selectId.end(), ROBOT_ID) != mode_selection.selectId.end())
            {
                if (this->ugv_basic_ != NULL)
                {
                    delete this->ugv_basic_;
                    this->ugv_basic_ = NULL;
                    system(CLOSEUGVBASIC.c_str());
                    sendTextInfo(TextInfo::MessageTypeGrade::MTG_INFO, "UGV" + to_string(ROBOT_ID) + " disconnect!!!");
                }
            }
        }
        this->is_heartbeat_ready_ = false;
    }
    else if (mode_selection.mode == ModeSelection::Mode::SWARMCONTROL)
    {
        // std::lock_guard<std::mutex> lg(g_m);
        if (this->swarm_control_ != NULL)
        {
            // 开启互斥锁
            //  boost::unique_lock<boost::shared_mutex> lockImageStatus(g_m);
            std::lock_guard<std::mutex> lg(g_m);
            delete this->swarm_control_;
            this->swarm_control_ = NULL;
            system(CLOSESWARMCONTROL.c_str());
        }
    }
    else if (mode_selection.mode == ModeSelection::Mode::AUTONOMOUSLANDING)
    {
        if (this->autonomous_landing_ != NULL)
        {
            delete this->autonomous_landing_;
            this->autonomous_landing_ = NULL;
            system(CLOSEOTHERMODE);
        }
    }
    else if (mode_selection.mode == ModeSelection::Mode::OBJECTTRACKING)
    {
        if (this->object_tracking_ != NULL)
        {
            delete this->object_tracking_;
            this->object_tracking_ = NULL;
            system(CLOSEOTHERMODE);
        }
    }
    else if (mode_selection.mode == ModeSelection::Mode::EGOPLANNER)
    {
        if (this->object_tracking_ != NULL)
        {
            delete this->ego_planner_;
            this->ego_planner_ = NULL;
            system(CLOSEEGOPLANNER);
        }
    }
}

// 接收组播地址的数据
void CommunicationBridge::multicastUdpFun()
{
    if (this->swarm_num_ == 0 && this->swarm_ugv_num_ == 0)
    {
        return;
    }
    while (this->is_simulation_ == 1)
    {
        std::lock_guard<std::mutex> lg_uav_basic(g_uav_basic);
        for (auto it = this->swarm_control_simulation_.begin(); it != this->swarm_control_simulation_.end(); it++)
        {
            // 开启互斥锁
            //  boost::shared_lock<boost::shared_mutex> lock(g_m);
            std::lock_guard<std::mutex> lg(g_m);
            if (this->swarm_control_ != NULL)
            {
                this->swarm_control_->updateAllUAVState(it->second->getUAVState());
                this->swarm_control_->allUAVStatePub(this->swarm_control_->getMultiUAVState());
            }
        }

        // std::lock_guard<std::mutex> lg_uav_basic(g_uav_basic);
        for (auto it = this->swarm_ugv_control_simulation_.begin(); it != this->swarm_ugv_control_simulation_.end(); it++)
        {
            std::lock_guard<std::mutex> lg(g_m);
            if (this->swarm_control_ != NULL)
            {
                this->swarm_control_->updateAllUGVState(it->second->getUGVState());
                this->swarm_control_->allUGVStatePub(this->swarm_control_->getMultiUGVState());
            }
        }
    }
    int valread;
    if (waitConnectionFromMulticast(UDP_PORT) < 0)
    {
        ROS_ERROR("[bridge_node]Socket recever creation error!");
        exit(EXIT_FAILURE);
    }

    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(multicast_udp_ip.c_str());

    setsockopt(udp_socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));
    sockaddr_in srv_Addr; // 用于存储发送方信息
    socklen_t addr_len = sizeof(srv_Addr);

    while (true)
    {
        std::lock_guard<std::mutex> lg(g_m);
        if (this->swarm_control_ == NULL)
        {
            continue;
        }

        valread = recvfrom(udp_socket, udp_recv_buf, BUF_LEN, 0, (struct sockaddr *)&srv_Addr, &addr_len);

        if (valread <= 0)
        {
            ROS_ERROR("Received message length <= 0, maybe connection has lost");
            continue;
        }

        // std::lock_guard<std::mutex> lg(g_m);

        std::cout << "udp valread: " << valread << std::endl;
        pubMsg(decodeMsg(udp_recv_buf, Send_Mode::UDP));
    }
}

// 无人机触发安全机制（一般为心跳包丢失） 进行降落
void CommunicationBridge::triggerUAV()
{
    // 触发降落  暂定
    struct UAVCommand uav_command;
    uav_command.Agent_CMD = UAVCommand::AgentCMD::Land;
    uav_command.Move_mode = UAVCommand::MoveMode::XYZ_VEL;
    uav_command.yaw_ref = 0;
    uav_command.Yaw_Rate_Mode = true;
    uav_command.yaw_rate_ref = 0;
    uav_command.latitude = 0;
    uav_command.longitude = 0;
    uav_command.altitude = 0;
    for (int i = 0; i < 3; i++)
    {
        uav_command.position_ref[i] = 0;
        uav_command.velocity_ref[i] = 0;
        uav_command.acceleration_ref[i] = 0;
        uav_command.att_ref[i] = 0;
    }
    uav_command.att_ref[3] = 0;
    this->uav_basic_->uavCmdPub(uav_command);
}
// 集群触发安全机制（一般为心跳包丢失）
void CommunicationBridge::triggerSwarmControl()
{
    if (this->is_simulation_ == 0)
    {
        this->swarm_control_->communicationStatePub(false);
    }
    else
    {
        for (int i = 0; i < this->swarm_num_; i++)
        {
            this->swarm_control_->communicationStatePub(false, i);
        }
    }
}
// 无人车触发安全机制（一般为心跳包丢失）
void CommunicationBridge::triggerUGV()
{
    // 停止小车
}

// 给地面站发送心跳包,  超时检测
void CommunicationBridge::toGroundStationFun()
{
    struct Heartbeat heartbeat;
    heartbeat.message = "OK";
    heartbeat.count = 0;

    // 记录 集群数据刷新的时间戳
    uint swarm_control_time[this->swarm_num_] = {0};
    // 记录 未刷新的次数
    uint swarm_control_timeout_count[this->swarm_num_] = {0};
    // 记录 无人机或无人车的时间戳
    uint time = 0;
    uint time_count = 0;
    while (true)
    {
        if (!this->is_heartbeat_ready_)
        {
            continue;
        }
        // std::cout << disconnect_num << std::endl;
        sendMsgByTcp(encodeMsg(Send_Mode::TCP, heartbeat), udp_ip);
        heartbeat_count++;
        heartbeat.count = heartbeat_count;
        if (disconnect_num >= try_connect_num) // 跟地面站断联后的措施
        {
            disconnect_flag = true;
            std::cout << "conenect ground station failed, please check the ground station IP!" << std::endl;
            // 如果是集群模式 由集群模块触发降落
            if (this->swarm_num_ != 0 && this->swarm_control_ != NULL)
            {
                triggerSwarmControl();
                sendTextInfo(TextInfo::MessageTypeGrade::MTG_ERROR, "TCP:" + udp_ip + " abnormal communication,triggering swarm control mode to land.");
            }
            // 无人机 触发降落或者返航
            else if (this->uav_basic_ != NULL)
            {
                triggerUAV();
                sendTextInfo(TextInfo::MessageTypeGrade::MTG_ERROR, "TCP:" + udp_ip + " abnormal communication,trigger landing.");
            }
            // 无人车  停止小车
            else if (this->ugv_basic_ != NULL)
            {
                triggerUGV();
            }
            // 触发机制后 心跳准备标志置为false，停止心跳包的发送 再次接收到地面站指令激活
            this->is_heartbeat_ready_ = false;
        }
        else if (disconnect_flag)
        {
            disconnect_flag = false;
            if (this->swarm_num_ != 0 && this->swarm_control_ != NULL)
            {
                if (this->is_simulation_ == 0)
                {
                    this->swarm_control_->communicationStatePub(true);
                }
                else
                {
                    for (int i = 0; i < this->swarm_num_; i++)
                    {
                        this->swarm_control_->communicationStatePub(true, i);
                    }
                }
            }
            sendTextInfo(TextInfo::MessageTypeGrade::MTG_INFO, "TCP:" + udp_ip + " communication returns to normal.");
        }

        // 无人机数据或者无人车数据是否超时
        if (this->uav_basic_ != NULL || this->ugv_basic_ != NULL)
        {
            uint time_stamp = 0;
            std::string type = "UAV";
            if (this->uav_basic_ != NULL)
            {
                time_stamp = this->uav_basic_->getTimeStamp();
            }
            else if (this->ugv_basic_ != NULL)
            {
                time_stamp = this->ugv_basic_->getTimeStamp();
                type = "UGV";
            }
            // 拿单机状态时间戳进行比较 如果不相等说明数据在更新
            static bool flag = true;
            if (time != time_stamp)
            {
                time = time_stamp;
                if (time_count > this->swarm_data_update_timeout_)
                {
                    sendTextInfo(TextInfo::MessageTypeGrade::MTG_INFO, type + to_string(ROBOT_ID) + " data update returns to normal.");
                    flag = true;
                }
                time_count = 0;
            }
            else // 相等 数据未更新
            {
                time_count++;
                if (time_count > this->swarm_data_update_timeout_)
                {
                    if (flag)
                    {
                        // 反馈地面站
                        sendTextInfo(TextInfo::MessageTypeGrade::MTG_ERROR, type + to_string(ROBOT_ID) + " data update timeout.");
                        flag = false;
                    }
                }
            }
        }

        sleep(1);
    }
}

// 心跳包状态检测
void CommunicationBridge::checkHeartbeatState(const ros::TimerEvent &time_event)
{
    static int disconnect = 0;
    if (!disconnect_flag && this->is_heartbeat_ready_)
    {
        static long count = 0;
        if (count != this->heartbeat_count)
        {
            count = heartbeat_count;
            disconnect = 0;
        }
        else
        {
            disconnect++;
            if (disconnect > try_connect_num)
            {
                std::cout << "conenect ground station failed, please check the ground station IP!" << std::endl;
                // 如果是集群模式 由集群模块触发降落
                if (this->swarm_num_ != 0 && this->swarm_control_ != NULL)
                {
                    triggerSwarmControl();
                }
                // 无人机 触发降落或者返航
                else if (this->uav_basic_ != NULL)
                {
                    triggerUAV();
                }
                // 无人车  停止小车
                else if (this->ugv_basic_ != NULL)
                {
                    // 停止小车
                    triggerUGV();
                }
                disconnect_flag = true;
                // this->is_heartbeat_ready_ = false;
            }
        }
    }
}

bool CommunicationBridge::getParam(struct Param *param)
{
    if (param->type == Param::Type::INT || param->type == Param::Type::LONG)
    {
        int value = 0;
        if (!nh_.getParam(param->param_name, value))
        {
            return false;
        }
        param->param_value = std::to_string(value);
    }
    else if (param->type == Param::Type::FLOAT)
    {
        float value = 0.0;
        if (!nh_.getParam(param->param_name, value))
        {
            return false;
        }
        param->param_value = std::to_string(value);
    }
    else if (param->type == Param::Type::DOUBLE)
    {
        double value = 0.0;
        if (!nh_.getParam(param->param_name, value))
        {
            return false;
        }
        param->param_value = std::to_string(value);
    }
    else if (param->type == Param::Type::BOOLEAN)
    {
        bool value = false;
        if (!nh_.getParam(param->param_name, value))
        {
            return false;
        }
        if (value)
            param->param_value = "true";
        else
            param->param_value = "false";
    }
    else if (param->type == Param::Type::STRING)
    {
        std::string value = "";
        if (!nh_.getParam(param->param_name, value))
        {
            return false;
        }
        param->param_value = value;
    }
    return true;
}

void CommunicationBridge::sendControlParam()
{
    /// communication_bridge/control/
    std::string param_name[15] = {"pos_controller", "enable_external_control", "Takeoff_height", "Land_speed", "Disarm_height", "location_source", "maximum_safe_vel_xy", "maximum_safe_vel_z", "maximum_vel_error_for_vision", "x_min", "x_max", "y_min", "y_max", "z_min", "z_max"};
    int8_t param_type[15] = {Param::Type::INT, Param::Type::BOOLEAN, Param::Type::FLOAT, Param::Type::FLOAT, Param::Type::FLOAT, Param::Type::INT, Param::Type::FLOAT, Param::Type::FLOAT, Param::Type::FLOAT, Param::Type::FLOAT, Param::Type::FLOAT, Param::Type::FLOAT, Param::Type::FLOAT, Param::Type::FLOAT, Param::Type::FLOAT};
    // sendTextInfo(TextInfo::MTG_INFO, "start loading parameters...");
    usleep(1000);
    struct ParamSettings param_settings;
    for (int i = 0; i < 15; i++)
    {
        if (i < 9)
            param_name[i] = "/communication_bridge/control/" + param_name[i];
        else
            param_name[i] = "/communication_bridge/geo_fence/" + param_name[i];
        struct Param param;
        param.param_name = param_name[i];
        param.type = param_type[i];
        if (getParam(&param))
        {
            param_settings.params.push_back(param);
        }
        else
        {
            sendTextInfo(TextInfo::MTG_INFO, "uav control node parameter loading failed...");
            return;
        }
    }
    param_settings.param_module = ParamSettings::ParamModule::UAVCONTROL;
    sendMsgByUdp(encodeMsg(Send_Mode::UDP, param_settings), multicast_udp_ip);
    // sendTextInfo(TextInfo::MTG_INFO, "parameter loading success...");
}
void CommunicationBridge::sendCommunicationParam()
{
    std::string param_name[12] = {"ROBOT_ID", "multicast_udp_ip", "ground_station_ip", "swarm_num", "autoload", "uav_control_start", "close_uav_control", "swarm_control_start", "close_swarm_control", "is_simulation", "swarm_data_update_timeout", "trajectory_ground_control"};
    int8_t param_type[12] = {Param::Type::INT, Param::Type::STRING, Param::Type::STRING, Param::Type::INT, Param::Type::BOOLEAN, Param::Type::STRING, Param::Type::STRING, Param::Type::STRING, Param::Type::STRING, Param::Type::INT, Param::Type::INT, Param::Type::BOOLEAN};
    // sendTextInfo(TextInfo::MTG_INFO, "start loading parameters...");
    usleep(1000);
    struct ParamSettings param_settings;
    for (int i = 0; i < 12; i++)
    {
        param_name[i] = "/communication_bridge/" + param_name[i];
        struct Param param;
        param.param_name = param_name[i];
        param.type = param_type[i];
        if (getParam(&param))
        {
            param_settings.params.push_back(param);
        }
        else
        {
            sendTextInfo(TextInfo::MTG_INFO, "communication node parameter loading failed...");
            return;
        }
    }
    param_settings.param_module = ParamSettings::ParamModule::UAVCOMMUNICATION;
    sendMsgByUdp(encodeMsg(Send_Mode::UDP, param_settings), multicast_udp_ip);
    // sendTextInfo(TextInfo::MTG_INFO, "parameter loading success...");
}
void CommunicationBridge::sendSwarmParam()
{
    std::string param_name[4] = {"takeoff_height", "warning_distance", "danger_distance", "setmode_timeout"};
    int8_t param_type[4] = {Param::Type::FLOAT, Param::Type::FLOAT, Param::Type::FLOAT, Param::Type::FLOAT};
    // sendTextInfo(TextInfo::MTG_INFO, "start loading parameters...");
    usleep(1000);
    struct ParamSettings param_settings;
    for (int i = 0; i < 4; i++)
    {
        param_name[i] = "/communication_bridge/" + param_name[i];
        struct Param param;
        param.param_name = param_name[i];
        param.type = param_type[i];
        if (getParam(&param))
        {
            param_settings.params.push_back(param);
        }
        else
        {
            sendTextInfo(TextInfo::MTG_INFO, "swarm control node parameter loading failed...");
            return;
        }
    }
    param_settings.param_module = ParamSettings::ParamModule::SWARMCONTROL;
    sendMsgByUdp(encodeMsg(Send_Mode::UDP, param_settings), multicast_udp_ip);
    // sendTextInfo(TextInfo::MTG_INFO, "parameter loading success...");
}
void CommunicationBridge::sendCommandPubParam()
{
    std::string param_name[13] = {"Circle/Center_x", "Circle/Center_y", "Circle/Center_z", "Circle/circle_radius", "Circle/direction", "Circle/linear_vel", "Eight/Center_x", "Eight/Center_y", "Eight/Center_z", "Eight/omega", "Eight/radial", "Step/step_interval", "Step/step_length"};
    int8_t param_type[13] = {Param::Type::FLOAT, Param::Type::FLOAT, Param::Type::FLOAT, Param::Type::FLOAT, Param::Type::FLOAT, Param::Type::FLOAT, Param::Type::FLOAT, Param::Type::FLOAT, Param::Type::FLOAT, Param::Type::FLOAT, Param::Type::FLOAT, Param::Type::FLOAT, Param::Type::FLOAT};
    // sendTextInfo(TextInfo::MTG_INFO, "start loading parameters...");
    usleep(1000);
    struct ParamSettings param_settings;
    for (int i = 0; i < 13; i++)
    {
        param_name[i] = "/communication_bridge/Controller_Test/" + param_name[i];
        struct Param param;
        param.param_name = param_name[i];
        param.type = param_type[i];
        if (getParam(&param))
        {
            param_settings.params.push_back(param);
        }
        else
        {
            sendTextInfo(TextInfo::MTG_INFO, "uav control node parameter loading failed...");
            return;
        }
    }
    param_settings.param_module = ParamSettings::ParamModule::UAVCOMMANDPUB;
    sendMsgByUdp(encodeMsg(Send_Mode::UDP, param_settings), multicast_udp_ip);
    // sendTextInfo(TextInfo::MTG_INFO, "parameter loading success...");
}

void CommunicationBridge::sendTextInfo(uint8_t message_type, std::string message)
{
    struct TextInfo text_info;
    text_info.MessageType = message_type;
    text_info.Message = message;
    text_info.sec = ros::Time::now().sec;
    sendMsgByUdp(encodeMsg(Send_Mode::UDP, text_info), multicast_udp_ip);
}
