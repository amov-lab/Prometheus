#include "communication_bridge.hpp"
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
std::mutex g_m;
std::mutex g_uav_basic;
// boost::shared_mutex g_m;

CommunicationBridge::CommunicationBridge(ros::NodeHandle &nh) : Communication()
{
    //是否仿真 1 为是  0为否
    nh.param<int>("is_simulation", this->is_simulation_, 1);
    //集群数量  非集群模式值为0
    nh.param<int>("swarm_num", this->swarm_num_, 0);
    //载体类型
    nh.param<int>("user_type", this->user_type_, 1);
    //集群模式下数据更新超时多久进行反馈
    nh.param<int>("swarm_data_update_timeout", this->swarm_data_update_timeout_, 5);

    nh.param<int>("udp_port", UDP_PORT, 8889);
    nh.param<int>("tcp_port", TCP_PORT, 55555);
    nh.param<int>("tcp_heartbeat_port", TCP_HEARTBEAT_PORT, 55556);
    // nh.param<int>("rviz_port", RVIZ_PORT, 8890);
    nh.param<int>("ROBOT_ID", ROBOT_ID, 1);
    nh.param<std::string>("ground_stationt_ip", udp_ip, "127.0.0.1");
    nh.param<std::string>("multicast_udp_ip", multicast_udp_ip, "224.0.0.88");
    nh.param<int>("try_connect_num", try_connect_num, 3);

    this->nh_ = nh;

    Communication::init(ROBOT_ID, UDP_PORT, TCP_PORT, TCP_HEARTBEAT_PORT);

    bool auto_start = false;
    this->nh_.param<bool>("auto_start", auto_start, false);
    //自动启动话题
    if (auto_start == true)
        init();

    // thread_recCommunicationBridgeiver
    boost::thread recv_thd(&CommunicationBridge::serverFun, this);
    recv_thd.detach();        //后台
    ros::Duration(1).sleep(); // wait

    // thread_receiver
    boost::thread recv_multicast_thd(&CommunicationBridge::multicastUdpFun, this);
    recv_multicast_thd.detach();
    ros::Duration(1).sleep(); // wait

    boost::thread to_ground_station_thd(&CommunicationBridge::toGroundStationFun, this);
    to_ground_station_thd.detach();
    ros::Duration(1).sleep(); // wait

    // system(OPENUAVBASIC);
    sendControlParam();
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

void CommunicationBridge::init()
{
    // if (this->user_type_ == 1)
    // {
    //     this->uav_basic_ = new UAVBasic(this->nh_, ROBOT_ID);
    //     if (this->is_simulation_ == 1)
    //     {
    //         if (this->swarm_num_ != 0)
    //         {
    //             for (int i = 1; i <= this->swarm_num_; i++)
    //             {
    //                 if (i == ROBOT_ID)
    //                 {
    //                     this->swarm_control_simulation_[i] = this->uav_basic_;
    //                     continue;
    //                 }
    //                 this->swarm_control_simulation_[i] = new UAVBasic(this->nh_, i);
    //             }
    //             this->swarm_control_ = new SwarmControl(this->nh_,this->swarm_num_);
    //         }
    //     }else
    //     {
    //         if(this->swarm_num_ != 0)
    //             this->swarm_control_ = new SwarmControl(this->nh_,ROBOT_ID,this->swarm_num_);
    //     }
    //     this->gimbal_basic_ = new GimbalBasic(this->nh_);
    //     this->object_tracking_ = new ObjectTracking(this->nh_);
    //     this->autonomous_landing_ = new  AutonomousLanding(this->nh_);
    // }
    //根据载体进行初始化
    if (this->user_type_ == UserType::UAV)
    {
        this->uav_basic_ = new UAVBasic(this->nh_, ROBOT_ID, (Communication *)this);
    }
    else if (this->user_type_ == UserType::UGV)
    {
        this->ugv_basic_ = new UGVBasic(this->nh_, (Communication *)this);
    }
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
        //等待连接队列中抽取第一个连接，创建一个与s同类的新的套接口并返回句柄。
        if ((recv_sock = accept(server_fd, (struct sockaddr *)NULL, NULL)) < 0)
        {
            perror("accept");
            exit(EXIT_FAILURE);
        }

        // recv函数从TCP连接的另一端接收数据
        valread = recv(recv_sock, tcp_recv_buf, BUF_LEN, 0);
        usleep(200000);

        if (valread <= 0)
        {
            ROS_ERROR("Received message length <= 0, maybe connection has lost");
            close(recv_sock);
            continue;
        }

        // std::lock_guard<std::mutex> lg(g_m);

        std::cout << "tcp valread: " << valread << std::endl;
        // char *ptr = tcp_recv_buf;
        //目前只有地面站发送TCP消息、所以TCP服务端接收到数据后开始心跳包的发送
        this->is_heartbeat_ready_ = true;

        pubMsg(decodeMsg(tcp_recv_buf));
        close(recv_sock);
    }
}

void CommunicationBridge::recvData(struct UAVState uav_state)
{

    if (this->swarm_control_ != NULL)
    {
        //融合到所有无人机状态然后发布话题
        this->swarm_control_->updateAllUAVState(uav_state);
        //发布话题
        this->swarm_control_->allUAVStatePub(this->swarm_control_->getMultiUAVState());
    }
}
void CommunicationBridge::recvData(struct UAVCommand uav_cmd)
{
    if (this->uav_basic_ == NULL)
    {
        return;
    }
    this->uav_basic_->uavCmdPub(uav_cmd);
}
void CommunicationBridge::recvData(struct SwarmCommand swarm_command)
{
    if (this->swarm_control_ == NULL)
    {
        return;
    }
    if (swarm_command.swarm_num != this->swarm_num_)
    {
        struct TextInfo text_info;
        text_info.MessageType = text_info.WARN;
        text_info.Message = "ground station swarm num ！= communication module swarm num";
        text_info.sec = ros::Time::now().sec;
        sendMsgByUdp(encodeMsg(Send_Mode::UDP, text_info), multicast_udp_ip);
    }
    //发布话题
    this->swarm_control_->swarmCommandPub(swarm_command);
}
void CommunicationBridge::recvData(struct ConnectState connect_state)
{
    if (this->is_simulation_ == 0)
        return;
    if (!connect_state.state || connect_state.num < this->swarm_num_)
        // this->swarm_control_->closeUAVState(connect_state.num);
        //触发降落信号
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
    //如果udp_msg数据不为空 则向udo端口发送数据。否则发布ros话题
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
void CommunicationBridge::recvData(struct RheaControl rhea_control)
{
    if (this->ugv_basic_ == NULL)
    {
        return;
    }
    this->ugv_basic_->rheaControlPub(rhea_control);
}
void CommunicationBridge::recvData(struct RheaState rhea_state)
{
    if (this->autonomous_landing_ == NULL)
    {
        return;
    }
    this->autonomous_landing_->rheaStatePub(rhea_state);
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
    if(param_settings.params.size() == 0 && (param_settings.param_module == ParamSettings::ParamModule::UAVCONTROL))
    {
        sendControlParam();
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
        }else if (param_settings.params[i].type == param_settings.params[i].BOOLEAN)
        {
            bool value = param_settings.params[i].param_value == "true"?true:false;
            is = setParam(param_settings.params[i].param_name, value);
        }
        //反馈消息 表示、设置成功与否 textinfo
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
void CommunicationBridge::recvData(struct CustomDataSegment custom_data_segment)
{
    //自定义
}

//根据协议中MSG_ID的值，将数据段数据转化为正确的结构体
void CommunicationBridge::pubMsg(int msg_id)
{
    switch (msg_id)
    {
    case MsgId::UAVSTATE:
        recvData(recv_uav_state_);
        break;
    case MsgId::SWARMCOMMAND:
        recvData(recv_swarm_command_);
        break;
    case MsgId::CONNECTSTATE:
        //集群仿真下有效
        recvData(recv_connect_state_);
        break;
    case MsgId::GIMBALCONTROL:
        recvData(recv_gimbal_control_);
        break;
    case MsgId::GIMBALSERVICE:
        recvData(recv_gimbal_service_);
        break;
    case MsgId::GIMBALPARAMSET:
        recvData(recv_param_set_);
        break;
    case MsgId::WINDOWPOSITION:
        recvData(recv_window_position_);
        break;
    case MsgId::RHEACONTROL:
        recvData(recv_rhea_control_);
        break;
    case MsgId::RHEASTATE:
        recvData(recv_rhea_state_);
        break;
    case MsgId::IMAGEDATA:
        recvData(recv_image_data_);
        break;
    case MsgId::UAVCOMMAND:
        recvData(recv_uav_cmd_);
        break;
    case MsgId::UAVSETUP:
        recvData(recv_uav_setup_);
        break;
    case MsgId::MODESELECTION:
        recvData(recv_mode_selection_);
        break;
    case MsgId::PARAMSETTINGS:
        recvData(recv_param_settings_);
        break;
    case MsgId::BSPLINE:
        recvData(recv_bspline_);
        break;
    case MsgId::MULTIBSPLINES:
        recvData(recv_multi_bsplines_);
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

    struct TextInfo text_info;

    text_info.sec = ros::Time::now().sec;
    if (mode_selection.use_mode == ModeSelection::UseMode::CREATE)
    {
        if (createMode(mode_selection))
        {
            text_info.MessageType = TextInfo::MessageTypeGrade::INFO;
            text_info.Message = "open mode success!";
        }
        else
        {
            text_info.MessageType = TextInfo::MessageTypeGrade::WARN;
            text_info.Message = "open mode fail!";
        }
    }
    else if (mode_selection.use_mode == ModeSelection::UseMode::DELETE)
    {
        if (deleteMode(mode_selection))
        {
            text_info.MessageType = TextInfo::MessageTypeGrade::INFO;
            text_info.Message = "close mode success!";
        }
        else
        {
            text_info.MessageType = TextInfo::MessageTypeGrade::WARN;
            text_info.Message = "close mode fail!";
        }
    }
    sendMsgByUdp(encodeMsg(Send_Mode::UDP, text_info), multicast_udp_ip);
}

bool CommunicationBridge::createMode(struct ModeSelection mode_selection)
{
    struct TextInfo text_info;
    text_info.MessageType = TextInfo::MessageTypeGrade::INFO;
    text_info.sec = ros::Time::now().sec;
    bool is = true;
    if (mode_selection.mode == ModeSelection::Mode::UAVBASIC)
    {
        //仿真模式 允许同一通信节点创建多个飞机的话题
        if (this->is_simulation_ == 1)
        {
            for (int i = 0; i < mode_selection.selectId.size(); i++)
            {
                //判断是否已经存在
                if (!this->swarm_control_simulation_.empty())
                {
                    if (this->swarm_control_simulation_.find(mode_selection.selectId[i]) != this->swarm_control_simulation_.end())
                    {
                        text_info.Message = "UAVBasic simulation id :" + to_string(mode_selection.selectId[i]) + " already exists";
                        sendMsgByUdp(encodeMsg(Send_Mode::UDP, text_info), multicast_udp_ip);
                        return false;
                    }
                }

                //创建并存入
                this->swarm_control_simulation_[mode_selection.selectId[i]] = new UAVBasic(this->nh_, mode_selection.selectId[i], (Communication *)this);
                text_info.Message = "create UAVBasic simulation id :" + to_string(mode_selection.selectId[i]) + "...";
                //如果id与通信节点相同则存入uav_basic_
                if (ROBOT_ID == mode_selection.selectId[i])
                {
                    if (this->uav_basic_ != NULL)
                    {
                        return false;
                    }

                    this->uav_basic_ = this->swarm_control_simulation_[mode_selection.selectId[i]];

                    //打开
                    system(OPENUAVBASIC);
                }

                text_info.Message = "create UAVBasic simulation id :" + to_string(mode_selection.selectId[0]) + "...";
                sendMsgByUdp(encodeMsg(Send_Mode::UDP, text_info), multicast_udp_ip);
            }
        }
        //真机模式 同一通信节点只能创建一个飞机的话题
        else
        {
            for (int i = 0; i < mode_selection.selectId.size(); i++)
            {
                //如果id与通信节点相同则存入uav_basic_
                if (mode_selection.selectId[i] == ROBOT_ID)
                {
                    if (this->uav_basic_ == NULL)
                    {
                        this->uav_basic_ = new UAVBasic(this->nh_, ROBOT_ID, (Communication *)this);
                        text_info.Message = "create UAVBasic :" + to_string(ROBOT_ID) + "...";

                        //启动 uav_control节点
                        //先关闭防止重复打开
                        // system(CLOSEUAVBASIC);
                        //打开
                        // system(OPENUAVBASIC);
                    }
                }
                else
                {
                    text_info.MessageType = TextInfo::MessageTypeGrade::WARN;
                    text_info.Message = "id inconformity";
                    is = false;
                }
                sendMsgByUdp(encodeMsg(Send_Mode::UDP, text_info), multicast_udp_ip);
            }
        }
    }
    else if (mode_selection.mode == ModeSelection::Mode::UGVBASIC)
    {
        if (this->ugv_basic_ == NULL)
        {
            this->ugv_basic_ = new UGVBasic(this->nh_, (Communication *)this);
            text_info.Message = "UGVBasic";
            sendMsgByUdp(encodeMsg(Send_Mode::UDP, text_info), multicast_udp_ip);
            system(CLOSEUGVBASIC);
            system(OPENUGVBASIC);
        }
    }
    //集群模式
    else if (mode_selection.mode == ModeSelection::Mode::SWARMCONTROL)
    {
        if (this->swarm_num_ != mode_selection.selectId.size())
        {
            text_info.MessageType = TextInfo::MessageTypeGrade::WARN;
            text_info.Message = "mode switch fail，because swarm num inconsistent.";
            sendMsgByUdp(encodeMsg(Send_Mode::UDP, text_info), multicast_udp_ip);
            return false;
        }
        if (this->is_simulation_ != mode_selection.is_simulation)
        {
            text_info.MessageType = TextInfo::MessageTypeGrade::WARN;
            text_info.Message = "mode switch fail，because mode inconsistent.";
            sendMsgByUdp(encodeMsg(Send_Mode::UDP, text_info), multicast_udp_ip);
            return false;
        }
        //仿真模式
        if (this->is_simulation_ == 1)
        {
            for (int i = 0; i < mode_selection.selectId.size(); i++)
            {
                if (this->swarm_control_simulation_.count(mode_selection.selectId[i]) == 0)
                {
                    text_info.MessageType = TextInfo::MessageTypeGrade::WARN;
                    text_info.Message = "mode switch fail，id " + to_string(mode_selection.selectId[i]) + " non-existent";
                    sendMsgByUdp(encodeMsg(Send_Mode::UDP, text_info), multicast_udp_ip);
                    return false;
                }
            }
            if (this->swarm_control_ == NULL)
            {
                this->swarm_control_ = new SwarmControl(this->nh_, this->swarm_num_, (Communication *)this);
                // this->swarm_control_ = std::make_shared<SwarmControl>(this->nh_, this->swarm_num);
                text_info.Message = "simulation SwarmControl: swarm_num:" + std::to_string(this->swarm_num_);
                sendMsgByUdp(encodeMsg(Send_Mode::UDP, text_info), multicast_udp_ip);
                system(OPENSWARMCONTROL);
            }
        }
        else //真机
        {
            for (int i = 0; i < mode_selection.selectId.size(); i++)
            {
                if (mode_selection.selectId[i] == ROBOT_ID)
                {
                    this->swarm_control_ = new SwarmControl(this->nh_, ROBOT_ID, this->swarm_num_, (Communication *)this);
                    text_info.Message = "SwarmControl: swarm_num:" + std::to_string(this->swarm_num_);
                    sendMsgByUdp(encodeMsg(Send_Mode::UDP, text_info), multicast_udp_ip);
                    break;
                }
                if (i == mode_selection.selectId.size() - 1)
                {
                    text_info.MessageType = TextInfo::MessageTypeGrade::WARN;
                    text_info.Message = "mode switch fail，id " + to_string(ROBOT_ID) + " non-existent";
                    sendMsgByUdp(encodeMsg(Send_Mode::UDP, text_info), multicast_udp_ip);
                    return false;
                }
            }
        }

        //启动子模块指令
        // system()
    }

    else if (mode_selection.mode == ModeSelection::Mode::AUTONOMOUSLANDING)
    {
        if (this->ugv_basic_ != NULL)
        {
            text_info.MessageType = TextInfo::MessageTypeGrade::WARN;
            text_info.Message = "mode switch fail，because user type ugv.";
            sendMsgByUdp(encodeMsg(Send_Mode::UDP, text_info), multicast_udp_ip);
            return false;
        }
        if (this->uav_basic_ != NULL)
        {
            if (this->gimbal_basic_ == NULL)
            {
                this->gimbal_basic_ = new GimbalBasic(this->nh_, (Communication *)this);
            }
            //自主降落
            if (this->autonomous_landing_ == NULL)
            {
                this->autonomous_landing_ = new AutonomousLanding(this->nh_, (Communication *)this);
            }
            text_info.Message = "AutonomousLanding";
            sendMsgByUdp(encodeMsg(Send_Mode::UDP, text_info), multicast_udp_ip);
            system(OPENAUTONOMOUSLANDING);
        }
    }
    //目标识别与跟踪模式
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
            text_info.Message = "ObjectTracking";
            sendMsgByUdp(encodeMsg(Send_Mode::UDP, text_info), multicast_udp_ip);
            system(OPENOBJECTTRACKING);
        }
    }
    else if (mode_selection.mode == ModeSelection::Mode::CUSTOMMODE)
    {
        system(mode_selection.cmd.c_str());
    }
    else if (mode_selection.mode == ModeSelection::Mode::EGOPLANNER)
    {
        if (this->ego_planner_ == NULL)
        {
            this->ego_planner_ = new EGOPlannerSwarm(this->nh_);
        }
        text_info.Message = "EGOPlannerSwarm";
        sendMsgByUdp(encodeMsg(Send_Mode::UDP, text_info), multicast_udp_ip);
        system(OPENEGOPLANNER);
    }
    this->current_mode_ = mode_selection.mode;
    return is;
}

bool CommunicationBridge::deleteMode(struct ModeSelection mode_selection)
{
    struct TextInfo text_info;
    text_info.MessageType = TextInfo::MessageTypeGrade::INFO;
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
                        // delete this->uav_basic_;
                        this->uav_basic_ = NULL;
                        system(CLOSEUAVBASIC);
                    }
                }
                text_info.Message = "delete UAVBasic simulation id :" + to_string(mode_selection.selectId[i]) + "...";
                sendMsgByUdp(encodeMsg(Send_Mode::UDP, text_info), multicast_udp_ip);
            }
        }
        else
        {
            for (int i = 0; i < mode_selection.selectId.size(); i++)
            {
                if (ROBOT_ID == mode_selection.selectId[i])
                {
                    if (this->uav_basic_ != NULL)
                    {
                        delete this->uav_basic_;
                        this->uav_basic_ = NULL;
                        system(CLOSEUAVBASIC);
                        text_info.Message = "delete UAVBasic id :" + to_string(mode_selection.selectId[i]) + "...";
                    }
                }
                else
                {
                    text_info.MessageType = TextInfo::MessageTypeGrade::WARN;
                    text_info.Message = "id inconformity";
                    return false;
                }
                sendMsgByUdp(encodeMsg(Send_Mode::UDP, text_info), multicast_udp_ip);
            }
        }
    }
    else if (mode_selection.mode == ModeSelection::Mode::UGVBASIC)
    {
        if (this->ugv_basic_ != NULL)
        {
            delete this->ugv_basic_;
            this->ugv_basic_ = NULL;
            system(CLOSEUGVBASIC);
        }
    }
    else if (mode_selection.mode == ModeSelection::Mode::SWARMCONTROL)
    {
        // std::lock_guard<std::mutex> lg(g_m);
        if (this->swarm_control_ != NULL)
        {
            //开启互斥锁
            // boost::unique_lock<boost::shared_mutex> lockImageStatus(g_m);
            std::lock_guard<std::mutex> lg(g_m);
            delete this->swarm_control_;
            this->swarm_control_ = NULL;
            system(CLOSEOTHERMODE);
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
    return true;
}

//接收组播地址的数据
void CommunicationBridge::multicastUdpFun()
{
    if (this->swarm_num_ == 0)
    {
        return;
    }
    while (this->is_simulation_ == 1)
    {
        std::lock_guard<std::mutex> lg_uav_basic(g_uav_basic);
        for (auto it = this->swarm_control_simulation_.begin(); it != this->swarm_control_simulation_.end(); it++)
        {
            //开启互斥锁
            // boost::shared_lock<boost::shared_mutex> lock(g_m);
            std::lock_guard<std::mutex> lg(g_m);
            if (this->swarm_control_ != NULL)
            {
                this->swarm_control_->updateAllUAVState(it->second->getUAVState());
                this->swarm_control_->allUAVStatePub(this->swarm_control_->getMultiUAVState());
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
    sockaddr_in srv_Addr; //用于存储发送方信息
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
        pubMsg(decodeMsg(udp_recv_buf));
    }
}

//给地面站发送心跳包,  超时检测
void CommunicationBridge::toGroundStationFun()
{
    struct Heartbeat heartbeat;
    heartbeat.message = "OK";
    heartbeat.count = 0;

    //记录 集群数据刷新的时间戳
    uint swarm_control_time[this->swarm_num_] = {0};
    //记录 未刷新的次数
    uint swarm_control_timeout_count[this->swarm_num_] = {0};
    //记录 无人机或无人车的时间戳
    uint time = 0;
    uint time_count = 0;
    while (true)
    {
        if (!this->is_heartbeat_ready_)
        {
            continue;
        }

        sendMsgByTcp(encodeMsg(Send_Mode::TCP, heartbeat), udp_ip);
        heartbeat.count++;
        if (disconnect_num > try_connect_num) //跟地面站断联后的措施
        {
            std::cout << "conenect ground station failed！" << std::endl;
            //如果是集群模式 由集群模块触发降落
            if (this->swarm_num_ != 0 && this->swarm_control_ != NULL)
            {
                if (this->is_simulation_ == 0)
                    this->swarm_control_->communicationStatePub(false);
                else
                {
                    for (int i = 0; i < this->swarm_num_; i++)
                    {
                        this->swarm_control_->communicationStatePub(false, i);
                    }
                }
            }
            //无人机 触发降落或者返航
            else if (this->uav_basic_ != NULL)
            {
                //触发降落  暂定
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
            //无人车  停止小车
            else if (this->ugv_basic_ != NULL)
            {
                //停止小车
                struct RheaControl rhea_control;
                rhea_control.Mode = RheaControl::Mode::Stop;
                rhea_control.linear = 0;
                rhea_control.angular = 0;
                this->ugv_basic_->rheaControlPub(rhea_control);
            }
            //触发机制后 心跳准备标志置为false，停止心跳包的发送 再次接收到地面站指令激活
            this->is_heartbeat_ready_ = false;
        }

        //无人机数据或者无人车数据是否超时
        if (this->uav_basic_ != NULL || this->ugv_basic_ != NULL)
        {
            uint time_stamp = 0;
            if (this->uav_basic_ != NULL)
            {
                time_stamp = this->uav_basic_->getTimeStamp();
            }
            else if (this->ugv_basic_ != NULL)
            {
                time_stamp = this->ugv_basic_->getTimeStamp();
            }
            //拿单机状态时间戳进行比较 如果不相等说明数据在更新
            if (time != time_stamp)
            {
                time = time_stamp;
                time_count = 0;
            }
            else //相等 数据未更新
            {
                time_count++;
                if (time_count > this->swarm_data_update_timeout_)
                {
                    //反馈地面站
                    struct TextInfo text_info;
                    text_info.MessageType = TextInfo::MessageTypeGrade::ERROR;
                    if (this->uav_basic_ != NULL)
                        text_info.Message = "UAV" + to_string(ROBOT_ID) + " data update timeout";
                    else
                        text_info.Message = "UGV" + to_string(ROBOT_ID) + " data update timeout";
                    text_info.sec = ros::Time::now().sec;
                    sendMsgByUdp(encodeMsg(Send_Mode::UDP, text_info), multicast_udp_ip);
                    usleep(10);
                }
            }
        }

        sleep(1);
    }
}

bool CommunicationBridge::getParam(struct Param* param)
{
    if(param->type == Param::Type::INT || param->type == Param::Type::LONG)
    {
        int value = 0;
        if(!nh_.getParam(param->param_name,value))
        {
            return false;
        }
        param->param_value = std::to_string(value);
    }else if(param->type == Param::Type::FLOAT)
    {
        float value = 0.0;
        if(!nh_.getParam(param->param_name,value))
        {
            return false;
        }
        param->param_value = std::to_string(value);
    }else if(param->type == Param::Type::DOUBLE)
    {
        double value = 0.0;
        if(!nh_.getParam(param->param_name,value))
        {
            return false;
        }
        param->param_value = std::to_string(value);
    }else if(param->type == Param::Type::BOOLEAN)
    {
        bool value = false;
        if(!nh_.getParam(param->param_name,value))
        {
            return false;
        }
        if(value) param->param_value = "true";
        else param->param_value = "false";
    }else if(param->type == Param::Type::STRING)
    {
        std::string value = "";
        if(!nh_.getParam(param->param_name,value))
        {
            return false;
        }
        param->param_value = value;
    }
    return true;
}

void CommunicationBridge::sendControlParam()
{
    ///communication_bridge/control/
    std::string param_name[15] = {"pos_controller","enable_external_control","Takeoff_height","Land_speed","Disarm_height","location_source","maximum_safe_vel_xy","maximum_safe_vel_z","maximum_vel_error_for_vision","x_min","x_max","y_min","y_max","z_min","z_max"};
    int8_t param_type[15] = {Param::Type::INT,Param::Type::BOOLEAN,Param::Type::FLOAT,Param::Type::FLOAT,Param::Type::FLOAT,Param::Type::INT,Param::Type::FLOAT,Param::Type::FLOAT,Param::Type::FLOAT,Param::Type::FLOAT,Param::Type::FLOAT,Param::Type::FLOAT,Param::Type::FLOAT,Param::Type::FLOAT,Param::Type::FLOAT};
    sendTextInfo(TextInfo::INFO,"开始加载参数...");
    usleep(500);
    struct ParamSettings param_settings;
    for(int i = 0;i < 15; i++)
    {
        if(i < 9) param_name[i] = "/communication_bridge/control/" + param_name[i];
        else param_name[i] = "/communication_bridge/geo_fence/" + param_name[i];
        struct Param param;
        param.param_name = param_name[i];
        param.type = param_type[i];
        if(getParam(&param))
        {
            param_settings.params.push_back(param);
            std::cout << param.param_name << " " << param.param_value << std::endl;
        }else
        {
            sendTextInfo(TextInfo::INFO,"参数加载失败...");
            return;
        }
    }
    param_settings.param_module = ParamSettings::ParamModule::UAVCONTROL;
    sendMsgByUdp(encodeMsg(Send_Mode::UDP, param_settings), multicast_udp_ip);
    usleep(500);
    sendTextInfo(TextInfo::INFO,"参数加载完成...");
    usleep(500);
}

void CommunicationBridge::sendTextInfo(uint8_t message_type,std::string message)
{
    struct TextInfo text_info;
    text_info.MessageType = message_type;
    text_info.Message = message;
    text_info.sec = ros::Time::now().sec;
    sendMsgByUdp(encodeMsg(Send_Mode::UDP, text_info), multicast_udp_ip);
}