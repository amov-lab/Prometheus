#include "uav_controller.h"

UAV_controller::UAV_controller(ros::NodeHandle &nh)
{
    // 【参数】无人机编号，从1开始编号
    nh.param<int>("uav_id", uav_id, 1);
    // 【变量】无人机名字
    uav_name = "/uav" + std::to_string(uav_id);
    // 【变量】节点名字
    node_name = "[uav_controller_uav" + std::to_string(uav_id) + "]";
    // 【参数】是否仿真模式
    nh.param<bool>("sim_mode", sim_mode, true);
    // 【参数】控制器标志位,具体说明见CONTOLLER_FLAG说明
    nh.param<int>("control/pos_controller", pos_controller, 0);
    // 【参数】使用外部控制器的控制指令，直接发送至PX4，这种方式依赖外部控制器的稳定性，需要小心！
    nh.param<bool>("control/enable_external_control", enable_external_control, false);
    // 【参数】起飞高度
    nh.param<float>("control/Takeoff_height", Takeoff_height, 1.0);
    // 【参数】降落时自动上锁高度
    nh.param<float>("control/Disarm_height", Disarm_height, 0.2);
    // 【参数】降落速度
    nh.param<float>("control/Land_speed", Land_speed, 0.2);
    // 【参数】地理围栏
    nh.param<float>("geo_fence/x_min", uav_geo_fence.x_min, -100.0);
    nh.param<float>("geo_fence/x_max", uav_geo_fence.x_max, 100.0);
    nh.param<float>("geo_fence/y_min", uav_geo_fence.y_min, -100.0);
    nh.param<float>("geo_fence/y_max", uav_geo_fence.y_max, 100.0);
    nh.param<float>("geo_fence/z_min", uav_geo_fence.z_min, -100.0);
    nh.param<float>("geo_fence/z_max", uav_geo_fence.z_max, 100.0);

    // 【函数】打印参数
    printf_param();
    cout << GREEN << node_name << " init! " << TAIL << endl;

    if (pos_controller == POS_CONTOLLER::PX4_ORIGIN)
    {
        cout << GREEN << node_name << " Using the PX4 original controller... " << TAIL << endl;
    }
    else if (pos_controller == POS_CONTOLLER::PID)
    {
        // 【控制器】PID控制器初始化
        pos_controller_pid.init(nh);
        cout << YELLOW << node_name << " Using the PID controller... " << TAIL << endl;
    }
    else if (pos_controller == POS_CONTOLLER::UDE)
    {
        // 【控制器】UDE控制器初始化
        pos_controller_ude.init(nh);
        cout << YELLOW << node_name << " Using the UDE controller... " << TAIL << endl;
    }
    else if (pos_controller == POS_CONTOLLER::NE)
    {
        // 【控制器】NE控制器初始化
        pos_controller_ne.init(nh);
        cout << YELLOW << node_name << " Using the NE controller... " << TAIL << endl;
    }
    else
    {
        pos_controller = POS_CONTOLLER::PX4_ORIGIN;
        cout << YELLOW << node_name << " wrong pos_controller param, reset to PX4_ORIGIN! " << TAIL << endl;
    }

    //【订阅】无人机控制指令(用于COMMAND_CONTROL模式)
    uav_cmd_sub =
        nh.subscribe<prometheus_msgs::UAVCommand>("/uav" + std::to_string(uav_id) + "/prometheus/command",
                                                  1,
                                                  &UAV_controller::uav_cmd_cb, this);

    //【订阅】无人机状态信息
    uav_state_sub = nh.subscribe<prometheus_msgs::UAVState>("/uav" + std::to_string(uav_id) + "/prometheus/state",
                                                            1,
                                                            &UAV_controller::uav_state_cb, this);

    //【订阅】无人机设置指令
    uav_setup_sub =
        nh.subscribe<prometheus_msgs::UAVSetup>("/uav" + std::to_string(uav_id) + "/prometheus/setup",
                                                1,
                                                &UAV_controller::uav_setup_cb, this);

    //【订阅】PX4中无人机的位置/速度/加速度设定值 坐标系:ENU系
    px4_position_target_sub =
        nh.subscribe<mavros_msgs::PositionTarget>("/uav" + std::to_string(uav_id) + "/mavros/setpoint_raw/target_local",
                                                  1,
                                                  &UAV_controller::px4_pos_target_cb, this);

    //【订阅】PX4中无人机的姿态设定值 坐标系:ENU系
    px4_attitude_target_sub =
        nh.subscribe<mavros_msgs::AttitudeTarget>("/uav" + std::to_string(uav_id) + "/mavros/setpoint_raw/target_attitude",
                                                  1,
                                                  &UAV_controller::px4_att_target_cb, this);

    //【订阅】PX4遥控器数据
    string rc_topic_name;
    if (sim_mode)
    {
        rc_topic_name = "/uav" + std::to_string(uav_id) + "/prometheus/fake_rc_in";
    }
    else
    {
        rc_topic_name = "/uav" + std::to_string(uav_id) + "/mavros/rc/in";
    }
    px4_rc_sub =
        nh.subscribe<mavros_msgs::RCIn>(rc_topic_name,
                                        1,
                                        &UAV_controller::px4_rc_cb, this);
    
    //【订阅】GPS位置偏移数据(用于户外多机飞行)
    offset_pose_sub =
        nh.subscribe<prometheus_msgs::OffsetPose>("/uav" + std::to_string(uav_id) + "/prometheus/offset_pose",
                                                  1,
                                                  &UAV_controller::offset_pose_cb, this);

    // 【发布】位置/速度/加速度期望值 坐标系:ENU系
    px4_setpoint_raw_local_pub = 
        nh.advertise<mavros_msgs::PositionTarget>("/uav" + std::to_string(uav_id) + "/mavros/setpoint_raw/local", 1);

    // 【发布】经纬度以及高度位置 坐标系:WGS84坐标系
    px4_setpoint_raw_global_pub = 
        nh.advertise<mavros_msgs::GlobalPositionTarget>("/uav" + std::to_string(uav_id) + "/mavros/setpoint_raw/global", 1);

    // 【发布】姿态期望值
    px4_setpoint_raw_attitude_pub =
        nh.advertise<mavros_msgs::AttitudeTarget>("/uav" + std::to_string(uav_id) + "/mavros/setpoint_raw/attitude", 1);

    // 【发布】控制状态（监控用）
    uav_control_state_pub =
        nh.advertise<prometheus_msgs::UAVControlState>("/uav" + std::to_string(uav_id) + "/prometheus/control_state", 1);

    //【发布】运行状态信息(本节点 -> 通信节点 -> 地面站)
    ground_station_info_pub = nh.advertise<prometheus_msgs::TextInfo>("/uav" + std::to_string(uav_id) + "/prometheus/text_info", 1);

    // 【服务】解锁/上锁
    px4_arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav" + std::to_string(uav_id) + "/mavros/cmd/arming");

    // 【服务】修改系统模式
    px4_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav" + std::to_string(uav_id) + "/mavros/set_mode");

    // 【服务】紧急上锁服务(KILL)
    px4_emergency_client = nh.serviceClient<mavros_msgs::CommandLong>("/uav" + std::to_string(uav_id) + "/mavros/cmd/command");

    // 【服务】重启PX4飞控
    px4_reboot_client = nh.serviceClient<mavros_msgs::CommandLong>("/uav" + std::to_string(uav_id) + "/mavros/cmd/command");
    
    this->ground_station_info_timer = nh.createTimer(ros::Duration(0.1), &UAV_controller::sendStationTextInfo, this);

    control_state = CONTROL_STATE::INIT;
    uav_control_state.failsafe = false;

    // 初始化命令
    uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
    uav_command.position_ref[0] = 0;
    uav_command.position_ref[1] = 0;
    uav_command.position_ref[2] = 0.0;
    uav_command.yaw_ref = 0;

    pos_des << 0.0, 0.0, 0.0;
    vel_des << 0.0, 0.0, 0.0;
    acc_des << 0.0, 0.0, 0.0;
    quick_land = false;
    set_landing_des = false;

    offset_pose.x = 0.0;
    offset_pose.y = 0.0;

    uav_pos.setZero();
    uav_vel.setZero();
    u_att.setZero();

    rc_input.init();

    text_info.MessageType = prometheus_msgs::TextInfo::INFO;
    text_info.Message = node_name + " init.";
}

void UAV_controller::mainloop()
{
    if (control_state == CONTROL_STATE::RC_POS_CONTROL || control_state == CONTROL_STATE::COMMAND_CONTROL)
    {
        // 安全检查 - 包括地理围栏、定位有效性检查
        int safety_flag = check_failsafe();

        if (safety_flag == -1)
        {
            // 与PX4断开连接，直接返回
            this->text_info.MessageType = prometheus_msgs::TextInfo::ERROR;
            this->text_info.Message = "Unconnected PX4, Waiting for PX4 connection";
            return;
        }
        else if (safety_flag == 1)
        {
            // 超出geofence，原地降落
            uav_control_state.failsafe = true;
            this->text_info.MessageType = prometheus_msgs::TextInfo::ERROR;
            this->text_info.Message = "Out of the geo fence, swtich to land control mode！";
            control_state = CONTROL_STATE::LAND_CONTROL;
        }
        else if (safety_flag == 2)
        {
            // 检测到odom失效，快速降落
            quick_land = true;
            uav_control_state.failsafe = true;
            this->text_info.MessageType = prometheus_msgs::TextInfo::ERROR;
            this->text_info.Message = "Odom invalid, swtich to land control mode!";
            control_state = CONTROL_STATE::LAND_CONTROL;
        }

        // 检查是否满足维持在HOVER_CONTROL的条件，不满足则自动退出
        if (uav_state.mode != "OFFBOARD")
        {
            // 进入HOVER_CONTROL，需启动OFFBOARD模式
            set_px4_mode_func("OFFBOARD");
        }
    }

    switch (control_state)
    {
    case CONTROL_STATE::INIT:

        if(sim_mode)
        {
            // 检查无人机是否位于定点模式，否则切换至定点模式
            if (uav_state.mode != "POSCTL")
            {
                set_px4_mode_func("POSCTL");
            }
        }else
        {
            // 检查无人机是否位于定点模式，否则切换至定点模式
            if (uav_state.mode != "POSCTL")
            {
                set_px4_mode_func("POSCTL");
            }
        }
        break;

    case CONTROL_STATE::RC_POS_CONTROL:

        set_hover_pose_with_rc();
        pos_des = Hover_position;
        vel_des << 0.0, 0.0, 0.0;
        acc_des << 0.0, 0.0, 0.0;
        yaw_des = Hover_yaw;
        break;

    case CONTROL_STATE::COMMAND_CONTROL:

        // 设置期望值
        if (pos_controller == POS_CONTOLLER::PX4_ORIGIN)
        {
            // PX4_ORIGIN支持所有控制模式
            set_command_des();
        }
        else
        {
            // 注意：目前内置的位置环控制器,如PID,UDE,NE仅支持位置控制(XYZ_POS)及轨迹控制(TRAJECTORY)
            // 如果收到不支持的指令,则会转到初始悬停点
            set_command_des_for_pos_controller();
        }

        break;

    // 当前位置原地降落，降落后会自动上锁，且切换为mannual模式
    case CONTROL_STATE::LAND_CONTROL:

        if (uav_state.location_source == prometheus_msgs::UAVState::GPS || uav_state.location_source == prometheus_msgs::UAVState::RTK)
        {
            if(uav_state.armed && uav_state.mode != "AUTO.LAND")
            {
                set_px4_mode_func("AUTO.LAND");
            }
        }else
        {
            // 第一次进入，设置降落的期望位置和速度
            if (!set_landing_des)
            {
                // 快速降落 - 一般用于无人机即将失控时，快速降落保证安全
                if (quick_land)
                {
                    Land_speed = 1.0;
                }
                pos_des[0] = uav_pos[0];
                pos_des[1] = uav_pos[1];
                pos_des[2] = Takeoff_position[2]; // 高度设定为初始起飞时的高度
                vel_des << 0.0, 0.0, -Land_speed;
                acc_des << 0.0, 0.0, 0.0;
                yaw_des = uav_yaw;
                set_landing_des = true;
            }
            // 当无人机位置低于指定高度时，自动上锁
            // 需要考虑万一高度数据不准确时，从高处自由落体
            if (uav_pos[2] < Disarm_height)
            {
                // 进入急停
                enable_emergency_func();
            }
        }

        // 降落结束的标志：无人机上锁
        if (!uav_state.armed)
        {
            control_state = CONTROL_STATE::INIT;
            //控制命令初始化,不初始化将影响setup接口切换command_control模式
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
            set_landing_des = false;
        }

        break;
    }

    last_control_state = control_state;

    // 发布控制器状态
    uav_control_state.uav_id = uav_id;
    uav_control_state.control_state = control_state;
    uav_control_state.pos_controller = pos_controller;
    uav_control_state_pub.publish(uav_control_state);

    // INIT下，Prometheus不发布任何控制指令，直接返回
    if (control_state == CONTROL_STATE::INIT)
    {
        return;
    }

    // 依据controller_flag调用不同位置环控制算法进行控制
    // 此时需要满足两个条件:
    // 1, 无人机有稳定准确的定位,由uav_state_cb()函数获得 
    // 2, 无人机知道自己要去哪，即期望位置pos_des等
    // HOVER_CONTROL和LAND_CONTROL的指令信息由程序根据当前状态计算得到，COMMAND_CONTROL的指令信息由uav_cmd_cb()函数获得

    // 发送控制指令 -> Mavros -> PX4
    if (pos_controller == POS_CONTOLLER::PX4_ORIGIN)
    {
        // 发送位置控制指令至PX4的原生位置环控制器
        send_pos_cmd_to_px4_original_controller();
    }
    else
    {
        // 利用Prometheus中编写的控制程序解算姿态控制指令
        Eigen::Vector4d u_att = get_cmd_from_controller();
        // 发送姿态控制指令至PX4的原生姿态环控制器
        send_attitude_setpoint(u_att);
    }
}

Eigen::Vector4d UAV_controller::get_cmd_from_controller()
{
    // 期望值
    Desired_State desired_state;
    desired_state.pos = pos_des;
    desired_state.vel = vel_des;
    desired_state.acc = acc_des;
    desired_state.yaw = yaw_des;
    desired_state.q = geometry_utils::yaw_to_quaternion(yaw_des);
    // 计算
    if (pos_controller == POS_CONTOLLER::PID)
    {
        // 设定期望值
        pos_controller_pid.set_desired_state(desired_state);
        // 设定当前值
        pos_controller_pid.set_current_state(uav_state);
        // 控制器更新
        Eigen::Vector4d u_att_from_pos_controller = pos_controller_pid.update(200.0);
        return u_att_from_pos_controller;
    }
    else if (pos_controller == POS_CONTOLLER::UDE)
    {
        // 设定期望值
        pos_controller_ude.set_desired_state(desired_state);
        // 设定当前值
        pos_controller_ude.set_current_state(uav_state);
        // 控制器更新
        Eigen::Vector4d u_att_from_pos_controller = pos_controller_ude.update(200.0);
        return u_att_from_pos_controller;
    }
    else if (pos_controller == POS_CONTOLLER::NE)
    {
        // 设定期望值
        pos_controller_ne.set_desired_state(desired_state);
        // 设定当前值
        pos_controller_ne.set_current_state(uav_state);
        // 控制器更新
        Eigen::Vector4d u_att_from_pos_controller = pos_controller_ne.update(200.0);
        return u_att_from_pos_controller;
    }
}

void UAV_controller::set_hover_pose_with_odom()
{
    // 设定悬停点
    Hover_position[0] = uav_pos[0] - offset_pose.x;
    Hover_position[1] = uav_pos[1] - offset_pose.y;
    Hover_position[2] = uav_pos[2];
    Hover_yaw = uav_yaw;

    last_set_hover_pose_time = ros::Time::now();
}

void UAV_controller::set_hover_pose_with_rc()
{
    ros::Time now = ros::Time::now();
    double delta_t = (now - last_set_hover_pose_time).toSec();
    last_set_hover_pose_time = now;

    double max_vel_xy = 1.5;
    double max_vel_z = 1.3;
    double max_vel_yaw = 1.5;

    // 悬停位置 = 前一个悬停位置 + 遥控器数值[-1,1] * 0.01(如果主程序中设定是100Hz的话)
    Hover_position(0) += rc_input.ch[1] * max_vel_xy * delta_t;
    Hover_position(1) += -rc_input.ch[0] * max_vel_xy * delta_t;
    Hover_position(2) += rc_input.ch[2] * max_vel_z * delta_t;
    Hover_yaw += -rc_input.ch[3] * max_vel_yaw * delta_t;
    // 因为这是一个积分系统，所以即使停杆了，无人机也还会继续移动一段距离

    // 高度限制
    if (Hover_position(2) < 0.2)
        Hover_position(2) = 0.2;
}

void UAV_controller::set_command_des()
{
    if (uav_command.Agent_CMD == prometheus_msgs::UAVCommand::Init_Pos_Hover)
    {
        //【Init_Pos_Hover】 移动到指定起飞位置
        pos_des << Takeoff_position + Eigen::Vector3d(0, 0, Takeoff_height);
        vel_des << 0.0, 0.0, 0.0;
        acc_des << 0.0, 0.0, 0.0;
        yaw_des = uav_command.yaw_ref;
    }
    else if (uav_command.Agent_CMD == prometheus_msgs::UAVCommand::Current_Pos_Hover)
    {
        // 【Current_Pos_Hover】 当前位置悬停
        if (uav_command_last.Agent_CMD != prometheus_msgs::UAVCommand::Current_Pos_Hover)
        {
            Hover_position[0] = uav_pos[0] - offset_pose.x;
            Hover_position[1] = uav_pos[1] - offset_pose.y;
            Hover_position[2] = uav_pos[2];
            Hover_yaw = uav_yaw;
        }
        pos_des << Hover_position;
        vel_des << 0.0, 0.0, 0.0;
        acc_des << 0.0, 0.0, 0.0;
        yaw_des = Hover_yaw;
    }
    else if (uav_command.Agent_CMD == prometheus_msgs::UAVCommand::Land)
    {
        //【Land】 降落，直接使用LAND_CONTROL
        control_state = CONTROL_STATE::LAND_CONTROL;
    }
    else if (uav_command.Agent_CMD == prometheus_msgs::UAVCommand::Move)
    {
        //【Move】 移动，移动子模式的区别详见UAVCommand.msg中的说明
        if (uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_POS)
        {
            // 【XYZ_POS】XYZ惯性系定点控制
            pos_des[0] = uav_command.position_ref[0] - offset_pose.x;
            pos_des[1] = uav_command.position_ref[1] - offset_pose.y;
            pos_des[2] = uav_command.position_ref[2];
            vel_des << 0.0, 0.0, 0.0;
            acc_des << 0.0, 0.0, 0.0;
            yaw_des = uav_command.yaw_ref;
        }
        else if (uav_command.Move_mode == prometheus_msgs::UAVCommand::XY_VEL_Z_POS)
        {
            // 【XYZ_POS】Z轴定高，XY速度控制
            pos_des[0] = 0.0;
            pos_des[1] = 0.0;
            pos_des[2] = uav_command.position_ref[2];
            vel_des[0] = uav_command.velocity_ref[0];
            vel_des[1] = uav_command.velocity_ref[1];
            vel_des[2] = 0.0;
            acc_des << 0.0, 0.0, 0.0;
            yaw_des = uav_command.yaw_ref;
        }
        else if (uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_VEL)
        {
            pos_des << 0.0, 0.0, 0.0;
            vel_des[0] = uav_command.velocity_ref[0];
            vel_des[1] = uav_command.velocity_ref[1];
            vel_des[2] = uav_command.velocity_ref[2];
            acc_des << 0.0, 0.0, 0.0;
            yaw_rate_des = uav_command.yaw_rate_ref;
            yaw_des = uav_command.yaw_ref;
        }
        else if (uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_POS_BODY)
        {
            // 【XYZ_POS_BODY】XYZ位置转换为惯性系，偏航角固定
            // 机体系的定点控制，必须使得Command_ID递增，否则无人机会持续移动
            if (uav_command.Command_ID > uav_command_last.Command_ID)
            {
                float d_pos_body[2] = {uav_command.position_ref[0], uav_command.position_ref[1]};
                float d_pos_enu[2];
                rotation_yaw(uav_yaw, d_pos_body, d_pos_enu);

                uav_command.position_ref[0] = uav_pos[0] + d_pos_enu[0];
                uav_command.position_ref[1] = uav_pos[1] + d_pos_enu[1];
                uav_command.position_ref[2] = uav_pos[2] + uav_command.position_ref[2];
                pos_des[0] = uav_command.position_ref[0] - offset_pose.x;
                pos_des[1] = uav_command.position_ref[1] - offset_pose.y;
                pos_des[2] = uav_command.position_ref[2];
                vel_des << 0.0, 0.0, 0.0;
                acc_des << 0.0, 0.0, 0.0;
                yaw_des = uav_command.yaw_ref;
            }
        }
        else if (uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_VEL_BODY)
        {
            if(uav_command.Command_ID > uav_command_last.Command_ID)
            {
                float d_vel_body[2] = {uav_command.velocity_ref[0], uav_command.velocity_ref[1]};
                float d_vel_enu[2];
                rotation_yaw(uav_yaw, d_vel_body, d_vel_enu);
                uav_command.velocity_ref[0] = d_vel_enu[0];
                uav_command.velocity_ref[1] = d_vel_enu[1];
                pos_des[0] = 0.0;
                pos_des[1] = 0.0;
                pos_des[2] = 0.0;
                vel_des[0] = uav_command.velocity_ref[0];
                vel_des[1] = uav_command.velocity_ref[1];
                vel_des[2] = uav_command.velocity_ref[2];
                acc_des << 0.0, 0.0, 0.0;
                yaw_rate_des = uav_command.yaw_rate_ref;
                yaw_des = uav_command.yaw_ref;
            }
        }
        else if (uav_command.Move_mode == prometheus_msgs::UAVCommand::XY_VEL_Z_POS_BODY)
        {
            if(uav_command.Command_ID > uav_command_last.Command_ID)
            {
                // 【XY_VEL_Z_POS_BODY】Z轴定高，偏航角固定，XY速度转换为惯性系
                float d_vel_body[2] = {uav_command.velocity_ref[0], uav_command.velocity_ref[1]};
                float d_vel_enu[2];
                rotation_yaw(uav_yaw, d_vel_body, d_vel_enu);
                uav_command.velocity_ref[0] = d_vel_enu[0];
                uav_command.velocity_ref[1] = d_vel_enu[1];
                pos_des[0] = 0.0;
                pos_des[1] = 0.0;
                pos_des[2] = uav_pos[2] + uav_command.position_ref[2];
                vel_des[0] = uav_command.velocity_ref[0];
                vel_des[1] = uav_command.velocity_ref[1];
                vel_des[2] = 0.0;
                acc_des << 0.0, 0.0, 0.0;
                yaw_des = uav_command.yaw_ref;
            }
            
        }
        else if (uav_command.Move_mode == prometheus_msgs::UAVCommand::TRAJECTORY)
        {
            // 【TRAJECTORY】轨迹控制，输入为期望位置、速度、加速度，其中速度和加速度可缺省（降级为定点控制）
            for (int i = 0; i < 3; i++)
            {
                pos_des(i) = uav_command.position_ref[i];
                vel_des(i) = uav_command.velocity_ref[i];
                acc_des(i) = uav_command.acceleration_ref[i];
            }
            yaw_des = uav_command.yaw_ref;
        }
        else if (uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_ATT)
        {
            // 【XYZ_ATT】姿态直接控制，必须先将enable_external_control设置为true
            if (enable_external_control)
            {
                u_att[0] = uav_command.att_ref[0];
                u_att[1] = uav_command.att_ref[1];
                u_att[2] = uav_command.att_ref[2];
                u_att[3] = uav_command.att_ref[3];
            }
            else
            {
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
                pos_des << Takeoff_position + Eigen::Vector3d(0, 0, Takeoff_height);
                vel_des << 0.0, 0.0, 0.0;
                acc_des << 0.0, 0.0, 0.0;
                yaw_des = uav_command.yaw_ref;
                cout << RED << node_name << "Pls set enable_external_control to true, reset to Init_Pos_Hover!" << TAIL << endl;
            }
        }
        else if (uav_command.Move_mode == prometheus_msgs::UAVCommand::LAT_LON_ALT)
        {
            global_pos_des[0] = uav_command.latitude;
            global_pos_des[1] = uav_command.longitude;
            global_pos_des[2] = uav_command.altitude;
            yaw_des = uav_command.yaw_ref;
        }
        else
        {
            cout << RED << node_name << "Wrong command!" << TAIL << endl;
        }
    }

    // 记录上一时刻命令
    uav_command_last = uav_command;
}

void UAV_controller::set_command_des_for_pos_controller()
{
    if (uav_command.Agent_CMD == prometheus_msgs::UAVCommand::Init_Pos_Hover)
    {
        //【Init_Pos_Hover】 移动到指定起飞位置
        pos_des << Takeoff_position + Eigen::Vector3d(0, 0, Takeoff_height);
        vel_des << 0.0, 0.0, 0.0;
        acc_des << 0.0, 0.0, 0.0;
        yaw_des = uav_command.yaw_ref;
    }
    else if (uav_command.Agent_CMD == prometheus_msgs::UAVCommand::Land)
    {
        //【Land】 降落，直接使用LAND_CONTROL
        control_state = CONTROL_STATE::LAND_CONTROL;
    }
    else if (uav_command.Agent_CMD == prometheus_msgs::UAVCommand::Move)
    {
        //【Move】 移动，移动子模式的区别详见UAVCommand.msg中的说明
        if (uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_POS)
        {
            pos_des[0] = uav_command.position_ref[0] - offset_pose.x;
            pos_des[1] = uav_command.position_ref[1] - offset_pose.y;
            pos_des[2] = uav_command.position_ref[2];
            vel_des << 0.0, 0.0, 0.0;
            acc_des << 0.0, 0.0, 0.0;
            yaw_des = uav_command.yaw_ref;
        }
        else if (uav_command.Move_mode == prometheus_msgs::UAVCommand::TRAJECTORY)
        {
            // 【TRAJECTORY】轨迹控制，输入为期望位置、速度、加速度，其中速度和加速度可缺省（降级为定点控制）
            for (int i = 0; i < 3; i++)
            {
                pos_des(i) = uav_command.position_ref[i];
                vel_des(i) = uav_command.velocity_ref[i];
                acc_des(i) = uav_command.acceleration_ref[i];
            }
            yaw_des = uav_command.yaw_ref;
        }
        else
        {
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
            cout << RED << node_name << "Pls set pos_controller to PX4_ORIGIN, reset to Current_Pos_Hover!" << TAIL << endl;
        }
    }
    else if (uav_command.Agent_CMD == prometheus_msgs::UAVCommand::Current_Pos_Hover)
    {
        // 【Current_Pos_Hover】 悬停。当前位置悬停
        if (uav_command_last.Agent_CMD != prometheus_msgs::UAVCommand::Current_Pos_Hover)
        {
            Hover_position[0] = uav_pos[0] - offset_pose.x;
            Hover_position[1] = uav_pos[1] - offset_pose.y;
            Hover_position[2] = uav_pos[2];
            Hover_yaw = uav_yaw;
        }
        pos_des << Hover_position;
        vel_des << 0.0, 0.0, 0.0;
        acc_des << 0.0, 0.0, 0.0;
        yaw_des = Hover_yaw;
    }

    // 记录上一时刻命令
    uav_command_last = uav_command;
}

void UAV_controller::uav_cmd_cb(const prometheus_msgs::UAVCommand::ConstPtr &msg)
{
    if(control_state != CONTROL_STATE::COMMAND_CONTROL)
    {
        // 非COMMAND_CONTROL模式，不接收uav_command信息，并且设置初始指令为Init_Pos_Hover
        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
        return;
    }

    uav_command = *msg;
}

void UAV_controller::send_pos_cmd_to_px4_original_controller()
{
    // RC_POS_CONTROL
    if (control_state == CONTROL_STATE::RC_POS_CONTROL)
    {
        send_pos_setpoint(pos_des, yaw_des);
        return;
    }

    if (control_state == CONTROL_STATE::LAND_CONTROL && uav_state.mode == "OFFBOARD" )
    {
        if (quick_land)
        {
            // quick_land一般用于位置失效情况，因此直接使用速度控制
            send_vel_setpoint(vel_des, yaw_des);
        }
        else
        {
            send_pos_vel_xyz_setpoint(pos_des, vel_des, yaw_des);
        }
        return;
    }

    if (control_state == CONTROL_STATE::COMMAND_CONTROL)
    {
        if (uav_command.Agent_CMD == prometheus_msgs::UAVCommand::Init_Pos_Hover ||
            uav_command.Agent_CMD == prometheus_msgs::UAVCommand::Current_Pos_Hover)
        {
            send_pos_setpoint(pos_des, yaw_des);
        }
        else if (uav_command.Agent_CMD == prometheus_msgs::UAVCommand::Move)
        {
            if (uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_POS ||
                uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_POS_BODY)
            {
                send_pos_setpoint(pos_des, yaw_des);
            }
            else if (uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_VEL ||
                     uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_VEL_BODY)
            {
                if (uav_command.Yaw_Rate_Mode)
                {
                    send_vel_setpoint_yaw_rate(vel_des, yaw_rate_des);
                }
                else
                {
                    send_vel_setpoint(vel_des, yaw_des);
                }
            }
            else if (uav_command.Move_mode == prometheus_msgs::UAVCommand::XY_VEL_Z_POS ||
                     uav_command.Move_mode == prometheus_msgs::UAVCommand::XY_VEL_Z_POS_BODY)
            {
                if (uav_command.Yaw_Rate_Mode)
                {
                    send_vel_xy_pos_z_setpoint_yaw_rate(pos_des, vel_des, yaw_rate_des);
                }
                else
                {
                    send_vel_xy_pos_z_setpoint(pos_des, vel_des, yaw_des);
                }
            }
            else if (uav_command.Move_mode == prometheus_msgs::UAVCommand::TRAJECTORY)
            {
                send_pos_vel_xyz_setpoint(pos_des, vel_des, yaw_des);
            }
            else if (uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_ATT)
            {
                // 此处为使用外部发布的姿态期望值
                send_attitude_setpoint(u_att);
            }
            else if (uav_command.Move_mode == prometheus_msgs::UAVCommand::LAT_LON_ALT)
            {
                send_global_setpoint(global_pos_des, yaw_des);
            }
        }
        return;
    }
}

void UAV_controller::uav_state_cb(const prometheus_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;

    uav_pos = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
    uav_vel = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);

    uav_quat.w() = msg->attitude_q.w;
    uav_quat.x() = msg->attitude_q.x;
    uav_quat.y() = msg->attitude_q.y;
    uav_quat.z() = msg->attitude_q.z;

    uav_yaw = geometry_utils::get_yaw_from_quaternion(uav_quat);

    // 将无人机解锁位置设定为起飞点
    if (uav_state.armed && !uav_state_last.armed)
    {
        Takeoff_position[0] = uav_pos[0] - offset_pose.x;
        Takeoff_position[1] = uav_pos[1] - offset_pose.y;
        Takeoff_position[2] = uav_pos[2];
        Takeoff_yaw = uav_yaw;
        cout << GREEN << "Set Takeoff_position [X Y Z] : " << Takeoff_position[0] << " [ m ] " << Takeoff_position[1] << " [ m ] " << Takeoff_position[2] << " [ m ] " << TAIL << endl;
        cout << GREEN << "Set Takeoff_yaw : " << Takeoff_yaw/3.1415926*180 << " [ deg ] " << TAIL << endl;
    }

    uav_state_last = uav_state;
}

void UAV_controller::px4_rc_cb(const mavros_msgs::RCIn::ConstPtr &msg)
{
    // 调用外部函数对遥控器数据进行处理，具体见rc_data.h，此时rc_input中的状态已更新
    rc_input.handle_rc_data(msg);

    // 重启PX4飞控，条件: 无人机已上锁
    if (rc_input.toggle_reboot && !uav_state.armed)
    {
        rc_input.toggle_reboot = false;
        reboot_PX4();
        return;
    }

    // 紧急KILL指令
    if (rc_input.toggle_kill)
    {
        rc_input.toggle_kill = false;
        enable_emergency_func();
        return;
    }

    // 解锁，条件: 无人机已上锁
    if (rc_input.toggle_arm)
    {
        rc_input.toggle_arm = false;
        arm_disarm_func(true);
        return;
    }

    // 上锁，条件: 无人机已解锁
    // PX4代码规定:无人机必须处于地面时才可以上锁
    if (rc_input.toggle_disarm)
    {
        rc_input.toggle_arm = false;
        arm_disarm_func(false);
        return;
    }

    // 自动降落，条件: 必须在HOVER_CONTROL或者COMMAND_CONTROL模式才可以触发
    bool if_in_hover_or_command_mode =
        control_state == CONTROL_STATE::RC_POS_CONTROL || control_state == CONTROL_STATE::COMMAND_CONTROL;
    if (rc_input.toggle_land && if_in_hover_or_command_mode)
    {
        rc_input.toggle_land = false;
        control_state = CONTROL_STATE::LAND_CONTROL;
        set_landing_des = false;
        return;
    }

    // 如果无人机没有解锁，则不需要判断模式切换指令，直接返回
    if (!uav_state.armed)
    {
        return;
    }

    // 如果无人机处于LAND_CONTROL下，单独判断无人机模式切换指令
    if (control_state == CONTROL_STATE::LAND_CONTROL)
    {
        // 收到进入INIT指令，且不在INIT模式时
        if (rc_input.enter_init)
        {
            rc_input.enter_init = false;
            control_state = CONTROL_STATE::INIT;
            cout << GREEN << node_name << " Switch to INIT" << TAIL << endl;
        }

        if (rc_input.enter_rc_pos_control)
        {
            rc_input.enter_rc_pos_control = false;
            // odom失效，拒绝进入
            if (!uav_state.odom_valid)
            {
                this->text_info.MessageType = prometheus_msgs::TextInfo::ERROR;
                this->text_info.Message = "Reject RC_POS_CONTROL. Odom invalid!";
                cout << RED << node_name << " Reject RC_POS_CONTROL. Odom invalid! " << TAIL << endl;
                return;
            }
            // 切换至HOVER_CONTROL
            control_state = CONTROL_STATE::RC_POS_CONTROL;
            // 初始化默认的UAVCommand
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
            // 进入HOVER_CONTROL，需设置初始悬停点
            set_hover_pose_with_odom();
            cout << GREEN << node_name << " Switch to RC_POS_CONTROL" << TAIL << endl;
            return;
        }

        return;
    }

    // 收到进入INIT指令，且不在INIT模式时
    if (rc_input.enter_init && control_state != CONTROL_STATE::INIT)
    {
        rc_input.enter_init = false;
        control_state = CONTROL_STATE::INIT;
        cout << GREEN << node_name << " Switch to INIT" << TAIL << endl;
    }

    if (rc_input.enter_rc_pos_control && control_state != CONTROL_STATE::RC_POS_CONTROL)
    {
        rc_input.enter_rc_pos_control = false;
        // odom失效，拒绝进入
        if (!uav_state.odom_valid)
        {
            this->text_info.MessageType = prometheus_msgs::TextInfo::ERROR;
            this->text_info.Message = "Reject RC_POS_CONTROL. Odom invalid!";
            cout << RED << node_name << " Reject RC_POS_CONTROL. Odom invalid! " << TAIL << endl;
            return;
        }
        // 切换至HOVER_CONTROL
        control_state = CONTROL_STATE::RC_POS_CONTROL;
        // 初始化默认的UAVCommand
        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
        // 进入HOVER_CONTROL，需设置初始悬停点
        set_hover_pose_with_odom();
        cout << GREEN << node_name << " Switch to RC_POS_CONTROL" << TAIL << endl;
        return;
    }

    // 必须从HOVER_CONTROL模式切入（确保odom和offboard模式正常）
    if (rc_input.enter_command_control && control_state == CONTROL_STATE::RC_POS_CONTROL && uav_state.mode == "OFFBOARD")
    {
        // 标志位重置
        rc_input.enter_command_control = false;
        // 切换至COMMAND_CONTROL
        control_state = CONTROL_STATE::COMMAND_CONTROL;
        // 初始化默认的UAVCommand
        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
        cout << GREEN << node_name << " Switch to COMMAND_CONTROL" << TAIL << endl;
        return;
    }
}

int UAV_controller::check_failsafe()
{
    // 一般不会出现，除非发送了重启飞控指令，或者飞控连接线物理断裂
    if (!uav_state.connected)
    {
        cout << RED << uav_name << ":----> Failsafe: Waiting for PX4 connection！" << TAIL << endl;
        return -1;
    }

    if (uav_state.position[0] < uav_geo_fence.x_min || uav_state.position[0] > uav_geo_fence.x_max ||
        uav_state.position[1] < uav_geo_fence.y_min || uav_state.position[1] > uav_geo_fence.y_max ||
        uav_state.position[2] < uav_geo_fence.z_min || uav_state.position[2] > uav_geo_fence.z_max)
    {
        cout << RED << uav_name << ":----> Failsafe: Out of the geo fence, swtich to land control mode！" << TAIL << endl;
        return 1;
    }
    else if (!uav_state.odom_valid)
    {
        cout << RED << uav_name << ":----> Failsafe: Odom invalid, swtich to land control mode!" << TAIL << endl;
        return 2;
    }
    else
    {
        return 0;
    }
}

void UAV_controller::uav_setup_cb(const prometheus_msgs::UAVSetup::ConstPtr &msg)
{
    if (msg->cmd == prometheus_msgs::UAVSetup::ARMING)
    {
        arm_disarm_func(msg->arming);
    }
    else if (msg->cmd == prometheus_msgs::UAVSetup::SET_PX4_MODE)
    {
        set_px4_mode_func(msg->px4_mode);
    }
    else if (msg->cmd == prometheus_msgs::UAVSetup::REBOOT_PX4)
    {
        reboot_PX4();
    }
    else if (msg->cmd == prometheus_msgs::UAVSetup::SET_CONTROL_MODE)
    {
        // todo more test
        if (msg->control_state == "COMMAND_CONTROL" && uav_state.armed)
        {
            cout << GREEN << node_name << " Switch to COMMAND_CONTROL by uav_setup cmd" << TAIL << endl;
            control_state = CONTROL_STATE::COMMAND_CONTROL;
        }
    }
}

void UAV_controller::send_idle_cmd()
{
    mavros_msgs::PositionTarget pos_setpoint;
    //飞控如何接收该信号请见mavlink_receiver.cpp
    //飞控如何执行该指令请见FlightTaskOffboard.cpp
    pos_setpoint.type_mask = 0x4000;
    px4_setpoint_raw_local_pub.publish(pos_setpoint);
}

//发送位置期望值至飞控（输入: 期望xyz,期望yaw）
void UAV_controller::send_pos_setpoint(const Eigen::Vector3d &pos_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    // Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    // Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    // Bit 10 should set to 0, means is not force sp
    // pos_setpoint.type_mask = 0b100111111000; // 100 111 111 000  xyz + yaw
    pos_setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                             mavros_msgs::PositionTarget::IGNORE_VY |
                             mavros_msgs::PositionTarget::IGNORE_VZ |
                             mavros_msgs::PositionTarget::IGNORE_AFX |
                             mavros_msgs::PositionTarget::IGNORE_AFY |
                             mavros_msgs::PositionTarget::IGNORE_AFZ |
                             mavros_msgs::PositionTarget::IGNORE_AFY |
                             mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    pos_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];
    pos_setpoint.yaw = yaw_sp;
    px4_setpoint_raw_local_pub.publish(pos_setpoint);
}

//发送速度期望值至飞控（输入: 期望vxvyvz,期望yaw）
void UAV_controller::send_vel_setpoint(const Eigen::Vector3d &vel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    pos_setpoint.type_mask = 0b100111000111;
    pos_setpoint.coordinate_frame = 1;
    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = vel_sp[2];
    pos_setpoint.yaw = yaw_sp;
    px4_setpoint_raw_local_pub.publish(pos_setpoint);
}

//发送速度期望值至飞控（输入: 期望vxvyvz,期望yaw_rate）
void UAV_controller::send_vel_setpoint_yaw_rate(const Eigen::Vector3d &vel_sp, float yaw_rate_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    pos_setpoint.type_mask = 0b010111000111;
    pos_setpoint.coordinate_frame = 1;
    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = vel_sp[2];
    pos_setpoint.yaw_rate = yaw_rate_sp;
    px4_setpoint_raw_local_pub.publish(pos_setpoint);
}

void UAV_controller::send_vel_xy_pos_z_setpoint(const Eigen::Vector3d &pos_sp, const Eigen::Vector3d &vel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    // 此处由于飞控暂不支持位置－速度追踪的复合模式，因此type_mask设定如下
    pos_setpoint.type_mask = 0b100111000011; // 100 111 000 011  vx vy vz z + yaw
    pos_setpoint.coordinate_frame = 1;
    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = 0.0;
    pos_setpoint.position.z = pos_sp[2];
    pos_setpoint.yaw = yaw_sp;
    px4_setpoint_raw_local_pub.publish(pos_setpoint);
}

void UAV_controller::send_vel_xy_pos_z_setpoint_yaw_rate(const Eigen::Vector3d &pos_sp, const Eigen::Vector3d &vel_sp, float yaw_rate_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    // 此处由于飞控暂不支持位置－速度追踪的复合模式，因此type_mask设定如下
    pos_setpoint.type_mask = 0b010111000011; // 010 111 000 011  vx vy vz z + yaw_rate
    pos_setpoint.coordinate_frame = 1;
    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = 0.0;
    pos_setpoint.position.z = pos_sp[2];
    pos_setpoint.yaw_rate = yaw_rate_sp;
    px4_setpoint_raw_local_pub.publish(pos_setpoint);
}

void UAV_controller::send_pos_vel_xyz_setpoint(const Eigen::Vector3d &pos_sp, const Eigen::Vector3d &vel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    // 速度作为前馈项， 参见FlightTaskOffboard.cpp
    // 控制方法请见 PositionControl.cpp
    pos_setpoint.type_mask = 0b100111000000; // 100 111 000 000  vx vy　vz x y z+ yaw
    pos_setpoint.coordinate_frame = 1;
    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];
    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = vel_sp[2];
    pos_setpoint.yaw = yaw_sp;
    px4_setpoint_raw_local_pub.publish(pos_setpoint);
}

//发送加速度期望值至飞控（输入: 期望axayaz,期望yaw）
void UAV_controller::send_acc_xyz_setpoint(const Eigen::Vector3d &accel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    pos_setpoint.type_mask = 0b100000111111;
    pos_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    pos_setpoint.acceleration_or_force.x = accel_sp[0];
    pos_setpoint.acceleration_or_force.y = accel_sp[1];
    pos_setpoint.acceleration_or_force.z = accel_sp[2];
    pos_setpoint.yaw = yaw_sp;
    px4_setpoint_raw_local_pub.publish(pos_setpoint);
}

// 发送角度期望值至飞控（输入: 期望角度-四元数,期望推力）
void UAV_controller::send_attitude_setpoint(Eigen::Vector4d &u_att)
{
    mavros_msgs::AttitudeTarget att_setpoint;
    // Mappings: If any of these bits are set, the corresponding input should be ignored:
    // bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude
    //  0b00000111;
    att_setpoint.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                             mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                             mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

    Eigen::Vector3d att_des;
    att_des << u_att(0), u_att(1), u_att(2);
    Eigen::Quaterniond q_des = quaternion_from_rpy(att_des);
    att_setpoint.orientation.x = q_des.x();
    att_setpoint.orientation.y = q_des.y();
    att_setpoint.orientation.z = q_des.z();
    att_setpoint.orientation.w = q_des.w();
    att_setpoint.thrust = u_att(3);
    px4_setpoint_raw_attitude_pub.publish(att_setpoint);
}

// 发送经纬度以及高度期望值至飞控(输入,期望lat/lon/alt,期望yaw)
void UAV_controller::send_global_setpoint(const Eigen::Vector3d &global_pos_sp, float yaw_sp)
{
    mavros_msgs::GlobalPositionTarget global_setpoint;
    //该话题支持三个坐标系:
    //FRAME_GLOBAL_INT 高度数据为海拔高度
    //FRAME_GLOBAL_REL_ALT 高度数据为相对起始位置的高度,HOME点的高度为0
    //FRAME_GLOBAL_TERRAIN_ALT 具有 AGL 高度的全球 (WGS84) 坐标系（在航路点坐标处）。第一个值/x：以度为单位的纬度，第二个值/y：以度为单位的经度，第三个值/z：以米为单位的正高度，0 表示地形模型中的地平面。
    //在仿真中测试使用后,FRAME_GLOBAL_REL_ALT使用比较方便推荐使用该坐标系
    //https://mavlink.io/en/messages/common.html#MAV_FRAME_GLOBAL_INT  坐标系的详细介绍
    global_setpoint.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
    global_setpoint.type_mask = 0b100111111000;
    global_setpoint.latitude = global_pos_sp[0];
    global_setpoint.longitude = global_pos_sp[1];
    global_setpoint.altitude = global_pos_sp[2];
    global_setpoint.yaw = yaw_sp;
    px4_setpoint_raw_global_pub.publish(global_setpoint);
}

// 【坐标系旋转函数】- 机体系到enu系
// body_frame是机体系,enu_frame是惯性系，yaw_angle是当前偏航角[rad]
void UAV_controller::rotation_yaw(double yaw_angle, float body_frame[2], float enu_frame[2])
{
    enu_frame[0] = body_frame[0] * cos(yaw_angle) - body_frame[1] * sin(yaw_angle);
    enu_frame[1] = body_frame[0] * sin(yaw_angle) + body_frame[1] * cos(yaw_angle);
}

void UAV_controller::printf_control_state()
{
    cout << GREEN << ">>>>>>>>>>>>>>>>>>>> UAV [" << uav_id << "] Controller  <<<<<<<<<<<<<<<<<<" << TAIL << endl;

    //固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为n位
    cout << setprecision(NUM_POINT);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    // 打印 control_state
    switch (control_state)
    {
    case CONTROL_STATE::INIT:
        cout << GREEN << "CONTROL_STATE: [ INIT ] " << TAIL << endl;
        break;

    case CONTROL_STATE::RC_POS_CONTROL:
        cout << GREEN << "CONTROL_STATE: [ RC_POS_CONTROL ] " << TAIL << endl;
        cout << GREEN << "Hover_Pos [X Y Z] : " << Hover_position[0] << " [ m ] " << Hover_position[1] << " [ m ] " << Hover_position[2] << " [ m ] " << TAIL << endl;
        break;

    case CONTROL_STATE::COMMAND_CONTROL:
        cout << GREEN << "CONTROL_STATE: [ COMMAND_CONTROL ] " << TAIL << endl;
        break;
    case CONTROL_STATE::LAND_CONTROL:
        if (quick_land)
        {
            cout << GREEN << "CONTROL_STATE: [ LAND_CONTROL ] - quick land mode " << TAIL << endl;
        }
        else
        {
            cout << GREEN << "CONTROL_STATE: [ LAND_CONTROL ] " << TAIL << endl;
        }
        break;
    }

    // 打印控制器选择信息
    switch (pos_controller)
    {
    case POS_CONTOLLER::PX4_ORIGIN:
        cout << GREEN << "Controller  : [ PX4_ORIGIN ] " << TAIL << endl;
        break;

    case POS_CONTOLLER::PID:
        cout << GREEN << "Controller  : [ PID ] " << TAIL << endl;
        pos_controller_pid.printf_result();
        break;
    case POS_CONTOLLER::UDE:
        cout << GREEN << "Controller  : [ UDE ] " << TAIL << endl;
        pos_controller_ude.printf_result();
        break;
    case POS_CONTOLLER::NE:
        cout << GREEN << "Controller  : [ NE ] " << TAIL << endl;
        pos_controller_ne.printf_result();
        break;
    }

    // 打印 指令信息
    if (control_state == CONTROL_STATE::COMMAND_CONTROL)
    {
        switch (uav_command.Agent_CMD)
        {
        case prometheus_msgs::UAVCommand::Init_Pos_Hover:
            cout << GREEN << "Command: [ Init_Pos_Hover ] " << TAIL << endl;
            cout << GREEN << "Init_Pos [X Y Z] : " << Takeoff_position[0] << " [ m ] " << Takeoff_position[1] << " [ m ] " << Takeoff_position[2] + Takeoff_height << " [ m ] " << TAIL << endl;
            break;

        case prometheus_msgs::UAVCommand::Current_Pos_Hover:
            cout << GREEN << "Command: [ Current_Pos_Hover ] " << TAIL << endl;
            cout << GREEN << "Hover_Pos [X Y Z] : " << Hover_position[0] << " [ m ] " << Hover_position[1] << " [ m ] " << Hover_position[2] << " [ m ] " << TAIL << endl;
            break;

        case prometheus_msgs::UAVCommand::Land:
            cout << GREEN << "Command: [ Land ] " << TAIL << endl;
            break;

        case prometheus_msgs::UAVCommand::Move:

            if (uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_POS)
            {
                cout << GREEN << "Command: [ Move in XYZ_POS ] " << TAIL << endl;
                cout << GREEN << "Pos_ref [X Y Z] : " << uav_command.position_ref[0] << " [ m ] " << uav_command.position_ref[1] << " [ m ] " << uav_command.position_ref[2] << " [ m ] " << TAIL << endl;
                cout << GREEN << "Yaw_ref : " << uav_command.yaw_ref * 180 / M_PI << " [deg] " << TAIL << endl;
            }
            else if (uav_command.Move_mode == prometheus_msgs::UAVCommand::XY_VEL_Z_POS)
            {
                cout << GREEN << "Command: [ Move in XY_VEL_Z_POS ] " << TAIL << endl;
                cout << GREEN << "Pos_ref [    Z] : " << uav_command.position_ref[2] << " [ m ] " << TAIL << endl;
                cout << GREEN << "Vel_ref [X Y  ] : " << uav_command.velocity_ref[0] << " [m/s] " << uav_command.velocity_ref[1] << " [m/s] " << TAIL << endl;
                cout << GREEN << "Yaw_ref : " << uav_command.yaw_ref * 180 / M_PI << " [deg] " << TAIL << endl;
            }
            else if (uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_VEL)
            {
                cout << GREEN << "Command: [ Move in XYZ_VEL ] " << TAIL << endl;
                cout << GREEN << "Vel_ref [X Y Z] : " << uav_command.velocity_ref[0] << " [m/s] " << uav_command.velocity_ref[1] << " [m/s] " << uav_command.velocity_ref[2] << " [m/s] " << TAIL << endl;
                cout << GREEN << "Yaw_ref : " << uav_command.yaw_ref * 180 / M_PI << " [deg] " << TAIL << endl;
            }
            else if (uav_command.Move_mode == prometheus_msgs::UAVCommand::TRAJECTORY)
            {
                cout << GREEN << "Command: [ Move in TRAJECTORY ] " << TAIL << endl;
                cout << GREEN << "Pos_ref [X Y Z] : " << uav_command.position_ref[0] << " [ m ] " << uav_command.position_ref[1] << " [ m ] " << uav_command.position_ref[2] << " [ m ] " << TAIL << endl;
                cout << GREEN << "Vel_ref [X Y Z] : " << uav_command.velocity_ref[0] << " [m/s] " << uav_command.velocity_ref[1] << " [m/s] " << uav_command.velocity_ref[2] << " [m/s] " << TAIL << endl;
                cout << GREEN << "Yaw_ref : " << uav_command.yaw_ref * 180 / M_PI << " [deg] " << TAIL << endl;
            }
            else if (uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_POS_BODY)
            {
                cout << GREEN << "Command: [ Move in XYZ_POS_BODY ] " << TAIL << endl;
                cout << GREEN << "Pos_ref [X Y Z] : " << uav_command.position_ref[0] << " [ m ] " << uav_command.position_ref[1] << " [ m ] " << uav_command.position_ref[2] << " [ m ] " << TAIL << endl;
                cout << GREEN << "Yaw_ref : " << uav_command.yaw_ref * 180 / M_PI << " [deg] " << TAIL << endl;
            }
            else if (uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_VEL_BODY)
            {
                cout << GREEN << "Command: [ Move in XYZ_VEL_BODY ] " << TAIL << endl;
                cout << GREEN << "Vel_ref [X Y Z] : " << uav_command.velocity_ref[0] << " [m/s] " << uav_command.velocity_ref[1] << " [m/s] " << uav_command.velocity_ref[2] << " [m/s] " << TAIL << endl;
                cout << GREEN << "Yaw_ref : " << uav_command.yaw_ref * 180 / M_PI << " [deg] " << TAIL << endl;
            }
            else if (uav_command.Move_mode == prometheus_msgs::UAVCommand::XY_VEL_Z_POS_BODY)
            {
                cout << GREEN << "Command: [ Move in XY_VEL_Z_POS_BODY ] " << TAIL << endl;
                cout << GREEN << "Pos_ref [    Z] : " << uav_command.position_ref[2] << " [ m ] " << TAIL << endl;
                cout << GREEN << "Vel_ref [X Y  ] : " << uav_command.velocity_ref[0] << " [m/s] " << uav_command.velocity_ref[1] << " [m/s] " << TAIL << endl;
                cout << GREEN << "Yaw_ref : " << uav_command.yaw_ref * 180 / M_PI << " [deg] " << TAIL << endl;
            }
            else if (uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_ATT)
            {
                cout << YELLOW << node_name << " Send control cmd from EXTERNAL_CONTROLLER, be careful! " << TAIL << endl;
                cout << GREEN << "Command: [ Move in XYZ_ATT ] " << TAIL << endl;
                cout << GREEN << "Att_ref [X Y Z] : " << uav_command.att_ref[0] * 180 / M_PI << " [deg] " << uav_command.att_ref[1] * 180 / M_PI << " [deg] " << uav_command.att_ref[2] * 180 / M_PI << " [deg] " << TAIL << endl;
                cout << GREEN << "Thrust_ref[0-1] : " << uav_command.att_ref[3] << TAIL << endl;
            }
            else if (uav_command.Move_mode == prometheus_msgs::UAVCommand::LAT_LON_ALT)
            {
                cout << GREEN << "Command: [ Move in LAT_LON_ALT ] " << TAIL << endl;
                cout << GREEN << "LAT : " << uav_command.latitude << " LON :  " << uav_command.longitude << " ALT : " << uav_command.altitude << TAIL << endl;
                cout << GREEN << "Yaw_ref : " << uav_command.yaw_ref * 180 / M_PI << " [deg] " << TAIL << endl;
            }
            else
            {
                cout << GREEN << "Command: [ Unknown Mode ]. " << TAIL << endl;
            }
            break;
        }
    }

    // 打印PX4回传信息用于验证
    if (pos_controller == POS_CONTOLLER::PX4_ORIGIN)
    {
        if (enable_external_control)
        {
            cout << GREEN << "Att_target [X Y Z] : " << px4_att_target[0] * 180 / M_PI << " [deg] " << px4_att_target[1] * 180 / M_PI << " [deg] " << px4_att_target[2] * 180 / M_PI << " [deg] " << TAIL << endl;
            cout << GREEN << "Thr_target [ 0-1 ] : " << px4_thrust_target << TAIL << endl;
        }
        else
        {
            cout << GREEN << "Pos_target [X Y Z] : " << px4_pos_target[0] << " [ m ] " << px4_pos_target[1] << " [ m ] " << px4_pos_target[2] << " [ m ] " << TAIL << endl;
            cout << GREEN << "Vel_target [X Y Z] : " << px4_vel_target[0] << " [m/s] " << px4_vel_target[1] << " [m/s] " << px4_vel_target[2] << " [m/s] " << TAIL << endl;
            cout << GREEN << "Yaw_target         : " << px4_att_target[2] * 180 / M_PI << " [deg] " << TAIL << endl;
            cout << GREEN << "Thr_target [ 0-1 ] : " << px4_thrust_target << TAIL << endl;
        }
    }
    else if (pos_controller == POS_CONTOLLER::PID ||
             pos_controller == POS_CONTOLLER::UDE ||
             pos_controller == POS_CONTOLLER::UDE)
    {
        cout << GREEN << "Att_target [R P Y] : " << px4_att_target[0] * 180 / M_PI << " [deg] " << px4_att_target[1] * 180 / M_PI << " [deg] " << px4_att_target[2] * 180 / M_PI << " [deg] " << TAIL << endl;
        cout << GREEN << "Thr_target [ 0-1 ] : " << px4_thrust_target << TAIL << endl;
    }
}

void UAV_controller::printf_param()
{
    cout << GREEN << ">>>>>>>>>>>>>>>> UAV controller Param <<<<<<<<<<<<<<<<" << TAIL << endl;

    cout << GREEN << "uav_id                    : " << uav_id << " " << TAIL << endl;
    cout << GREEN << "sim_mode                  : " << sim_mode << " " << TAIL << endl;
    cout << GREEN << "pos_controller            : " << pos_controller << TAIL << endl;
    cout << GREEN << "enable_external_control   : " << enable_external_control << TAIL << endl;
    cout << GREEN << "Takeoff_height            : " << Takeoff_height << " [m] " << TAIL << endl;
    cout << GREEN << "Disarm_height             : " << Disarm_height << " [m] " << TAIL << endl;
    cout << GREEN << "Land_speed                : " << Land_speed << " [m/s] " << TAIL << endl;
    cout << GREEN << "geo_fence_x : " << uav_geo_fence.x_min << " [m]  to  " << uav_geo_fence.x_min << " [m]" << TAIL << endl;
    cout << GREEN << "geo_fence_y : " << uav_geo_fence.y_min << " [m]  to  " << uav_geo_fence.y_max << " [m]" << TAIL << endl;
    cout << GREEN << "geo_fence_z : " << uav_geo_fence.z_min << " [m]  to  " << uav_geo_fence.z_max << " [m]" << TAIL << endl;
}

void UAV_controller::px4_pos_target_cb(const mavros_msgs::PositionTarget::ConstPtr &msg)
{
    px4_pos_target = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
    px4_vel_target = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    px4_acc_target = Eigen::Vector3d(msg->acceleration_or_force.x, msg->acceleration_or_force.y, msg->acceleration_or_force.z);
}

void UAV_controller::px4_att_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
    Eigen::Quaterniond px4_q_target = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    px4_att_target = quaternion_to_euler(px4_q_target);
    px4_rates_target = Eigen::Vector3d(msg->body_rate.x, msg->body_rate.y, msg->body_rate.z);
    px4_thrust_target = msg->thrust;
}

void UAV_controller::offset_pose_cb(const prometheus_msgs::OffsetPose::ConstPtr &msg)
{
    if (uav_state.location_source == prometheus_msgs::UAVState::GPS || uav_state.location_source == prometheus_msgs::UAVState::RTK)
    {
        offset_pose = *msg;
    }else
    {
        cout << RED << node_name << " get offset_pose, but wrong location_source!" << TAIL << endl;
    }
}

/***
 * 上锁解锁函数，调用mavros上锁和解决服务
 * 参数: bool on_or_off，true为解锁指令，false为上锁指令
 * 判断当前无人机状态，为解锁状态且on_or_off为
 * -----------------------------------------------------------------------|
 * |on_or_off/armed    |     true(无人机已解锁)      |   false（无人机未解锁） |
 * -----------------------------------------------------------------------|
 * |true（解锁指令）     |     无人机已经解锁           | 无人机正在解锁，解锁成功|
 * ----------------------------------------------------------------------|
 * |false（上锁指令）    |     无人机正在上锁，上锁成功  |   无人机已经上锁        |
 * -----------------------------------------------------------------------|
 */
void UAV_controller::arm_disarm_func(bool on_or_off)
{
    mavros_msgs::CommandBool arm_cmd;

    if (uav_state.armed)
    {
        if (!on_or_off)
        {
            arm_cmd.request.value = on_or_off;
            px4_arming_client.call(arm_cmd);
            if (arm_cmd.response.success)
            {
                cout << GREEN << node_name << " vehicle disarming, success!" << TAIL << endl;
            }
            else
            {
                this->text_info.MessageType = prometheus_msgs::TextInfo::ERROR;
                this->text_info.Message = "vehicle disarming, fail!";
                cout << RED << node_name << " vehicle disarming, fail!" << TAIL << endl;
            }
        }
        else
        {
            this->text_info.MessageType = prometheus_msgs::TextInfo::WARN;
            this->text_info.Message = "vehicle already armed";
            cout << YELLOW << node_name << " vehicle already armed" << TAIL << endl;
        }
    }
    else if (on_or_off)
    {
        arm_cmd.request.value = on_or_off;
        px4_arming_client.call(arm_cmd);
        if (arm_cmd.response.success)
        {
            cout << GREEN << node_name << "vehicle arming, success!" << TAIL << endl;
        }
        else
        {
            this->text_info.MessageType = prometheus_msgs::TextInfo::ERROR;
            this->text_info.Message = "vehicle arming, fail!";
            cout << RED << node_name << "vehicle arming, fail!" << TAIL << endl;
        }
    }
    else
    {
        this->text_info.MessageType = prometheus_msgs::TextInfo::WARN;
        this->text_info.Message = "vehicle already disarmed";
        cout << YELLOW << node_name << "vehicle already disarmed" << TAIL << endl;
    }
}

void UAV_controller::set_px4_mode_func(string mode)
{
    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = mode;
    px4_set_mode_client.call(mode_cmd);
}

void UAV_controller::enable_emergency_func()
{
    mavros_msgs::CommandLong emergency_srv;
    emergency_srv.request.broadcast = false;
    emergency_srv.request.command = 400;
    emergency_srv.request.confirmation = 0;
    emergency_srv.request.param1 = 0.0;
    emergency_srv.request.param2 = 21196;
    emergency_srv.request.param3 = 0.0;
    emergency_srv.request.param4 = 0.0;
    emergency_srv.request.param5 = 0.0;
    emergency_srv.request.param6 = 0.0;
    emergency_srv.request.param7 = 0.0;
    px4_emergency_client.call(emergency_srv);
    cout << RED << node_name << " send kill cmd: force disarmed!" << TAIL << endl;
    this->text_info.MessageType = prometheus_msgs::TextInfo::ERROR;
    this->text_info.Message = "send kill cmd: force disarmed!";
}

void UAV_controller::reboot_PX4()
{
    // https://mavlink.io/en/messages/common.html, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(#246)
    mavros_msgs::CommandLong reboot_srv;
    reboot_srv.request.broadcast = false;
    reboot_srv.request.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    reboot_srv.request.param1 = 1;    // Reboot autopilot
    reboot_srv.request.param2 = 0;    // Do nothing for onboard computer
    reboot_srv.request.confirmation = true;
    px4_reboot_client.call(reboot_srv);
    cout << GREEN << node_name << " Reboot PX4!" << TAIL << endl;
    this->text_info.MessageType = prometheus_msgs::TextInfo::WARN;
    this->text_info.Message = "Reboot PX4!";
}

//向地面发送反馈信息,如果重复,将不会发送
void UAV_controller::sendStationTextInfo(const ros::TimerEvent &e)
{
    if(this->text_info.Message == this->last_text_info.Message)
    {
        return;
    }
    else
    {
        this->text_info.header.stamp = ros::Time::now();
        this->ground_station_info_pub.publish(this->text_info);
        this->last_text_info = this->text_info;
        return;
    }
    
}
