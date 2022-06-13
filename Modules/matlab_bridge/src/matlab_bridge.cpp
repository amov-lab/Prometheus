#include "matlab_bridge.h"

Matlab_Bridge::Matlab_Bridge(ros::NodeHandle &nh)
{
    // 【参数】编号
    nh.param<int>("uav_id", uav_id, 1);
    // 【参数】无人机质量（需要根据无人机实际情况进行设置）
    nh.param<float>("controller/quad_mass", ctrl_param.quad_mass, 1.0f);
    // 【参数】悬停油门（需要根据无人机实际情况进行设置）
    nh.param<float>("controller/hov_percent", ctrl_param.hov_percent, 0.47f);
    // 【参数】最大倾斜角度（如果需要进行大角度机动，此处建议调大）
    nh.param<float>("controller/tilt_angle_max", ctrl_param.tilt_angle_max, 15.0f);
    // 【参数】重力常数
    ctrl_param.g << 0.0, 0.0, 9.8;

    agent_name = "/uav" + std::to_string(uav_id);
    //【订阅】来自Matlab的配置信息
    matlab_setting_cmd_sub =
        nh.subscribe<geometry_msgs::Point>(agent_name + "/prometheus/matlab_setting_cmd",
                                           10,
                                           &Matlab_Bridge::matlab_setting_cmd_cb, this);

    //【订阅】来自Matlab的控制信息
    matlab_cmd_sub =
        nh.subscribe<geometry_msgs::Pose>(agent_name + "/prometheus/matlab_cmd",
                                          10,
                                          &Matlab_Bridge::matlab_cmd_cb, this);

    //【订阅】无人机状态信息
    uav_state_sub =
        nh.subscribe<prometheus_msgs::UAVState>(agent_name + "/prometheus/state",
                                                10,
                                                &Matlab_Bridge::uav_state_cb, this);

    //【订阅】无人机控制信息
    uav_contorl_state_sub =
        nh.subscribe<prometheus_msgs::UAVControlState>(agent_name + "/prometheus/control_state",
                                                       10,
                                                       &Matlab_Bridge::uav_control_state_cb, this);

    //【发布】发布matlab配置结果 -> matlab节点
    matlab_drone_status_pub =
        nh.advertise<geometry_msgs::Point>(agent_name + "/prometheus/matlab_drone_status", 10);

    //【发布】发布控制指令 -> uav_controller.cpp
    uav_command_pub =
        nh.advertise<prometheus_msgs::UAVCommand>(agent_name + "/prometheus/command", 10);

    //【发布】mavros接口调用指令(-> uav_control.cpp)
    uav_setup_pub = 
        nh.advertise<prometheus_msgs::UAVSetup>(agent_name + "/prometheus/setup", 1);

    //【发布】发布文字消息 -> matlab节点
    text_pub =
        nh.advertise<std_msgs::String>(agent_name + "/prometheus/matlab_text", 1);

    //【定时器】matlab_drone_status_pub 发布定时器
    timer_matlab_drone_status_pub = nh.createTimer(ros::Duration(0.05), &Matlab_Bridge::matlab_drone_status_pub_cb, this);

    //【定时器】打印定时器
    timer_printf = nh.createTimer(ros::Duration(5.0), &Matlab_Bridge::printf_msgs, this);

    cmd_timeout = false;

    matlab_drone_status.x = 0;
    matlab_drone_status.y = 0;
    matlab_drone_status.z = 0;

    cout << GREEN << "matlab bridge init!" << TAIL << endl;

    text.data = "matlab bridge init.";
    text_pub.publish(text);
}

void Matlab_Bridge::mainloop()
{
    // 没收到matlab指令时，没有任何操作
    // 收到matlab指令后，如果control_cmd超时，则会有对应的处理机制
    
    if (matlab_setting_cmd.x == MATLAB_CMD_X::TAKEOFF)
    {
        // 本接口暂不开放
        // uav_command.header.stamp = ros::Time::now();
        // uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
        // uav_command_pub.publish(uav_command);
    }
    else if (matlab_setting_cmd.x == MATLAB_CMD_X::LAND)
    {
        uav_command.header.stamp = ros::Time::now();
        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Land;
        uav_command_pub.publish(uav_command);
    }
    else if (matlab_setting_cmd.x == MATLAB_CMD_X::HOLD)
    {
        uav_command.header.stamp = ros::Time::now();
        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
        uav_command_pub.publish(uav_command);
    }
    else if (matlab_setting_cmd.x == MATLAB_CMD_X::MATLAB_CMD)
    {
        // 检查matlab_control_cmd是否有存在延迟
        if(!matlab_control_cmd_safety_check())
        {
            return;
        }

        if (matlab_setting_cmd.y == MATLAB_CMD_Y::POS_CTRL_MODE)
        {
            matlab_control_mode = MATLAB_CMD_Y::POS_CTRL_MODE;
            uav_command.header.stamp = ros::Time::now();
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
            uav_command.position_ref[0] = matlab_cmd.position.x;
            uav_command.position_ref[1] = matlab_cmd.position.y;
            uav_command.position_ref[2] = matlab_cmd.position.z;
            uav_command.yaw_ref = matlab_cmd.orientation.w;
            uav_command_pub.publish(uav_command);
        }
        else if (matlab_setting_cmd.y == MATLAB_CMD_Y::VEL_CTRL_MODE)
        {
            matlab_control_mode = MATLAB_CMD_Y::VEL_CTRL_MODE;
            uav_command.header.stamp = ros::Time::now();
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL;
            uav_command.velocity_ref[0] = matlab_cmd.position.x;
            uav_command.velocity_ref[1] = matlab_cmd.position.y;
            uav_command.velocity_ref[2] = matlab_cmd.position.z;
            uav_command.yaw_ref = matlab_cmd.orientation.w;
            uav_command_pub.publish(uav_command);
        }
        else if (matlab_setting_cmd.y == MATLAB_CMD_Y::ACC_CTRL_MODE)
        {
            matlab_control_mode = MATLAB_CMD_Y::ACC_CTRL_MODE;
            // 根据Matlab的加速度控制指令解算姿态控制指令
            Eigen::Vector3d acc_cmd;
            acc_cmd << matlab_cmd.position.x, matlab_cmd.position.y, matlab_cmd.position.z;
            double yaw_cmd;
            yaw_cmd = matlab_cmd.orientation.w;
            Eigen::Vector4d att_cmd = acc_cmd_to_att_cmd(acc_cmd, yaw_cmd);

            uav_command.header.stamp = ros::Time::now();
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_ATT;
            uav_command.att_ref[0] = att_cmd[0];
            uav_command.att_ref[1] = att_cmd[1];
            uav_command.att_ref[2] = att_cmd[2];
            uav_command.att_ref[3] = att_cmd[3];
            uav_command_pub.publish(uav_command);
        }
        else if (matlab_setting_cmd.y == MATLAB_CMD_Y::ATT_CTRL_MODE)
        {
            matlab_control_mode = MATLAB_CMD_Y::ATT_CTRL_MODE;
            uav_command.header.stamp = ros::Time::now();
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_ATT;
            uav_command.att_ref[0] = matlab_cmd.orientation.x;
            uav_command.att_ref[1] = matlab_cmd.orientation.y;
            uav_command.att_ref[2] = matlab_cmd.orientation.z;
            uav_command.att_ref[3] = matlab_cmd.orientation.w;
            uav_command_pub.publish(uav_command);
        }
        else
        {
            cout << RED << "wrong matlab_setting_cmd.y!" << TAIL << endl;
        }
    }
}

bool Matlab_Bridge::matlab_control_cmd_safety_check()
{
    // 接收指令超时（网络状态较差会导致）
    // 原地悬停等待（MATLAB_CMD_TIMEOUT）-> 返回起始点（RETURN_INIT_POS_TIMEOUT）-> 降落（LAND_TIMEOUT）
    double delta_time_matlab_cmd = (ros::Time::now() - last_matlab_cmd_time).toSec();
    if (delta_time_matlab_cmd > MATLAB_CMD_TIMEOUT)
    {
        // 第一次进入 - 悬停
        if (!cmd_timeout)
        {
            // cout << RED << "MATLAB_CMD_TIMEOUT!" << TAIL << endl;
            text.data = "MATLAB_CMD_TIMEOUT!";
            uav_command.header.stamp = ros::Time::now();
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
            uav_command_pub.publish(uav_command);
        }
        else
        {
            // 接收指令超时，降落
            if (delta_time_matlab_cmd > LAND_TIMEOUT)
            {
                // cout << RED << "MATLAB TIMEOUT, LAND now!" << TAIL << endl;
                text.data = "MATLAB TIMEOUT, LAND now!";
                uav_command.header.stamp = ros::Time::now();
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Land;
                uav_command_pub.publish(uav_command);
            }
            // 接收指令超时，返回起始点
            else if (delta_time_matlab_cmd > RETURN_INIT_POS_TIMEOUT)
            {
                // cout << RED << "MATLAB TIMEOUT, RETURN Init Pos!" << TAIL << endl;
                text.data = "MATLAB TIMEOUT, RETURN Init Pos!";
                uav_command.header.stamp = ros::Time::now();
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
                uav_command_pub.publish(uav_command);
            }
        }
        // 打印延迟
        static int printf_num = 0;
        if(printf_num > 50)
        {
            printf_num = 0;
            cout << RED << text.data << TAIL << endl;
        }else
        {
            printf_num++;
        }
        cmd_timeout = true;
        return false;
    }

    return true;
}

void Matlab_Bridge::matlab_drone_status_pub_cb(const ros::TimerEvent &e)
{
    // 0b 000000
    // bit1: connected or not
    // bit2: odom_valid or not
    // bit3: armed or not
    // bit4: OFFBOARD or not
    // bit5 & 6 & 7: OTHER(000) MANUAL(001) COMMAND_MODE(010) or LAND_CONTROL(011)
    // bit8: failsafe
    // bit9: 预留接口
    // bit10: 预留接口
    // bit11: 预留接口
    // bit12: always 1
    // 4、^ 按位异或运算。
    
    // 根据uav_state对matlab_drone_status进行赋值
    matlab_drone_status.x = 0b000000000000;              // 无人机状态量
    matlab_drone_status.y = uav_state.battery_state;  // 电池电压
    matlab_drone_status.z = 1;                        // 电池百分比
    if(uav_control_state.failsafe)
    {
        matlab_drone_status.x = (int)matlab_drone_status.x ^ 0b000010000000;
    }

    if (uav_control_state.control_state == prometheus_msgs::UAVControlState::RC_POS_CONTROL)
    {
        matlab_drone_status.x = (int)matlab_drone_status.x ^ 0b000000010000;
    }else if (uav_control_state.control_state == prometheus_msgs::UAVControlState::COMMAND_CONTROL)
    {
        matlab_drone_status.x = (int)matlab_drone_status.x ^ 0b000000100000;
    }else if (uav_control_state.control_state == prometheus_msgs::UAVControlState::LAND_CONTROL)
    {
        matlab_drone_status.x = (int)matlab_drone_status.x ^ 0b000000110000;
    }else
    {
        matlab_drone_status.x = 0b000000000000;
    }

    if (uav_state.connected)
    {
        matlab_drone_status.x = (int)matlab_drone_status.x ^ 0b000000000001;
    }

    if (uav_state.odom_valid)
    {
        matlab_drone_status.x = (int)matlab_drone_status.x ^ 0b000000000010;
    }

    if (uav_state.armed)
    {
        matlab_drone_status.x = (int)matlab_drone_status.x ^ 0b000000000100;
    }

    if (uav_state.mode == "OFFBOARD")
    {
        matlab_drone_status.x = (int)matlab_drone_status.x ^ 0b000000001000;
    }

    matlab_drone_status.x = (int)matlab_drone_status.x ^ 0b100000000000;
    
    matlab_drone_status_pub.publish(matlab_drone_status);
}

void Matlab_Bridge::matlab_setting_cmd_cb(const geometry_msgs::Point::ConstPtr &msg)
{
    matlab_setting_cmd = *msg;
    last_matlab_setting_cmd_time = ros::Time::now();
}

void Matlab_Bridge::matlab_cmd_cb(const geometry_msgs::Pose::ConstPtr &msg)
{
    matlab_cmd = *msg;
    last_matlab_cmd_time = ros::Time::now();

    if (cmd_timeout)
    {
        cmd_timeout = false;
        cout << GREEN << "MATLAB_CMD regain!" << TAIL << endl;
    }
}

void Matlab_Bridge::uav_state_cb(const prometheus_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
    get_uav_state_stamp = ros::Time::now();
}

void Matlab_Bridge::uav_control_state_cb(const prometheus_msgs::UAVControlState::ConstPtr &msg)
{
    uav_control_state = *msg;
}

Eigen::Vector4d Matlab_Bridge::acc_cmd_to_att_cmd(Eigen::Vector3d &acc_cmd, double yaw_cmd)
{
    Eigen::Vector4d att_cmd;
    // 期望力 = 质量*控制量 + 重力抵消 + 期望加速度*质量*Ka
    // F_des是基于模型的位置控制器计算得到的三轴期望推力（惯性系），量纲为牛
    // att_cmd是用于PX4的姿态控制输入，att_cmd 前三位是roll pitch yaw， 第四位为油门值[0-1]
    Eigen::Vector3d F_des;
    F_des = acc_cmd * ctrl_param.quad_mass + ctrl_param.quad_mass * ctrl_param.g;

    // 如果向上推力小于重力的一半
    // 或者向上推力大于重力的两倍
    if (F_des(2) < 0.5 * ctrl_param.quad_mass * ctrl_param.g(2))
    {
        F_des = F_des / F_des(2) * (0.5 * ctrl_param.quad_mass * ctrl_param.g(2));
    }
    else if (F_des(2) > 2 * ctrl_param.quad_mass * ctrl_param.g(2))
    {
        F_des = F_des / F_des(2) * (2 * ctrl_param.quad_mass * ctrl_param.g(2));
    }

    // 角度限制幅度
    if (std::fabs(F_des(0) / F_des(2)) > std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max)))
    {
        ROS_INFO("pitch too tilt");
        F_des(0) = F_des(0) / std::fabs(F_des(0)) * F_des(2) * std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max));
    }

    // 角度限制幅度
    if (std::fabs(F_des(1) / F_des(2)) > std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max)))
    {
        ROS_INFO("roll too tilt");
        F_des(1) = F_des(1) / std::fabs(F_des(1)) * F_des(2) * std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max));
    }

    // F_des是位于ENU坐标系的,F_c是FLU
    double current_yaw = (double)uav_state.attitude[2];
    Eigen::Matrix3d wRc = geometry_utils::rotz(current_yaw);
    Eigen::Vector3d F_c = wRc.transpose() * F_des;
    double fx = F_c(0);
    double fy = F_c(1);
    double fz = F_c(2);

    // 期望roll, pitch
    att_cmd(0) = std::atan2(-fy, fz);
    att_cmd(1) = std::atan2(fx, fz);
    att_cmd(2) = yaw_cmd;

    Eigen::Quaterniond uav_quat;
    uav_quat = Eigen::Quaterniond(uav_state.attitude_q.w, uav_state.attitude_q.x, uav_state.attitude_q.y, uav_state.attitude_q.z);

    // 无人机姿态的矩阵形式
    Eigen::Matrix3d wRb_odom = uav_quat.toRotationMatrix();
    // 第三列
    Eigen::Vector3d z_b_curr = wRb_odom.col(2);
    // 机体系下的电机推力 相当于Rb * F_enu 惯性系到机体系
    double u1 = F_des.dot(z_b_curr);
    // 悬停油门与电机参数有关系,也取决于质量
    double full_thrust = ctrl_param.quad_mass * ctrl_param.g(2) / ctrl_param.hov_percent;

    // 油门 = 期望推力/最大推力
    // 这里相当于认为油门是线性的,满足某种比例关系,即认为某个重量 = 悬停油门
    att_cmd(3) = u1 / full_thrust;

    if (att_cmd(3) < 0.1)
    {
        att_cmd(3) = 0.1;
        // ROS_INFO("throttle too low");
    }

    if (att_cmd(3) > 1.0)
    {
        att_cmd(3) = 1.0;
        // ROS_INFO("throttle too high");
    }

    return att_cmd;
}

void Matlab_Bridge::printf_msgs(const ros::TimerEvent &e)
{
    //固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为n位
    cout << setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);
    cout << GREEN << "--------------> Matlab Bridge <------------- " << TAIL << endl;

    cout << GREEN << "[ ID: " << uav_id << "]  " << TAIL;

    cout << GREEN << "uav_state: " << matlab_drone_status.x << "  " << TAIL << endl;

    if(cmd_timeout)
    {
        cout << RED << "[ CMD TIMEOUT ]  " << TAIL << endl;
    }
    else
    {
        cout << GREEN << "[ CMD GOOD ]  " << TAIL << endl;
    }

    if (matlab_setting_cmd.x == MATLAB_CMD_X::CHECK)
    {
        cout << GREEN << "Setting Cmd: [ CHECK ] " << TAIL << endl;
    }
    else if (matlab_setting_cmd.x == MATLAB_CMD_X::TAKEOFF)
    {
        cout << GREEN << "Setting Cmd: [ TAKEOFF ] " << TAIL << endl;
    }
    else if (matlab_setting_cmd.x == MATLAB_CMD_X::LAND)
    {
        cout << GREEN << "Setting Cmd: [ LAND ] " << TAIL << endl;
    }
    else if (matlab_setting_cmd.x == MATLAB_CMD_X::HOLD)
    {
        cout << GREEN << "Setting Cmd: [ HOLD ] " << TAIL << endl;
    }
    else if (matlab_setting_cmd.x == MATLAB_CMD_X::MATLAB_CMD)
    {
        cout << GREEN << "Setting Cmd: [ MATLAB_CMD ] " << TAIL << endl;
    }

    if (matlab_setting_cmd.x == MATLAB_CMD_X::MATLAB_CMD)
    {
        if (matlab_control_mode == MATLAB_CMD_Y::POS_CTRL_MODE)
        {

            cout << GREEN << "Matlab CMD: [ POS_CTRL_MODE ] " << TAIL << endl;
            cout << GREEN << "Pos_ref [X Y Z] : " << matlab_cmd.position.x << " [ m ] " << matlab_cmd.position.y << " [ m ] " << matlab_cmd.position.z << " [ m ] " << TAIL << endl;
            cout << GREEN << "Yaw_ref         : " << matlab_cmd.orientation.w * 180 / M_PI << " [deg] " << TAIL << endl;
        }
        else if (matlab_control_mode == MATLAB_CMD_Y::VEL_CTRL_MODE)
        {
            cout << GREEN << "Matlab CMD: [ VEL_CTRL_MODE ] " << TAIL << endl;
            cout << GREEN << "Vel_ref [X Y Z] : " << matlab_cmd.position.x << " [m/s] " << matlab_cmd.position.y << " [m/s] " << matlab_cmd.position.z << " [m/s] " << TAIL << endl;
            cout << GREEN << "Yaw_ref         : " << matlab_cmd.orientation.w * 180 / M_PI << " [deg] " << TAIL << endl;
        }
        else if (matlab_control_mode == MATLAB_CMD_Y::ACC_CTRL_MODE)
        {
            cout << GREEN << "Matlab CMD: [ ACC_CTRL_MODE ] " << TAIL << endl;
            cout << GREEN << "Acc_ref [X Y Z] : " << matlab_cmd.position.x << " [m/s^2] " << matlab_cmd.position.y << " [m/s^2] " << matlab_cmd.position.z << " [m/s^2] " << TAIL << endl;
            cout << GREEN << "Yaw_ref         : " << matlab_cmd.orientation.w * 180 / M_PI << " [deg] " << TAIL << endl;

            cout << GREEN << "Att_ref [X Y Z] : " << uav_command.att_ref[0] * 180 / M_PI << " [deg] " << uav_command.att_ref[1] * 180 / M_PI << " [deg] " << uav_command.att_ref[2] * 180 / M_PI << " [deg] " << TAIL << endl;
            cout << GREEN << "Thrust_ref[0-1] : " << uav_command.att_ref[3] << TAIL << endl;
        }
        else if (matlab_control_mode == MATLAB_CMD_Y::ATT_CTRL_MODE)
        {
            // cout << GREEN << "Matlab CMD: [ ATT_CTRL_MODE ] " << TAIL << endl;
            // cout << GREEN << "Att_ref [X Y Z] : " << uav_command.att_ref[0] * 180 / M_PI << " [deg] " << uav_command.att_ref[1] * 180 / M_PI << " [deg] " << uav_command.att_ref[2] * 180 / M_PI << " [deg] " << TAIL << endl;
            // cout << GREEN << "Thrust_ref[0-1] : " << uav_command.att_ref[3] << TAIL << endl;
        }
    }
}