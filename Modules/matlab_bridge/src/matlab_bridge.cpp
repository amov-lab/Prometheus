#include "matlab_bridge.h"

void matlab_setting_cmd_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    matlab_setting_cmd = *msg;
    last_matlab_setting_cmd_time = ros::Time::now();
    // 收到配置信息，默认设置屏蔽控制消息
    ready_for_matlab_cmd = false;

    // 这里先写一个大概逻辑，具体逻辑还要结合实际情况考虑
    if(matlab_setting_cmd.x == MATLAB_CMD_X::CHECK)
    {
        cout << GREEN  << "Get matlab_setting_cmd: check uav state" << TAIL<<endl;
        bool uav_ready = check_for_uav_state();
        if(uav_ready)
        {
            matlab_setting_result.x = MATLAB_RESULT_X::SUCCESS;       
            ready_for_matlab_check = true;
            cout << GREEN  << "Send matlab_setting_result: SUCCESS" << TAIL<<endl;
        }else
        {
            matlab_setting_result.x = MATLAB_RESULT_X::REJECT;
            ready_for_matlab_check = false;
            cout << GREEN  << "Send matlab_setting_result: REJECT" << TAIL<<endl;
        }
        matlab_setting_result_pub.publish(matlab_setting_result);
    }else if(matlab_setting_cmd.x == MATLAB_CMD_X::TAKEOFF)
    {
        cout << GREEN  << "Get matlab_setting_cmd: TAKEOFF" << TAIL<<endl;
        uav_command.header.stamp = ros::Time::now();
        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
        uav_command_pub.publish(uav_command);
    }else if(matlab_setting_cmd.x == MATLAB_CMD_X::LAND)
    {
        cout << GREEN  << "Get matlab_setting_cmd: LAND" << TAIL<<endl;
        uav_command.header.stamp = ros::Time::now();
        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Land;
        uav_command_pub.publish(uav_command);
    }else if(matlab_setting_cmd.x == MATLAB_CMD_X::HOLD)
    {
        cout << GREEN  << "Get matlab_setting_cmd: HOLD" << TAIL<<endl;
        uav_command.header.stamp = ros::Time::now();
        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
        uav_command_pub.publish(uav_command);
    }else if(matlab_setting_cmd.x == MATLAB_CMD_X::MATLAB_CMD)
    {
        ready_for_matlab_cmd = true;
        cout << GREEN  << "Get matlab_setting_cmd: MATLAB_CMD" << TAIL<<endl;
        // 配置MATLAB_CMD下的控制模式
        if(matlab_setting_cmd.y == MATLAB_CMD_Y::POS_CTRL_MODE)
        {
            matlab_control_mode = MATLAB_CMD_Y::POS_CTRL_MODE;
            cout << GREEN  << "Get matlab_setting_cmd:  POS_CTRL_MODE" << TAIL<<endl;
        }else if(matlab_setting_cmd.x == MATLAB_CMD_Y::VEL_CTRL_MODE)
        {
            matlab_control_mode = MATLAB_CMD_Y::VEL_CTRL_MODE;
            cout << GREEN  << "Get matlab_setting_cmd:  VEL_CTRL_MODE" << TAIL<<endl;
        }else if(matlab_setting_cmd.x == MATLAB_CMD_Y::ACC_CTRL_MODE)
        {
            matlab_control_mode = MATLAB_CMD_Y::ACC_CTRL_MODE;
            cout << GREEN  << "Get matlab_setting_cmd:  ACC_CTRL_MODE" << TAIL<<endl;
        }else if(matlab_setting_cmd.x == MATLAB_CMD_Y::ATT_CTRL_MODE)
        {
            matlab_control_mode = MATLAB_CMD_Y::ATT_CTRL_MODE;
            cout << GREEN  << "Get matlab_setting_cmd:  ATT_CTRL_MODE" << TAIL<<endl;
        }else
        {
            ready_for_matlab_cmd = false;
            cout << RED  << "wrong matlab_setting_cmd.y!" << TAIL<<endl;
        }
    }else
    {
        cout << RED  << "wrong matlab_setting_cmd.x!" << TAIL<<endl;
    }
}

void matlab_cmd_cb(const geometry_msgs::Pose::ConstPtr& msg)
{
    // 
    if(!ready_for_matlab_check)
    {
        cout << RED  << "uav not ready, pls check uav state first!" << TAIL<<endl;
        return;
    }

    if(!ready_for_matlab_cmd)
    {
        cout << RED  << "not in MATLAB_CMD!" << TAIL<<endl;
        return;
    }

    matlab_cmd = *msg;

    last_matlab_cmd_time = ros::Time::now();

    if(cmd_timeout)
    {
        cmd_timeout = false;
        cout << GREEN  << "MATLAB_CMD regain!" << TAIL<<endl;
    }

    if(matlab_control_mode == MATLAB_CMD_Y::POS_CTRL_MODE)
    {
        uav_command.header.stamp = ros::Time::now();
        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
        uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
        uav_command.position_ref[0] = matlab_cmd.position.x;
        uav_command.position_ref[1] = matlab_cmd.position.y;
        uav_command.position_ref[2] = matlab_cmd.position.z;
        uav_command.yaw_ref = matlab_cmd.orientation.w;
        uav_command_pub.publish(uav_command);
    }else if(matlab_control_mode == MATLAB_CMD_Y::VEL_CTRL_MODE)
    {
        uav_command.header.stamp = ros::Time::now();
        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
        uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL;
        uav_command.velocity_ref[0] = matlab_cmd.position.x;
        uav_command.velocity_ref[1] = matlab_cmd.position.y;
        uav_command.velocity_ref[2] = matlab_cmd.position.z;
        uav_command.yaw_ref = matlab_cmd.orientation.w;
        uav_command_pub.publish(uav_command);
    }else if(matlab_control_mode == MATLAB_CMD_Y::ACC_CTRL_MODE)
    {
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
    }else if(matlab_control_mode == MATLAB_CMD_Y::ATT_CTRL_MODE)
    {
        uav_command.header.stamp = ros::Time::now();
        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
        uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_ATT;
        uav_command.att_ref[0] = matlab_cmd.orientation.x;
        uav_command.att_ref[1] = matlab_cmd.orientation.y;
        uav_command.att_ref[2] = matlab_cmd.orientation.z;
        uav_command.att_ref[3] = matlab_cmd.orientation.w;
        uav_command_pub.publish(uav_command);
    }

}
void uav_state_cb(const prometheus_msgs::UAVState::ConstPtr& msg)
{
    uav_state = *msg;
    get_uav_state_stamp = ros::Time::now();
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "matlab_bridge");
    ros::NodeHandle nh("~");

    // 【参数】编号
    nh.param<int>("uav_id", uav_id, 1);
     // 【参数】无人机质量
    nh.param<float>("controller/quad_mass" , ctrl_param.quad_mass, 1.0f);
    // 【参数】悬停油门
    nh.param<float>("controller/hov_percent" , ctrl_param.hov_percent, 0.5f);
    // 【参数】最大倾斜角度
    nh.param<float>("controller/tilt_angle_max" , ctrl_param.tilt_angle_max, 10.0f);
    // 【参数】重力
    ctrl_param.g << 0.0, 0.0, 9.8;

    //【订阅】来自Matlab的控制信息
    ros::Subscriber matlab_cmd_sub = nh.subscribe<geometry_msgs::Pose>("/uav"+std::to_string(uav_id)+ "/prometheus/matlab_cmd", 1, matlab_cmd_cb);

    //【订阅】来自Matlab的配置信息
    ros::Subscriber matlab_setting_cmd_sub = nh.subscribe<geometry_msgs::Point>("/uav"+std::to_string(uav_id)+ "/prometheus/matlab_setting_cmd", 1, matlab_setting_cmd_cb);

    //【订阅】状态信息
    ros::Subscriber uav_state_sub = nh.subscribe<prometheus_msgs::UAVState>("/uav"+std::to_string(uav_id)+"/prometheus/state", 1, uav_state_cb);
    
    //【发布】发布控制指令 -> uav_controller.cpp
    uav_command_pub = nh.advertise<prometheus_msgs::UAVCommand>("/uav"+std::to_string(uav_id)+ "/prometheus/command", 1);

    //【发布】发布matlab配置结果 -> matlab节点
    matlab_setting_result_pub = nh.advertise<geometry_msgs::Point>("/uav"+std::to_string(uav_id)+ "/prometheus/matlab_setting_result", 1);

    //【定时器】打印定时器
    ros::Timer timer_printf = nh.createTimer(ros::Duration(0.1), printf_msgs);

    //【定时器】安全检查定时器
    ros::Timer timer_matlab_safety_check = nh.createTimer(ros::Duration(0.1), matlab_safety_check);

    cout << GREEN  << "matlab bridge init!" << TAIL<<endl;

    while(ros::ok())
    {
        ros::spinOnce();

        ros::Duration(0.01).sleep();
    }
    return 0;
}
