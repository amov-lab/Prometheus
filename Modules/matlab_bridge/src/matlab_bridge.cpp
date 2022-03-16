#include "matlab_bridge.h"

void matlab_setting_cmd_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    matlab_setting_cmd = *msg;

    // 这里先写一个大概逻辑，具体逻辑还要结合实际情况考虑
    if(matlab_setting_cmd.x == MATLAB_CMD_X::CHECK)
    {
        cout << GREEN  << "Matlab_setting_cmd: check uav state" << TAIL<<endl;
        if(uav_state.state == prometheus_msgs::UAVState::ready)
        {
            matlab_setting_result.x = 1;
            matlab_setting_result_pub.publish(matlab_setting_result);
            ready_for_matlab_check = true;
        }
    }else if(matlab_setting_cmd.x == MATLAB_CMD_X::TAKEOFF)
    {
        cout << GREEN  << "Matlab_setting_cmd: takeoff" << TAIL<<endl;
        // todo
    }else if(matlab_setting_cmd.x == MATLAB_CMD_X::LAND)
    {
        // todo
    }else if(matlab_setting_cmd.x == MATLAB_CMD_X::HOLD)
    {
        // todo
    }else if(matlab_setting_cmd.x == MATLAB_CMD_X::MATLAB_CMD)
    {
        ready_for_matlab_cmd = true;
    }else
    {
        cout << RED  << "wrong matlab_setting_cmd.x!" << TAIL<<endl;
    }

    // 配置MATLAB_CMD下的控制模式
    if(matlab_setting_cmd.y == MATLAB_CMD_Y::POS_CTRL_MODE)
    {
        matlab_control_mode = MATLAB_CMD_Y::POS_CTRL_MODE;
        cout << GREEN  << "matlab_control_mode: POS_CTRL_MODE" << TAIL<<endl;
    }else if(matlab_setting_cmd.x == MATLAB_CMD_Y::VEL_CTRL_MODE)
    {
        matlab_control_mode = MATLAB_CMD_Y::VEL_CTRL_MODE;
        cout << GREEN  << "matlab_control_mode: VEL_CTRL_MODE" << TAIL<<endl;
    }else if(matlab_setting_cmd.x == MATLAB_CMD_Y::ATT_CTRL_MODE)
    {
        matlab_control_mode = MATLAB_CMD_Y::ATT_CTRL_MODE;
        cout << GREEN  << "matlab_control_mode: ATT_CTRL_MODE" << TAIL<<endl;
    }else
    {
        ready_for_matlab_cmd = false;
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
    }else
    {
        // todo
    }
}
void uav_state_cb(const prometheus_msgs::UAVState::ConstPtr& msg)
{
    uav_state = *msg;
}
void printf_msgs(const ros::TimerEvent &e);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "matlab_bridge");
    ros::NodeHandle nh("~");

    // 【参数】编号
    nh.param<int>("uav_id", uav_id, 1);
 
    //【订阅】来自Matlab的控制信息
    ros::Subscriber matlab_cmd_sub = nh.subscribe<geometry_msgs::Pose>("/uav"+std::to_string(uav_id)+ "/prometheus/matlab_cmd", 1, matlab_cmd_cb);

    //【订阅】来自Matlab的配置信息
    ros::Subscriber matlab_setting_cmd_sub = nh.subscribe<geometry_msgs::Point>("/uav"+std::to_string(uav_id)+ "/prometheus/matlab_setting_cmd", 1, matlab_setting_cmd_cb);

    //【发布】发布控制指令 -> uav_controller.cpp
    uav_command_pub = nh.advertise<prometheus_msgs::UAVCommand>("/uav"+std::to_string(uav_id)+ "/prometheus/command", 1);

    //【发布】发布matlab配置结果 -> matlab节点
    matlab_setting_result_pub = nh.advertise<geometry_msgs::Point>("/uav"+std::to_string(uav_id)+ "/prometheus/matlab_setting_result", 1);

    //【定时器】打印定时器
    ros::Timer timer_printf = nh.createTimer(ros::Duration(0.1), printf_msgs);

    cout << GREEN  << "matlab bridge init!" << TAIL<<endl;

    while(ros::ok())
    {
        ros::spinOnce();

        ros::Duration(0.01).sleep();
    }
    return 0;
}

void printf_msgs(const ros::TimerEvent &e)
{
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    if(matlab_setting_cmd.x == MATLAB_CMD_X::MATLAB_CMD)
    {
        if(matlab_control_mode == MATLAB_CMD_Y::POS_CTRL_MODE)
        {
            cout << GREEN  << "Command: [ Move in XYZ_POS ] " << TAIL<<endl;
            cout << GREEN  << "Pos_ref [X Y Z] : " << uav_command.position_ref[0] << " [ m ] "<< uav_command.position_ref[1]<<" [ m ] "<< uav_command.position_ref[2]<<" [ m ] "<< TAIL<<endl;
            cout << GREEN  << "Yaw_ref : "  << uav_command.yaw_ref* 180/M_PI << " [deg] " << TAIL<<endl;
        }
    }

    // todo 


    // // 打印 指令信息
    // switch(matlab_setting_cmd.x)
    // {
    //     case prometheus_msgs::UAVCommand::Init_Pos_Hover:
    //         cout << GREEN  << "Command: [ Init_Pos_Hover ] " << TAIL<<endl;
    //         break;

    //     case prometheus_msgs::UAVCommand::Current_Pos_Hover:
    //         cout << GREEN  << "Command: [ Current_Pos_Hover ] " << TAIL<<endl;
    //         break;

    //     case prometheus_msgs::UAVCommand::Land:
    //         cout << GREEN  << "Command: [ Land ] " << TAIL<<endl;
    //         break;

    //     case prometheus_msgs::UAVCommand::Move:

    //         if(uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_POS)
    //         {
    //             cout << GREEN  << "Command: [ Move in XYZ_POS ] " << TAIL<<endl;
    //             cout << GREEN  << "Pos_ref [X Y Z] : " << uav_command.position_ref[0] << " [ m ] "<< uav_command.position_ref[1]<<" [ m ] "<< uav_command.position_ref[2]<<" [ m ] "<< TAIL<<endl;
    //             cout << GREEN  << "Yaw_ref : "  << uav_command.yaw_ref* 180/M_PI << " [deg] " << TAIL<<endl;
    //         }else if(uav_command.Move_mode == prometheus_msgs::UAVCommand::XY_VEL_Z_POS)
    //         {
    //             cout << GREEN  << "Command: [ Move in XY_VEL_Z_POS ] " << TAIL<<endl;
    //             cout << GREEN  << "Pos_ref [    Z] : " << uav_command.position_ref[2]<<" [ m ] "<< TAIL<<endl;
    //             cout << GREEN  << "Vel_ref [X Y  ] : " << uav_command.velocity_ref[0] << " [m/s] "<< uav_command.velocity_ref[1]<<" [m/s] "<< TAIL<<endl;
    //             cout << GREEN  << "Yaw_ref : "  << uav_command.yaw_ref* 180/M_PI << " [deg] " << TAIL<<endl;
    //         }else if(uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_VEL)
    //         {
    //             cout << GREEN  << "Command: [ Move in XYZ_VEL ] " << TAIL<<endl;
    //             cout << GREEN  << "Vel_ref [X Y Z] : " << uav_command.velocity_ref[0] << " [m/s] "<< uav_command.velocity_ref[1]<<" [m/s] "<< uav_command.velocity_ref[2]<<" [m/s] "<< TAIL<<endl;
    //             cout << GREEN  << "Yaw_ref : "  << uav_command.yaw_ref* 180/M_PI << " [deg] " << TAIL<<endl;
    //         }else if(uav_command.Move_mode == prometheus_msgs::UAVCommand::TRAJECTORY)
    //         {
    //             cout << GREEN  << "Command: [ Move in TRAJECTORY ] " << TAIL<<endl;
    //             cout << GREEN  << "Pos_ref [X Y Z] : " << uav_command.position_ref[0] << " [ m ] "<< uav_command.position_ref[1]<<" [ m ] "<< uav_command.position_ref[2]<<" [ m ] "<< TAIL<<endl;
    //             cout << GREEN  << "Vel_ref [X Y Z] : " << uav_command.velocity_ref[0] << " [m/s] "<< uav_command.velocity_ref[1]<<" [m/s] "<< uav_command.velocity_ref[2]<<" [m/s] "<< TAIL<<endl;
    //             cout << GREEN  << "Yaw_ref : "  << uav_command.yaw_ref* 180/M_PI << " [deg] " << TAIL<<endl;
    //         }else if(uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_POS_BODY)
    //         {
    //             cout << GREEN  << "Command: [ Move in XYZ_POS_BODY ] " << TAIL<<endl;
    //             cout << GREEN  << "Pos_ref [X Y Z] : " << uav_command.position_ref[0] << " [ m ] "<< uav_command.position_ref[1]<<" [ m ] "<< uav_command.position_ref[2]<<" [ m ] "<< TAIL<<endl;
    //             cout << GREEN  << "Yaw_ref : "  << uav_command.yaw_ref* 180/M_PI << " [deg] " << TAIL<<endl;
    //         }else if(uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_VEL_BODY)
    //         {
    //             cout << GREEN  << "Command: [ Move in XYZ_VEL_BODY ] " << TAIL<<endl;
    //             cout << GREEN  << "Vel_ref [X Y Z] : " << uav_command.velocity_ref[0] << " [m/s] "<< uav_command.velocity_ref[1]<<" [m/s] "<< uav_command.velocity_ref[2]<<" [m/s] "<< TAIL<<endl;
    //             cout << GREEN  << "Yaw_ref : "  << uav_command.yaw_ref* 180/M_PI << " [deg] " << TAIL<<endl;
    //         }else if(uav_command.Move_mode == prometheus_msgs::UAVCommand::XY_VEL_Z_POS_BODY)
    //         {
    //             cout << GREEN  << "Command: [ Move in XY_VEL_Z_POS_BODY ] " << TAIL<<endl;
    //             cout << GREEN  << "Pos_ref [    Z] : " << uav_command.position_ref[2]<<" [ m ] "<< TAIL<<endl;
    //             cout << GREEN  << "Vel_ref [X Y  ] : " << uav_command.velocity_ref[0] << " [m/s] "<< uav_command.velocity_ref[1]<<" [m/s] "<< TAIL<<endl;
    //             cout << GREEN  << "Yaw_ref : "  << uav_command.yaw_ref* 180/M_PI << " [deg] " << TAIL<<endl;
    //         }else if(uav_command.Move_mode == prometheus_msgs::UAVCommand::XYZ_ATT)
    //         {
    //             cout << GREEN  << "Command: [ Move in XYZ_ATT ] " << TAIL<<endl;
    //             cout << GREEN  << "Att_ref [X Y Z] : " << uav_command.att_ref[0] * 180/M_PI<< " [deg] "<< uav_command.att_ref[1]* 180/M_PI<<" [deg] "<< uav_command.att_ref[2]* 180/M_PI<<" [deg] "<< TAIL<<endl;
    //             cout << GREEN  << "Thrust_ref[0-1] : " << uav_command.att_ref[3] << TAIL<<endl;
    //         }else
    //         {
    //             cout << GREEN  << "Command: [ Unknown Mode ]. " << TAIL<<endl;
    //         }
    //         break;
    // }

}