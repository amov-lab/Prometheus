/***************************************************************************************************************************
* px4_pos_att_controller.cpp
*
* Author: Qyp
*
* Update Time: 2019.3.16
*
* Introduction:  PX4 Position & Attitude Controller using own control mehthod (but it is pid now)
*         1. Subscribe command.msg from upper nodes
*         2. Calculate the accel_sp using pos_controller_PID.h
*         3. Send command using command_to_mavros.h
***************************************************************************************************************************/

#include <ros/ros.h>

#include <command_to_mavros.h>
#include <pos_controller_PID.h>
#include <att_controller_PID.h>
#include <prometheus_msgs/ControlCommand.h>


#include <Eigen/Eigen>

using namespace std;
 
using namespace namespace_PID;

//自定义的Command变量
//相应的命令分别为 待机 起飞 悬停 降落 移动(惯性系ENU) 上锁 移动(机体系)
//但目前 起飞和待机 并没有正式使用
enum Command
{
    Move_ENU,
    Move_Body,
    Hold,
    Land,
    Disarm,
    PPN_land,
    Idle
};
//Command Now [from upper node]
prometheus_msgs::ControlCommand Command_Now;                      //无人机当前执行命令

//Command Last [from upper node]
prometheus_msgs::ControlCommand Command_Last;                     //无人机上一条执行命令

int flag_using_pid;

float get_time_in_sec(ros::Time begin);
void prinft_command_state();
void rotation_yaw(float yaw_angle, float input[2], float output[2]);

void Command_cb(const prometheus_msgs::ControlCommand::ConstPtr& msg)
{
    Command_Now = *msg;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_pos_att_controller");
    ros::NodeHandle nh("~");

    //flag of using our own pid control law or not: 0 for not use, 1 for use
    nh.param<int>("flag_using_pid", flag_using_pid, 0);

    ros::Subscriber Command_sub = nh.subscribe<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10, Command_cb);


    ros::Rate rate(250.0);

    Eigen::Vector3d pos_sp(0,0,0);
    Eigen::Vector3d vel_sp(0,0,0);
    Eigen::Vector3d accel_sp(0,0,0);

    Eigen::Vector4d actuator_sp(0,0,0,0);

    command_to_mavros command_fsc;

    pos_controller_PID pos_controller_fsc;

    att_controller_PID att_controller_fsc;

    command_fsc.printf_param();

    pos_controller_fsc.printf_pid_param();

    att_controller_fsc.printf_pid_param();

    command_fsc.show_geo_fence();

    int check_flag;
    // 这一步是为了程序运行前检查一下参数是否正确
    // 输入1,继续，其他，退出程序
    cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
    cin >> check_flag;

    if(check_flag != 1)
    {
        return -1;
    }

    // 等待和飞控的连接
    while(ros::ok() && command_fsc.current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Not Connected");
    }

    // 连接成功
    ROS_INFO("Connected!!");

    // 先读取一些飞控的数据
    int i =0;
    for(i=0;i<50;i++)
    {
        ros::spinOnce();
        rate.sleep();

    }


    command_fsc.set_takeoff_position();


    //初始化命令-
    // 默认设置：move模式 子模式：位置控制 起飞到当前位置点上方

    Command_Now.Command_ID = 0;
    Command_Now.Mode = Move_ENU;
    Command_Now.Move_mode  = 0;
    Command_Now.Reference_State.position_ref[0] = command_fsc.Takeoff_position[0];          //ENU Frame
    Command_Now.Reference_State.position_ref[1] = command_fsc.Takeoff_position[1];          //ENU Frame
    Command_Now.Reference_State.position_ref[2] = command_fsc.Takeoff_position[2] + command_fsc.Takeoff_height;         //ENU Frame
    Command_Now.Reference_State.velocity_ref[0] = 0;          //ENU Frame
    Command_Now.Reference_State.velocity_ref[1] = 0;          //ENU Frame
    Command_Now.Reference_State.velocity_ref[2] = 0;          //ENU Frame
    Command_Now.Reference_State.yaw_ref = 0;
    Command_Now.yaw_rate_sp = 0;


    // 记录启控时间
    ros::Time begin_time = ros::Time::now();

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        //执行回调函数
        ros::spinOnce();

        // 当前时间
        float cur_time = get_time_in_sec(begin_time);

        //Printf the drone state
        command_fsc.prinft_drone_state2(cur_time);

        //Printf the command state
        prinft_command_state();

        command_fsc.failsafe();

        //Printf the pid controller result
        //pos_controller_fsc.printf_result();

        att_controller_fsc.printf_result();

        //无人机一旦接受到Land指令，则会屏蔽其他指令
        if(Command_Last.Mode == Land)
        {
            Command_Now.Mode = Land;
        }

        switch (Command_Now.Mode)
        {

        case Move_ENU:
            pos_sp = Eigen::Vector3d(Command_Now.Reference_State.position_ref[0],Command_Now.Reference_State.position_ref[1],Command_Now.Reference_State.position_ref[2]);
            vel_sp = Eigen::Vector3d(Command_Now.Reference_State.velocity_ref[0],Command_Now.Reference_State.velocity_ref[1],Command_Now.Reference_State.velocity_ref[2]);

            accel_sp = pos_controller_fsc.pos_controller(command_fsc.pos_drone_fcu, command_fsc.vel_drone_fcu, pos_sp, vel_sp, Command_Now.Move_mode , cur_time);

            actuator_sp = att_controller_fsc.att_controller(command_fsc.Euler_fcu, command_fsc.rates_fcu, accel_sp, Command_Now.Reference_State.yaw_ref, cur_time);


            //for test
            //command_fsc.send_accel_setpoint(accel_sp, Command_Now.Reference_State.yaw_ref );


            command_fsc.send_actuator_setpoint(actuator_sp);

            break;

        case Move_Body:
            //只有在comid增加时才会进入解算
            if( Command_Now.Command_ID  >  Command_Last.comid )
            {
                //xy velocity mode
                if( Command_Now.Move_mode  & 0b10 )
                {
                    float d_vel_body[2] = {Command_Now.Reference_State.velocity_ref[0], Command_Now.Reference_State.velocity_ref[1]};         //the desired xy velocity in Body Frame
                    float d_vel_enu[2];                                                           //the desired xy velocity in NED Frame

                    rotation_yaw(command_fsc.Euler_fcu[2], d_vel_body, d_vel_enu);
                    vel_sp[0] = d_vel_enu[0];
                    vel_sp[1] = d_vel_enu[1];
                }
                //xy position mode
                else
                {
                    float d_pos_body[2] = {Command_Now.Reference_State.position_ref[0], Command_Now.Reference_State.position_ref[1]};         //the desired xy position in Body Frame
                    float d_pos_enu[2];                                                           //the desired xy position in enu Frame (The origin point is the drone)
                    rotation_yaw(command_fsc.Euler_fcu[2], d_pos_body, d_pos_enu);

                    pos_sp[0] = command_fsc.pos_drone_fcu[0] + d_pos_enu[0];
                    pos_sp[1] = command_fsc.pos_drone_fcu[1] + d_pos_enu[1];
                }

                //z velocity mode
                if( Command_Now.Move_mode  & 0b01 )
                {
                    vel_sp[2] = Command_Now.Reference_State.velocity_ref[2];
                }
                //z posiiton mode
                {
                    pos_sp[2] = command_fsc.pos_drone_fcu[2] + Command_Now.Reference_State.position_ref[2];
                }
            }

            accel_sp = pos_controller_fsc.pos_controller(command_fsc.pos_drone_fcu, command_fsc.vel_drone_fcu, pos_sp, vel_sp, Command_Now.Move_mode , cur_time);

            actuator_sp = att_controller_fsc.att_controller(command_fsc.Euler_fcu, command_fsc.rates_fcu, accel_sp, Command_Now.Reference_State.yaw_ref, cur_time);

            command_fsc.send_actuator_setpoint(actuator_sp);

            break;

        case Hold:
            if (Command_Last.Mode != Hold)
            {
                command_fsc.Hold_position = Eigen::Vector3d(Command_Now.Reference_State.position_ref[0],Command_Now.Reference_State.position_ref[1],Command_Now.Reference_State.position_ref[2]);
            }

            accel_sp = pos_controller_fsc.pos_controller(command_fsc.pos_drone_fcu, command_fsc.vel_drone_fcu, pos_sp, vel_sp, Command_Now.Move_mode , cur_time);

            actuator_sp = att_controller_fsc.att_controller(command_fsc.Euler_fcu, command_fsc.rates_fcu, accel_sp, Command_Now.Reference_State.yaw_ref, cur_time);

            command_fsc.send_actuator_setpoint(actuator_sp);

            break;


        case Land:
            if (Command_Last.Mode != Land)
            {
                pos_sp = Eigen::Vector3d(command_fsc.pos_drone_fcu[0],command_fsc.pos_drone_fcu[1],command_fsc.Takeoff_position[2]);
            }

            //如果距离起飞高度小于20厘米，则直接上锁并切换为手动模式；
            if(abs(command_fsc.pos_drone_fcu[2] - command_fsc.Takeoff_position[2]) < ( 0.2))
            {
                if(command_fsc.current_state.mode == "OFFBOARD")
                {
                    command_fsc.mode_cmd.request.custom_mode = "MANUAL";
                    command_fsc.set_mode_client.call(command_fsc.mode_cmd);
                }

                if(command_fsc.current_state.armed)
                {
                    command_fsc.arm_cmd.request.value = false;
                    command_fsc.arming_client.call(command_fsc.arm_cmd);

                }

                if (command_fsc.arm_cmd.response.success)
                {
                    cout<<"Disarm successfully!"<<endl;
                }
            }else
            {
                accel_sp = pos_controller_fsc.pos_controller(command_fsc.pos_drone_fcu, command_fsc.vel_drone_fcu, pos_sp, vel_sp, Command_Now.Move_mode , cur_time);

                actuator_sp = att_controller_fsc.att_controller(command_fsc.Euler_fcu, command_fsc.rates_fcu, accel_sp, Command_Now.Reference_State.yaw_ref, cur_time);

                command_fsc.send_actuator_setpoint(actuator_sp);
            }

            break;

        case Disarm:

            if(command_fsc.current_state.mode == "OFFBOARD")
            {
                command_fsc.mode_cmd.request.custom_mode = "MANUAL";
                command_fsc.set_mode_client.call(command_fsc.mode_cmd);
            }

            if(command_fsc.current_state.armed)
            {
                command_fsc.arm_cmd.request.value = false;
                command_fsc.arming_client.call(command_fsc.arm_cmd);

            }

            if (command_fsc.arm_cmd.response.success)
            {
                cout<<"Disarm successfully!"<<endl;
            }

            break;

        // 【】
        case PPN_land:


            break;

        // 【】
        case Idle:


            break;
        }

        Command_Last = Command_Now;

        rate.sleep();
    }

    return 0;

}

// 【获取当前时间函数】 单位：秒
float get_time_in_sec(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}
// 【打印控制指令函数】
void prinft_command_state()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Command State<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    switch(Command_Now.Mode)
    {
    case Move_ENU:
        cout << "Command: [ Move_ENU ] " <<endl;
        break;
    case Move_Body:
        cout << "Command: [ Move_Body ] " <<endl;
        break;
    case Hold:
        cout << "Command: [ Hold ] " <<endl;
        break;
    case Land:
        cout << "Command: [ Land ] " <<endl;
        break;
    case Disarm:
        cout << "Command: [ Disarm ] " <<endl;
        break;
    case PPN_land:
        cout << "Command: [ PPN_land ] " <<endl;
        break;
    case Idle:
        cout << "Command: [ Idle ] " <<endl;
        break;
    }

    int sub_mode;
    sub_mode = Command_Now.Move_mode ;

    if((sub_mode & 0b10) == 0) //xy channel
    {
        cout << "Submode: xy position control "<<endl;
        cout << "X_setpoint   : " << Command_Now.Reference_State.position_ref[0] << " [ m ]"  << "  Y_setpoint : "<< Command_Now.Reference_State.position_ref[1] << " [ m ]"<<endl;
    }
    else{
        cout << "Submode: xy velocity control "<<endl;
        cout << "X_setpoint   : " << Command_Now.Reference_State.velocity_ref[0] << " [m/s]" << "  Y_setpoint : "<< Command_Now.Reference_State.velocity_ref[1] << " [m/s]" <<endl;
    }

    if((sub_mode & 0b01) == 0) //z channel
    {
        cout << "Submode:  z position control "<<endl;
        cout << "Z_setpoint   : "<< Command_Now.Reference_State.position_ref[2] << " [ m ]" << endl;
    }
    else
    {
        cout << "Submode:  z velocity control "<<endl;
        cout << "Z_setpoint   : "<< Command_Now.Reference_State.velocity_ref[2] << " [m/s]" <<endl;
    }

    cout << "Yaw_setpoint : "  << Command_Now.Reference_State.yaw_ref << " [deg] " <<endl;
}
// 【坐标系旋转函数】- 机体系到enu系
// input是机体系,output是惯性系，yaw_angle是当前偏航角
void rotation_yaw(float yaw_angle, float input[2], float output[2])
{
    output[0] = input[0] * cos(yaw_angle) - input[1] * sin(yaw_angle);
    output[1] = input[0] * sin(yaw_angle) + input[1] * cos(yaw_angle);
}
