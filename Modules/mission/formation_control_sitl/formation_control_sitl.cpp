/***************************************************************************************************************************
* formation_control.cpp
*
* Author: Qyp
*
* Update Time: 2019.6.16
***************************************************************************************************************************/
#include <ros/ros.h>
#include <prometheus_msgs/ControlCommand.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/Imu.h>
#include <bitset>
#include <Eigen/Eigen>
using namespace std;
 
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>变量声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

mavros_msgs::State current_state_uav1;
Eigen::Vector3d pos_drone_uav1;
Eigen::Vector3d vel_drone_uav1;

mavros_msgs::State current_state_uav2;
Eigen::Vector3d pos_drone_uav2;
Eigen::Vector3d vel_drone_uav2;

mavros_msgs::State current_state_uav3;
Eigen::Vector3d pos_drone_uav3;
Eigen::Vector3d vel_drone_uav3;

Eigen::Vector3d Takeoff_position_uav1 = Eigen::Vector3d(0.0,0.0,0.0);
Eigen::Vector3d Takeoff_position_uav2 = Eigen::Vector3d(0.0,0.0,0.0);
Eigen::Vector3d Takeoff_position_uav3 = Eigen::Vector3d(0.0,0.0,0.0);


Eigen::Vector3d delta_uav1 = Eigen::Vector3d(2.0,0.0,0.0);
Eigen::Vector3d delta_uav2 = Eigen::Vector3d(-2.0,2.0,0.0);
Eigen::Vector3d delta_uav3 = Eigen::Vector3d(-2.0,-2.0,0.0);

Eigen::Vector3d pos_sp(0,0,0);
double yaw_sp = 0;

mavros_msgs::PositionTarget pos_setpoint1;
mavros_msgs::PositionTarget pos_setpoint2;
mavros_msgs::PositionTarget pos_setpoint3;

Eigen::Vector3d Takeoff_position = Eigen::Vector3d(0.0,0.0,0.0);

prometheus_msgs::ControlCommand Command_Now;                      //无人机当前执行命令
prometheus_msgs::ControlCommand Command_Last;                     //无人机上一条执行命令

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float get_time_in_sec(ros::Time begin);
void prinft_command_state();
void rotation_yaw(float yaw_angle, float input[2], float output[2]);
void prinft_drone_state(float current_time);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void Command_cb(const prometheus_msgs::ControlCommand::ConstPtr& msg)
{
    Command_Now = *msg;
}
void uav1_state_sub_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state_uav1 = *msg;
}

void uav1_position_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone_uav1  = Eigen::Vector3d(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
}

void uav1_velocity_sub_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    vel_drone_uav1 = Eigen::Vector3d(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);
}

void uav2_state_sub_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state_uav2 = *msg;
}

void uav2_position_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone_uav2  = Eigen::Vector3d(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
}

void uav2_velocity_sub_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    vel_drone_uav2 = Eigen::Vector3d(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);
}
void uav3_state_sub_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state_uav3 = *msg;
}

void uav3_position_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone_uav3  = Eigen::Vector3d(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
}

void uav3_velocity_sub_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    vel_drone_uav3 = Eigen::Vector3d(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation_control_sitl");
    ros::NodeHandle nh("~");

    ros::Subscriber Command_sub = nh.subscribe<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10, Command_cb);

    ros::Subscriber uav1_state_sub = nh.subscribe<mavros_msgs::State>("/uav1/mavros/state", 10, uav1_state_sub_cb);
    ros::Subscriber uav1_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav1/mavros/local_position/pose", 100, uav1_position_sub_cb);
    ros::Subscriber uav1_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav1/mavros/local_position/velocity_local", 100, uav1_velocity_sub_cb);
    ros::Publisher uav1_setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav1/mavros/setpoint_raw/local", 10);

    ros::Subscriber uav2_state_sub = nh.subscribe<mavros_msgs::State>("/uav2/mavros/state", 10, uav1_state_sub_cb);
    ros::Subscriber uav2_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav2/mavros/local_position/pose", 100, uav2_position_sub_cb);
    ros::Subscriber uav2_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav2/mavros/local_position/velocity_local", 100, uav2_velocity_sub_cb);
    ros::Publisher uav2_setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav2/mavros/setpoint_raw/local", 10);

    ros::Subscriber uav3_state_sub = nh.subscribe<mavros_msgs::State>("/uav3/mavros/state", 10, uav3_state_sub_cb);
    ros::Subscriber uav3_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav3/mavros/local_position/pose", 100, uav3_position_sub_cb);
    ros::Subscriber uav3_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav3/mavros/local_position/velocity_local", 100, uav3_velocity_sub_cb);
    ros::Publisher uav3_setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav3/mavros/setpoint_raw/local", 10);


    ros::ServiceClient uav1_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav1/mavros/set_mode");
    ros::ServiceClient uav1_arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav1/mavros/cmd/arming");
    ros::ServiceClient uav2_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav2/mavros/set_mode");
    ros::ServiceClient uav2_arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav2/mavros/cmd/arming");
    ros::ServiceClient uav3_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav3/mavros/set_mode");
    ros::ServiceClient uav3_arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav3/mavros/cmd/arming");

    mavros_msgs::SetMode uav1_mode_cmd;
    mavros_msgs::CommandBool uav1_arm_cmd;
    uav1_arm_cmd.request.value = true;

    mavros_msgs::SetMode uav2_mode_cmd;
    mavros_msgs::CommandBool uav2_arm_cmd;
    uav2_arm_cmd.request.value = true;

    mavros_msgs::SetMode uav3_mode_cmd;
    mavros_msgs::CommandBool uav3_arm_cmd;
    uav3_arm_cmd.request.value = true;

    ros::Rate rate(10.0);

    int check_flag;
    // 这一步是为了程序运行前检查一下参数是否正确
    // 输入1,继续，其他，退出程序
    cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
    cin >> check_flag;

    if(check_flag != 1)
    {
        return -1;
    }

    // 先读取一些飞控的数据
    int i =0;
    for(i=0;i<20;i++)
    {
        ros::spinOnce();
        rate.sleep();
    }

    //Set the takeoff position
    Takeoff_position_uav1 = pos_drone_uav1;
    Takeoff_position_uav2 = pos_drone_uav2;
    Takeoff_position_uav3 = pos_drone_uav3;



    //初始化命令-
    // 默认设置：Idle模式 电机怠速旋转 等待来自上层的控制指令
    Command_Now.Command_ID = 0;
    Command_Now.Mode = Command_Now.Idle;
    Command_Now.Move_mode  = Command_Now.Reference_State.XYZ_POS;
    Command_Now.Reference_State.position_ref[0] = 0;
    Command_Now.Reference_State.position_ref[1] = 0;
    Command_Now.Reference_State.position_ref[2] = 0;
    Command_Now.Reference_State.velocity_ref[0] = 0;
    Command_Now.Reference_State.velocity_ref[1] = 0;
    Command_Now.Reference_State.velocity_ref[2] = 0;
    Command_Now.Reference_State.yaw_ref = 0;


    // 记录启控时间
    ros::Time begin_time = ros::Time::now();
    float last_time = get_time_in_sec(begin_time);
    float dt = 0;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        //执行回调函数
        ros::spinOnce();

        // 当前时间
        float cur_time = get_time_in_sec(begin_time);
        dt = cur_time  - last_time;

        last_time = cur_time;

        prinft_drone_state(cur_time);

        prinft_command_state();


        switch (Command_Now.Mode)
        {
        // 【Idle】 怠速旋转，此时可以切入offboard模式，但不会起飞。
        case Command_Now.Idle:
            //Here pls ref to mavlink_receiver.cpp
            pos_setpoint1.header.stamp = ros::Time::now();
            pos_setpoint2.header.stamp = ros::Time::now();
            pos_setpoint3.header.stamp = ros::Time::now();
            pos_setpoint1.type_mask = 0x4000;
            pos_setpoint2.type_mask = 0x4000;
            pos_setpoint3.type_mask = 0x4000;

            uav1_setpoint_raw_local_pub.publish(pos_setpoint1);
            uav2_setpoint_raw_local_pub.publish(pos_setpoint2);
            uav3_setpoint_raw_local_pub.publish(pos_setpoint3);


            if(current_state_uav1.mode != "OFFBOARD")
            {
                uav1_mode_cmd.request.custom_mode = "OFFBOARD";
                uav1_set_mode_client.call(uav1_mode_cmd);
                cout << "Setting to OFFBOARD Mode..." <<endl;
            }
            if(!current_state_uav1.armed)
            {
                uav1_arm_cmd.request.value = true;
                uav1_arming_client.call(uav1_arm_cmd);

                cout << "Arming..." <<endl;
            }

            if(current_state_uav2.mode != "OFFBOARD")
            {
                uav2_mode_cmd.request.custom_mode = "OFFBOARD";
                uav2_set_mode_client.call(uav2_mode_cmd);
                cout << "Setting to OFFBOARD Mode..." <<endl;
            }
            if(!current_state_uav2.armed)
            {
                uav2_arm_cmd.request.value = true;
                uav2_arming_client.call(uav2_arm_cmd);

                cout << "Arming..." <<endl;
            }

            if(current_state_uav3.mode != "OFFBOARD")
            {
                uav3_mode_cmd.request.custom_mode = "OFFBOARD";
                uav3_set_mode_client.call(uav1_mode_cmd);
                cout << "Setting to OFFBOARD Mode..." <<endl;
            }
            if(!current_state_uav3.armed)
            {
                uav3_arm_cmd.request.value = true;
                uav3_arming_client.call(uav3_arm_cmd);

                cout << "Arming..." <<endl;
            }

            break;

        // 【Move_ENU】 ENU系移动。只有PID算法中才有追踪速度的选项，其他控制只能追踪位置
        case Command_Now.Move_ENU:
            pos_sp = Eigen::Vector3d(Command_Now.Reference_State.position_ref[0],Command_Now.Reference_State.position_ref[1],Command_Now.Reference_State.position_ref[2]);
            yaw_sp = Command_Now.Reference_State.yaw_ref;

            pos_setpoint1.header.stamp = ros::Time::now();
            pos_setpoint2.header.stamp = ros::Time::now();
            pos_setpoint3.header.stamp = ros::Time::now();

            pos_setpoint1.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
            pos_setpoint2.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
            pos_setpoint3.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw

            pos_setpoint1.coordinate_frame = 1;
            pos_setpoint2.coordinate_frame = 1;
            pos_setpoint3.coordinate_frame = 1;

            pos_setpoint1.position.x = pos_sp[0] + delta_uav1[0];
            pos_setpoint1.position.y = pos_sp[1] + delta_uav1[1];
            pos_setpoint1.position.z = pos_sp[2] + delta_uav1[2];

            pos_setpoint2.position.x = pos_sp[0] + delta_uav2[0];
            pos_setpoint2.position.y = pos_sp[1] + delta_uav2[1];
            pos_setpoint2.position.z = pos_sp[2] + delta_uav2[2];

            pos_setpoint3.position.x = pos_sp[0] + delta_uav3[0];
            pos_setpoint3.position.y = pos_sp[1] + delta_uav3[1];
            pos_setpoint3.position.z = pos_sp[2] + delta_uav3[2];

            pos_setpoint1.yaw = yaw_sp;
            pos_setpoint2.yaw = yaw_sp;
            pos_setpoint3.yaw = yaw_sp;

            uav1_setpoint_raw_local_pub.publish(pos_setpoint1);
            uav2_setpoint_raw_local_pub.publish(pos_setpoint2);
            uav3_setpoint_raw_local_pub.publish(pos_setpoint3);

            break;

        // 【Land】 降落。当前位置原地降落，降落后会自动上锁，且切换为mannual模式
        case Command_Now.Land:

        // 【Disarm】 紧急上锁。直接上锁，不建议使用，危险。
        case Command_Now.Disarm:

        // 【PPN_land】 暂空。可进行自定义
        case Command_Now.PPN_land:



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

    int sub_mode;
    sub_mode = Command_Now.Move_mode ;

    switch(Command_Now.Mode)
    {
    case Command_Now.Move_ENU:
        cout << "Command: [ Move_ENU ] " <<endl;

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

        cout << "Yaw_setpoint : "  << Command_Now.Reference_State.yaw_ref* 180/M_PI << " [deg] " <<endl;

        break;
    case Command_Now.Move_Body:
        cout << "Command: [ Move_Body ] " <<endl;

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

        cout << "Yaw_setpoint : "  << Command_Now.Reference_State.yaw_ref * 180/M_PI<< " [deg] " <<endl;

        break;

    case Command_Now.Hold:
        cout << "Command: [ Hold ] " <<endl;
        cout << "Hold Position [X Y Z] : " << pos_sp[0] << " [ m ] "<< pos_sp[1]<<" [ m ] "<< pos_sp[2]<<" [ m ] "<<endl;
        cout << "Yaw_setpoint : "  << yaw_sp* 180/M_PI << " [deg] " <<endl;
        break;

    case Command_Now.Land:
        cout << "Command: [ Land ] " <<endl;
        cout << "Land Position [X Y Z] : " << pos_sp[0] << " [ m ] "<< pos_sp[1]<<" [ m ] "<< pos_sp[2]<<" [ m ] "<<endl;
        cout << "Yaw_setpoint : "  << yaw_sp* 180/M_PI << " [deg] " <<endl;
        break;

    case Command_Now.Disarm:
        cout << "Command: [ Disarm ] " <<endl;
        break;

    case Command_Now.PPN_land:
        cout << "Command: [ PPN_land ] " <<endl;
        break;

    case Command_Now.Idle:
        cout << "Command: [ Idle ] " <<endl;
        break;

    case Command_Now.Takeoff:
        cout << "Command: [ Takeoff ] " <<endl;
        cout << "Takeoff Position [X Y Z] : " << pos_sp[0] << " [ m ] "<< pos_sp[1]<<" [ m ] "<< pos_sp[2]<<" [ m ] "<<endl;
        cout << "Yaw_setpoint : "  << yaw_sp* 180/M_PI << " [deg] " <<endl;
        break;
    }



}

void prinft_drone_state(float current_time)
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV1 State<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

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

    cout << "Time: " << current_time <<" [s] ";

    //是否和飞控建立起连接
    if (current_state_uav1.connected == true)
    {
        cout << " [ Connected ]  ";
    }
    else
    {
        cout << " [ Unconnected ]  ";
    }

    //是否上锁
    if (current_state_uav1.armed == true)
    {
        cout << "  [ Armed ]   ";
    }
    else
    {
        cout << "  [ DisArmed ]   ";
    }

    cout << " [ " << current_state_uav1.mode<<" ]   " <<endl;

    cout << "Position_uav1 [X Y Z] : " << pos_drone_uav1[0] << " [ m ] "<< pos_drone_uav1[1]<<" [ m ] "<<pos_drone_uav1[2]<<" [ m ] "<<endl;
    cout << "Velocity_uav1 [X Y Z] : " << vel_drone_uav1[0] << " [m/s] "<< vel_drone_uav1[1]<<" [m/s] "<<vel_drone_uav1[2]<<" [m/s] "<<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV2 State<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "Time: " << current_time <<" [s] ";

    //是否和飞控建立起连接
    if (current_state_uav2.connected == true)
    {
        cout << " [ Connected ]  ";
    }
    else
    {
        cout << " [ Unconnected ]  ";
    }

    //是否上锁
    if (current_state_uav2.armed == true)
    {
        cout << "  [ Armed ]   ";
    }
    else
    {
        cout << "  [ DisArmed ]   ";
    }

    cout << " [ " << current_state_uav2.mode<<" ]   " <<endl;

    cout << "Position_uav2 [X Y Z] : " << pos_drone_uav2[0] << " [ m ] "<< pos_drone_uav2[1]<<" [ m ] "<<pos_drone_uav2[2]<<" [ m ] "<<endl;
    cout << "Velocity_uav2 [X Y Z] : " << vel_drone_uav2[0] << " [m/s] "<< vel_drone_uav2[1]<<" [m/s] "<<vel_drone_uav2[2]<<" [m/s] "<<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV3 State<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "Time: " << current_time <<" [s] ";

    //是否和飞控建立起连接
    if (current_state_uav3.connected == true)
    {
        cout << " [ Connected ]  ";
    }
    else
    {
        cout << " [ Unconnected ]  ";
    }

    //是否上锁
    if (current_state_uav3.armed == true)
    {
        cout << "  [ Armed ]   ";
    }
    else
    {
        cout << "  [ DisArmed ]   ";
    }

    cout << " [ " << current_state_uav3.mode<<" ]   " <<endl;

    cout << "Position_uav3 [X Y Z] : " << pos_drone_uav3[0] << " [ m ] "<< pos_drone_uav3[1]<<" [ m ] "<<pos_drone_uav3[2]<<" [ m ] "<<endl;
    cout << "Velocity_uav3 [X Y Z] : " << vel_drone_uav3[0] << " [m/s] "<< vel_drone_uav3[1]<<" [m/s] "<<vel_drone_uav3[2]<<" [m/s] "<<endl;
}

// 【坐标系旋转函数】- 机体系到enu系
// input是机体系,output是惯性系，yaw_angle是当前偏航角
void rotation_yaw(float yaw_angle, float input[2], float output[2])
{
    output[0] = input[0] * cos(yaw_angle) - input[1] * sin(yaw_angle);
    output[1] = input[0] * sin(yaw_angle) + input[1] * cos(yaw_angle);
}
