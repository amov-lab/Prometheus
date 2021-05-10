#include <ros/ros.h>
#include <iostream>

#include <prometheus_msgs/ControlCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>

#define NODE_NAME "px4_ego_terminal_control"

using namespace std;

//即将发布的command
prometheus_msgs::ControlCommand Command_to_pub;

//发布
ros::Publisher move_pub;
ros::Publisher ego_wp_pub;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　函数声明　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void mainloop1();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　主函数　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_ego_terminal_control");
    ros::NodeHandle nh;

    //　【发布】　控制指令
    move_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    //　【发布】　参考轨迹
    ego_wp_pub = nh.advertise<nav_msgs::Path>("/waypoint_generator/waypoints", 10);

    // 初始化命令 - Idle模式 电机怠速旋转 等待来自上层的控制指令
    Command_to_pub.Mode                                = prometheus_msgs::ControlCommand::Idle;
    Command_to_pub.Command_ID                          = 0;
    Command_to_pub.source = NODE_NAME;
    Command_to_pub.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
    Command_to_pub.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_to_pub.Reference_State.position_ref[0]     = 0;
    Command_to_pub.Reference_State.position_ref[1]     = 0;
    Command_to_pub.Reference_State.position_ref[2]     = 0;
    Command_to_pub.Reference_State.velocity_ref[0]     = 0;
    Command_to_pub.Reference_State.velocity_ref[1]     = 0;
    Command_to_pub.Reference_State.velocity_ref[2]     = 0;
    Command_to_pub.Reference_State.acceleration_ref[0] = 0;
    Command_to_pub.Reference_State.acceleration_ref[1] = 0;
    Command_to_pub.Reference_State.acceleration_ref[2] = 0;
    Command_to_pub.Reference_State.yaw_ref             = 0;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    //cout.setf(ios::showpos);

    mainloop1();

    return 0;
}

void mainloop1()
{
    int Control_Mode = 0;
    int Move_mode = 0;
    float state_desired[4];

    while(ros::ok())
    {
        // Waiting for input
        cout << ">>>>>>>>>>>>>>>> Welcome to use Prometheus Terminal Control <<<<<<<<<<<<<<<<"<< endl;
        cout << "Please enter: 0 for Idle, 1 for Takeoff, 2 for Hold, 3 for Land, 4 for Move(hover control), 5 for Move(traj track control) "<<endl;
        cout << "Input 999 to switch to offboard mode and arm the drone (ONLY for simulation, please use RC in experiment!!!)"<<endl;
        cin  >> Control_Mode;

        if(Control_Mode == 999)
        {
            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Idle;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            Command_to_pub.Reference_State.yaw_ref = 999;
            move_pub.publish(Command_to_pub);
            Command_to_pub.Reference_State.yaw_ref = 0.0;
        }
        else if(Control_Mode == 4 || Control_Mode == 5)
        {
            cout << "Please input the reference state [x y z yaw]: "<< endl;
            cout << "setpoint_t[0] --- x [m] : "<< endl;
            cin >> state_desired[0];
            cout << "setpoint_t[1] --- y [m] : "<< endl;
            cin >> state_desired[1];
            cout << "setpoint_t[2] --- z [m] : "<< endl;
            cin >> state_desired[2];
            cout << "setpoint_t[3] --- yaw [du] : "<< endl;
            cin >> state_desired[3];
        }

        if(Control_Mode == 5)
        {
            Control_Mode = prometheus_msgs::ControlCommand::Move;
            Move_mode = prometheus_msgs::PositionReference::TRAJECTORY;
        }else
        {
            Move_mode = prometheus_msgs::PositionReference::XYZ_POS;
        }
        

        switch (Control_Mode)
        {
            case prometheus_msgs::ControlCommand::Idle:
                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Idle;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                move_pub.publish(Command_to_pub);
                break;

            case prometheus_msgs::ControlCommand::Takeoff:
                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Takeoff;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                move_pub.publish(Command_to_pub);
                break;

            case prometheus_msgs::ControlCommand::Hold:
                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Hold;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                move_pub.publish(Command_to_pub);
                break;
    
            case prometheus_msgs::ControlCommand::Land:
                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Land;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                move_pub.publish(Command_to_pub);
                break;

            case prometheus_msgs::ControlCommand::Move:
                if(Move_mode == prometheus_msgs::PositionReference::TRAJECTORY)
                {
                    nav_msgs::Path waypoints;
                    geometry_msgs::PoseStamped pt;
                    pt.pose.orientation = tf::createQuaternionMsgFromYaw(state_desired[3]/180*3.1415926);
                    
                    pt.pose.position.x = state_desired[0];
                    pt.pose.position.y = state_desired[1];
                    pt.pose.position.z = state_desired[2];
                    waypoints.poses.push_back(pt);     

                    waypoints.header.frame_id = std::string("world");
                    waypoints.header.stamp = ros::Time::now();
                    // 发送至ego
                    ego_wp_pub.publish(waypoints);
                    waypoints.poses.clear();
                }

                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                Command_to_pub.Reference_State.Move_mode  = Move_mode;
                Command_to_pub.Reference_State.position_ref[0] = state_desired[0];
                Command_to_pub.Reference_State.position_ref[1] = state_desired[1];
                Command_to_pub.Reference_State.position_ref[2] = state_desired[2];
                Command_to_pub.Reference_State.yaw_ref = state_desired[3];
                move_pub.publish(Command_to_pub);
                break;
            
            case prometheus_msgs::ControlCommand::Disarm:
                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Disarm;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                move_pub.publish(Command_to_pub);
                break;

            case prometheus_msgs::ControlCommand::User_Mode1:
                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::User_Mode1;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                move_pub.publish(Command_to_pub);
                break;
            
            case prometheus_msgs::ControlCommand::User_Mode2:
                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::User_Mode2;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                move_pub.publish(Command_to_pub);
                break;
        }
        
        cout << "....................................................." <<endl;
        
        sleep(1.0);
    }
}