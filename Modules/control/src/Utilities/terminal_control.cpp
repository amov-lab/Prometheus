/***************************************************************************************************************************
* terminal_control.cpp
*
* Author: Qyp
* Edited by: Eason Hua
* Update Time: 2024.04.10
*
* Introduction:  test function for sending ControlCommand.msg
***************************************************************************************************************************/
#include <ros/ros.h>
#include <iostream>

#include <prometheus_msgs/ControlCommand.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>

#include "controller_test.h"
#include "KeyboardEvent.h"

#define VEL_XY_STEP_SIZE 0.1
#define VEL_Z_STEP_SIZE 0.1
#define YAW_STEP_SIZE 0.08
#define TRA_WINDOW 2000
#define NODE_NAME "terminal_control"

using namespace std;

//即将发布的command
prometheus_msgs::ControlCommand Command_to_pub;
//轨迹容器
std::vector<geometry_msgs::PoseStamped> posehistory_vector_;

float time_trajectory = 0.0;
//Geigraphical fence 地理围栏
Eigen::Vector2f geo_fence_x;
Eigen::Vector2f geo_fence_y;
Eigen::Vector2f geo_fence_z;

//发布
ros::Publisher move_pub;
ros::Publisher ref_trajectory_pub;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　函数声明　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void mainloop1();
void mainloop2();
void generate_com(int Move_mode, float state_desired[4]);
void Draw_in_rviz(const prometheus_msgs::PositionReference& pos_ref, bool draw_trajectory);

void timerCallback(const ros::TimerEvent& e){
    cout << ">>>>>>>>>>>>>>>> Welcome to use Prometheus Terminal Control <<<<<<<<<<<<<<<<"<< endl;
    cout << "ENTER key to control the drone: " <<endl;
    cout << "1 for Arm, Space for Takeoff, L for Land, H for Hold, 0 for Disarm, 8/9 for Trajectory tracking" <<endl;
    cout << "Move mode is fixed (XYZ_VEL,BODY_FRAME): w/s for body_x, a/d for body_y, k/m for z, q/e for body_yaw" <<endl;
    cout << "CTRL-C to quit." <<endl;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　主函数　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv){
    ros::init(argc, argv, "terminal_control");
    ros::NodeHandle nh;

    //　【发布】　控制指令
    move_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);
    //　【发布】　参考轨迹
    ref_trajectory_pub = nh.advertise<nav_msgs::Path>("/prometheus/reference_trajectory", 10);

    nh.param<float>("geo_fence/x_min", geo_fence_x[0], -20.0);
    nh.param<float>("geo_fence/x_max", geo_fence_x[1], 20.0);
    nh.param<float>("geo_fence/y_min", geo_fence_y[0], -20.0);
    nh.param<float>("geo_fence/y_max", geo_fence_y[1], 20.0);
    nh.param<float>("geo_fence/z_min", geo_fence_z[0], -0.3);
    nh.param<float>("geo_fence/z_max", geo_fence_z[1], 10.0);

    // 初始化命令 - Idle模式 电机怠速旋转 等待来自上层的控制指令
    Command_to_pub.Mode                                = prometheus_msgs::ControlCommand::Idle;
    Command_to_pub.Command_ID                          = 0;
    Command_to_pub.source = NODE_NAME;
    Command_to_pub.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
    Command_to_pub.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_to_pub.Reference_State.position_ref[0]     = 0.0;
    Command_to_pub.Reference_State.position_ref[1]     = 0.0;
    Command_to_pub.Reference_State.position_ref[2]     = 0.0;
    Command_to_pub.Reference_State.velocity_ref[0]     = 0.0;
    Command_to_pub.Reference_State.velocity_ref[1]     = 0.0;
    Command_to_pub.Reference_State.velocity_ref[2]     = 0.0;
    Command_to_pub.Reference_State.acceleration_ref[0] = 0.0;
    Command_to_pub.Reference_State.acceleration_ref[1] = 0.0;
    Command_to_pub.Reference_State.acceleration_ref[2] = 0.0;
    Command_to_pub.Reference_State.yaw_ref             = 0.0;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout << setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    //cout.setf(ios::showpos);

    // 选择通过终端输入控制或键盘控制
    int Remote_Mode;
    bool valid_Remote_Mode = false;

    while (!valid_Remote_Mode) {
        cout << ">>>>>>>>>>>>>>>> Welcome to use Prometheus Terminal Control <<<<<<<<<<<<<<<<" << endl;
        cout << "Please choose the Remote Mode: 0 for COMMAND input control, 1 for KEYBOARD input control" << endl;
        if (cin >> Remote_Mode) {
            if (Remote_Mode == 0) {
                valid_Remote_Mode = true;
                cout << "COMMAND input control mode" << endl;
                mainloop1();
            } else if (Remote_Mode == 1) {
                valid_Remote_Mode = true;
                ros::Timer timer = nh.createTimer(ros::Duration(30.0), timerCallback);
                cout << "KEYBOARD input control mode" << endl;
                mainloop2();
            } else {
                cout << "Invalid input! Please enter 0 or 1." << endl;
            }
        } else {
            // Clear error flags
            cin.clear();
            // Discard invalid input
            cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            cout << "Invalid input! Please enter an integer." << endl;
        }
    }
    
    return 0;
}


void mainloop1(){
    int Control_Mode = 0;
    bool valid_Control_Mode = false;
    int Move_mode = 0;
    bool valid_move_mode = false;
    int Move_frame = 0;
    bool valid_move_frame = false;
    int Trjectory_mode = 0;
    bool valid_trajectory_mode = false;
    // 轨迹追踪总时长，键盘控制时固定时长，指令输入控制可调
    float trajectory_total_time = 50.0;
    bool valid_total_time = false;
    float state_desired[4];
    bool valid_x_input = false;
    bool valid_y_input = false;
    bool valid_z_input = false;
    bool valid_yaw_input = false;

    //用于控制器测试的类，功能例如：生成圆形轨迹，８字轨迹等
    Controller_Test Controller_Test;    // 打印参数
    Controller_Test.printf_param();

    while(ros::ok()){
        while (!valid_Control_Mode) {
            cout << ">>>>>>>>>>>>>>>> Welcome to use Prometheus Terminal Control <<<<<<<<<<<<<<<<" << endl;
            cout << "Please choose the Command.Mode: 0 for IDLE, 1 for TAKEOFF, 2 for HOLD, 3 for LAND, 4 for MOVE, 5 for DISARM" << endl;
            cout << "Input 999 to switch to OFFBOARD mode and ARM the drone (ONLY for simulation, please use RC in experiment!!!)" << endl;
            if (cin >> Control_Mode) {
                if (Control_Mode == 0 ||
                    Control_Mode == 1 ||
                    Control_Mode == 2 ||
                    Control_Mode == 3 ||
                    Control_Mode == 4 ||
                    Control_Mode == 5 ||
                    Control_Mode == 999) {
                    valid_Control_Mode = true;
                }else{
                    cout << "Invalid input! Please enter a valid command mode." << endl;
                }
            } else {
                // Clear error flags
                cin.clear();
                // Discard invalid input
                cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                cout << "Invalid input! Please enter an integer." << endl;
            }
        }
        valid_Control_Mode = false;

        switch (Control_Mode){
            case prometheus_msgs::ControlCommand::Idle:
                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Idle;
                // TODO: why unreachable?
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
                if (Move_mode == prometheus_msgs::PositionReference::TRAJECTORY) {
                    time_trajectory = 0.0;

                    while (time_trajectory < trajectory_total_time) {
                        Command_to_pub.header.stamp = ros::Time::now();
                        Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                        Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                        Command_to_pub.source = NODE_NAME;

                        if (Trjectory_mode == 0) {
                            Command_to_pub.Reference_State = Controller_Test.Circle_trajectory_generation(
                                    time_trajectory);
                        } else if (Trjectory_mode == 1) {
                            Command_to_pub.Reference_State = Controller_Test.Eight_trajectory_generation(
                                    time_trajectory);
                        } else if (Trjectory_mode == 2) {
                            Command_to_pub.Reference_State = Controller_Test.Step_trajectory_generation(
                                    time_trajectory);
                        } else if (Trjectory_mode == 3) {
                            Command_to_pub.Reference_State = Controller_Test.Line_trajectory_generation(
                                    time_trajectory);
                        }

                        move_pub.publish(Command_to_pub);
                        // TODO: time not matching
                        time_trajectory = time_trajectory + 0.01;

                        cout << "Trajectory tracking: " << time_trajectory << " / " << trajectory_total_time << " [ s ]" << endl;

                        Draw_in_rviz(Command_to_pub.Reference_State, true);

                        ros::Duration(0.01).sleep();
                    }
                } else {
                    while (!valid_move_mode) {
                        cout << "Please choose the Command.Reference_State.Move_mode: 0 for XYZ_POS, 1 for XY_POS_Z_VEL, 2 for XY_VEL_Z_POS, 3 for XYZ_VEL, 5 for TRAJECTORY" << endl;
                        if (cin >> Move_mode) {
                            if (Move_mode == 0 ||
                                Move_mode == 1 ||
                                Move_mode == 2 ||
                                Move_mode == 3 ||
                                Move_mode == 5) {
                                valid_move_mode = true;
                            } else {
                                cout << "Invalid input! Please enter a valid move mode." << endl;
                            }
                        } else {
                            // Clear error flags
                            cin.clear();
                            // Discard invalid input
                            cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                            cout << "Invalid input! Please enter an integer." << endl;
                        }
                    }
                    valid_move_mode = false;

                    if (Move_mode == prometheus_msgs::PositionReference::TRAJECTORY) {
                        while (!valid_trajectory_mode) {
                            cout << "Please choose the trajectory type: 0 for Circle, 1 for Eight Shape, 2 for Step, 3 for Line" << endl;
                            if (cin >> Trjectory_mode) {
                                if (Trjectory_mode >= 0 && Trjectory_mode <= 3) {
                                    valid_trajectory_mode = true;
                                } else {
                                    cout << "Invalid input! Please enter a valid trajectory mode." << endl;
                                }
                            } else {
                                // Clear error flags
                                cin.clear();
                                // Discard invalid input
                                cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                                cout << "Invalid input! Please enter an integer." << endl;
                            }
                        }
                        valid_trajectory_mode = false;

                        while (!valid_total_time) {
                            cout << "Input the trajectory_total_time:" << endl;
                            if (cin >> trajectory_total_time) {
                                if (trajectory_total_time >= 1.0 && trajectory_total_time <= 100.0) {
                                    valid_total_time = true;
                                } else {
                                    cout << "Invalid input! Please enter a float between 1 and 100." << endl;
                                }
                            } else {
                                // Clear error flags
                                cin.clear();
                                // Discard invalid input
                                cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                                cout << "Invalid input! Please enter a float." << endl;
                            }
                        }
                        valid_total_time = false;

                    } else {
                        while (!valid_move_frame) {
                            cout << "Please choose the Command.Reference_State.Move_frame: 0 for ENU_FRAME, 1 for BODY_FRAME" << endl;
                            if (cin >> Move_frame) {
                                if (Move_frame == 0 || Move_frame == 1) {
                                    valid_move_frame = true;
                                } else {
                                    cout << "Invalid input! Please enter 0 or 1." << endl;
                                }
                            } else {
                                // Clear error flags
                                cin.clear();
                                // Discard invalid input
                                cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                                cout << "Invalid input! Please enter an integer." << endl;
                            }
                        }
                        valid_move_frame = false;

                        cout << "Please input the reference state [x y z yaw]: " << endl;
                        while (!valid_x_input) {
                            cout << "setpoint_t[0] --- x [m] : " << endl;
                            if (cin >> state_desired[0]) {
                                // Check if x is within the range defined by geo_fence_x
                                if (state_desired[0] >= geo_fence_x[0] && state_desired[0] <= geo_fence_x[1]) {
                                    valid_x_input = true;
                                } else {
                                    cout << "Invalid input for x! Please enter a value between " << geo_fence_x[0] << " and " << geo_fence_x[1] << endl;
                                }
                            } else {
                                // Clear error flags
                                cin.clear();
                                // Discard invalid input
                                cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                                cout << "Invalid input! Please enter a number." << endl;
                            }
                        }
                        valid_x_input = false;

                        while (!valid_y_input) {
                            cout << "setpoint_t[1] --- y [m] : " << endl;
                            if (cin >> state_desired[1]) {
                                // Check if y is within the range defined by geo_fence_y
                                if (state_desired[1] >= geo_fence_y[0] && state_desired[1] <= geo_fence_y[1]) {
                                    valid_y_input = true;
                                } else {
                                    cout << "Invalid input for y! Please enter a value between " << geo_fence_y[0] << " and " << geo_fence_y[1] << endl;
                                }
                            } else {
                                // Clear error flags
                                cin.clear();
                                // Discard invalid input
                                cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                                cout << "Invalid input! Please enter a number." << endl;
                            }
                        }
                        valid_y_input = false;

                        while (!valid_z_input) {
                            cout << "setpoint_t[2] --- z [m] : " << endl;
                            if (cin >> state_desired[2]) {
                                // Check if z is within the range defined by geo_fence_z
                                if (state_desired[2] >= geo_fence_z[0] && state_desired[2] <= geo_fence_z[1]) {
                                    valid_z_input = true;
                                } else {
                                    cout << "Invalid input for z! Please enter a value between " << geo_fence_z[0] << " and " << geo_fence_z[1] << endl;
                                }
                            } else {
                                // Clear error flags
                                cin.clear();
                                // Discard invalid input
                                cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                                cout << "Invalid input! Please enter a number." << endl;
                            }
                        }
                        valid_z_input = false;

                        while (!valid_yaw_input) {
                            cout << "setpoint_t[3] --- yaw [deg] : " << endl;
                            if (cin >> state_desired[3]) {
                                // Check if yaw is within the range
                                if (state_desired[3] >= 0 && state_desired[3] < 360) {
                                    valid_yaw_input = true;
                                } else {
                                    cout << "Invalid input for yaw! Please enter a value between 0 and 360" << endl;
                                }
                            } else {
                                // Clear error flags
                                cin.clear();
                                // Discard invalid input
                                cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                                cout << "Invalid input! Please enter a number." << endl;
                            }
                        }
                    }
                    valid_yaw_input = false;

                    Command_to_pub.header.stamp = ros::Time::now();
                    Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                    Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                    Command_to_pub.source = NODE_NAME;
                    Command_to_pub.Reference_State.Move_mode = Move_mode;
                    Command_to_pub.Reference_State.Move_frame = Move_frame;
                    // yaw_rate control
                    // Command_to_pub.Reference_State.Yaw_Rate_Mode = 1;
                    Command_to_pub.Reference_State.time_from_start = -1;
                    generate_com(Move_mode, state_desired);

                    move_pub.publish(Command_to_pub);
                }
                break;

            case prometheus_msgs::ControlCommand::Disarm:
                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Disarm;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                move_pub.publish(Command_to_pub);
                break;

            case 999:
                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Idle;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                Command_to_pub.Reference_State.yaw_ref = 999;
                move_pub.publish(Command_to_pub);
                Command_to_pub.Reference_State.yaw_ref = 0.0;
                break;
        }

        cout << "....................................................." << endl;
    }
}


void mainloop2(){
    KeyboardEvent keyboardcontrol;
    // 轨迹追踪总时长，键盘控制时固定时长，指令输入控制可调
    float trajectory_total_time = 50.0;
    Controller_Test Controller_Test;

    char key_now;
    char key_last;

    cout << ">>>>>>>>>>>>>>>> Welcome to use Prometheus Terminal Control <<<<<<<<<<<<<<<<"<< endl;
    cout << "ENTER key to control the drone: " <<endl;
    cout << "1 for Arm, Space for Takeoff, L for Land, H for Hold, 0 for Disarm, 8/9 for Trajectory tracking" <<endl;
    cout << "Move mode is fixed (XYZ_VEL,BODY_FRAME): w/s for body_x, a/d for body_y, k/m for z, q/e for body_yaw" <<endl;
    cout << "CTRL-C to quit." <<endl;

    while (ros::ok()){
        keyboardcontrol.RosWhileLoopRun();
        key_now = keyboardcontrol.GetPressedKey();
        switch (key_now){

            //悬停, 应当只发送一次, 不需要循环发送
            case U_KEY_NONE:

                if (key_last != U_KEY_NONE){
                    //to be continued.
                }
                sleep(0.5);

                break;

                // 数字1（非小键盘数字）：解锁及切换到OFFBOARD模式
            case U_KEY_1:
                cout << " " <<endl;
                cout << "Arm and Switch to OFFBOARD." <<endl;

                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Idle;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                Command_to_pub.Reference_State.yaw_ref = 999;
                move_pub.publish(Command_to_pub);
                sleep(1.0);
                break;

                // 空格：起飞
            case U_KEY_SPACE:
                cout << " " <<endl;
                cout << "Switch to Takeoff Mode." <<endl;

                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Takeoff;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.Reference_State.yaw_ref = 0.0;
                Command_to_pub.source = NODE_NAME;
                move_pub.publish(Command_to_pub);

                sleep(1.0);

                break;

                // 键盘L：降落
            case U_KEY_L:
                cout << " " <<endl;
                cout << "Switch to Land Mode." <<endl;

                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Land;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                move_pub.publish(Command_to_pub);

                break;

                // 键盘0（非小键盘数字）：紧急停止
            case U_KEY_0:
                cout << " " <<endl;
                cout << "Switch to Disarm Mode." <<endl;

                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Disarm;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                move_pub.publish(Command_to_pub);

                break;

                //起飞要维持起飞的模式?
            case U_KEY_T:
                cout << " " <<endl;
                cout << "Switch to Takeoff Mode." <<endl;

                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Takeoff;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                move_pub.publish(Command_to_pub);

                sleep(2.0);

                break;

                //起飞要维持起飞的模式?
            case U_KEY_H:
                cout << " " <<endl;
                cout << "Switch to Hold Mode." <<endl;

                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Hold;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                Command_to_pub.Reference_State.position_ref[0]     = 0.0;
                Command_to_pub.Reference_State.position_ref[1]     = 0.0;
                Command_to_pub.Reference_State.position_ref[2]     = 0.0;
                Command_to_pub.Reference_State.velocity_ref[0]     = 0.0;
                Command_to_pub.Reference_State.velocity_ref[1]     = 0.0;
                Command_to_pub.Reference_State.velocity_ref[2]     = 0.0;
                Command_to_pub.Reference_State.acceleration_ref[0] = 0.0;
                Command_to_pub.Reference_State.acceleration_ref[1] = 0.0;
                Command_to_pub.Reference_State.acceleration_ref[2] = 0.0;
                move_pub.publish(Command_to_pub);

                sleep(1.0);

                break;

                // 向前匀速运动
            case U_KEY_W:

                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                Command_to_pub.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_VEL;
                Command_to_pub.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
                Command_to_pub.Reference_State.velocity_ref[0]     += VEL_XY_STEP_SIZE;
                move_pub.publish(Command_to_pub);

                cout << " " <<endl;
                cout << "Current Velocity [X Y Z]: " << Command_to_pub.Reference_State.velocity_ref[0] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[1] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[2] << " [m/s] "<<endl;

                sleep(0.1);

                break;

                // 向后匀速运动
            case U_KEY_S:


                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                Command_to_pub.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_VEL;
                Command_to_pub.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
                Command_to_pub.Reference_State.velocity_ref[0]     -= VEL_XY_STEP_SIZE;
                move_pub.publish(Command_to_pub);
                cout << " " <<endl;
                cout << "Current Velocity [X Y Z]: " << Command_to_pub.Reference_State.velocity_ref[0] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[1] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[2] << " [m/s] "<<endl;

                sleep(0.1);

                break;

                // 向左匀速运动
            case U_KEY_A:

                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                Command_to_pub.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_VEL;
                Command_to_pub.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
                Command_to_pub.Reference_State.velocity_ref[1]     += VEL_XY_STEP_SIZE;
                move_pub.publish(Command_to_pub);

                cout << " " <<endl;
                cout << "Current Velocity [X Y Z]: " << Command_to_pub.Reference_State.velocity_ref[0] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[1] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[2] << " [m/s] "<<endl;

                sleep(0.1);

                break;

                // 向右匀速运动
            case U_KEY_D:

                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                Command_to_pub.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_VEL;
                Command_to_pub.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
                Command_to_pub.Reference_State.velocity_ref[1]     -= VEL_XY_STEP_SIZE;
                move_pub.publish(Command_to_pub);

                cout << " " <<endl;
                cout << "Current Velocity [X Y Z]: " << Command_to_pub.Reference_State.velocity_ref[0] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[1] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[2] << " [m/s] "<<endl;

                sleep(0.1);

                break;

                // 向上运动
            case U_KEY_K:


                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                Command_to_pub.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_VEL;
                Command_to_pub.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
                Command_to_pub.Reference_State.velocity_ref[2]     += VEL_Z_STEP_SIZE;
                move_pub.publish(Command_to_pub);

                cout << " " <<endl;
                cout << "Current Velocity [X Y Z]: " << Command_to_pub.Reference_State.velocity_ref[0] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[1] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[2] << " [m/s] "<<endl;

                sleep(0.1);

                break;

                // 向下运动
            case U_KEY_M:


                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                Command_to_pub.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_VEL;
                Command_to_pub.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
                Command_to_pub.Reference_State.velocity_ref[2]     -= VEL_Z_STEP_SIZE;
                move_pub.publish(Command_to_pub);

                cout << " " <<endl;
                cout << "Current Velocity [X Y Z]: " << Command_to_pub.Reference_State.velocity_ref[0] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[1] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[2] << " [m/s] "<<endl;

                sleep(0.1);

                break;

                // 偏航运动，左转 （这个里偏航控制的是位置 不是速度）
            case U_KEY_Q:


                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                Command_to_pub.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_VEL;
                Command_to_pub.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
                Command_to_pub.Reference_State.yaw_ref             = YAW_STEP_SIZE;
                move_pub.publish(Command_to_pub);

                cout << " " <<endl;
                cout << "Increase the Yaw angle." <<endl;

                sleep(0.1);

                break;

                // 偏航运动，右转
            case U_KEY_E:


                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                Command_to_pub.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
                Command_to_pub.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
                Command_to_pub.Reference_State.yaw_ref             = YAW_STEP_SIZE;
                move_pub.publish(Command_to_pub);

                cout << " " <<endl;
                cout << "Decrease the Yaw angle." <<endl;

                sleep(0.1);

                break;

                // 圆形追踪
            case U_KEY_9:
                time_trajectory = 0.0;
                trajectory_total_time = 50.0;
                // 需要设置
                while(time_trajectory < trajectory_total_time){
                    Command_to_pub.header.stamp = ros::Time::now();
                    Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                    Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                    Command_to_pub.source = NODE_NAME;

                    Command_to_pub.Reference_State = Controller_Test.Circle_trajectory_generation(time_trajectory);

                    move_pub.publish(Command_to_pub);
                    time_trajectory = time_trajectory + 0.01;

                    cout << "Trajectory tracking: "<< time_trajectory << " / " << trajectory_total_time  << " [ s ]" <<endl;

                    Draw_in_rviz(Command_to_pub.Reference_State, true);

                    ros::Duration(0.01).sleep();
                }
                break;

                // 8字追踪
            case U_KEY_8:
                time_trajectory = 0.0;
                trajectory_total_time = 50.0;
                // 需要设置
                while(time_trajectory < trajectory_total_time){
                    Command_to_pub.header.stamp = ros::Time::now();
                    Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                    Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                    Command_to_pub.source = NODE_NAME;

                    Command_to_pub.Reference_State = Controller_Test.Eight_trajectory_generation(time_trajectory);

                    move_pub.publish(Command_to_pub);
                    time_trajectory = time_trajectory + 0.01;

                    cout << "Trajectory tracking: "<< time_trajectory << " / " << trajectory_total_time  << " [ s ]" <<endl;

                    Draw_in_rviz(Command_to_pub.Reference_State, true);

                    ros::Duration(0.01).sleep();
                }
                break;
        }

        key_last = key_now;
        ros::spinOnce();
        sleep(0.1);
    }
}


void generate_com(int Move_mode, float state_desired[4]){
    //# Move_mode 2-bit value:
    //# 0 for position, 1 for vel, 1st for xy, 2nd for z.
    //#                   xy position     xy velocity
    //# z position       	0b00(0)       0b10(2)
    //# z velocity		0b01(1)       0b11(3)

    if(Move_mode == prometheus_msgs::PositionReference::XYZ_ACC){
        cout << "ACC control not support yet." <<endl;
    }
    if((Move_mode & 0b10) == 0) //xy channel
    {
        Command_to_pub.Reference_State.position_ref[0] = state_desired[0];
        Command_to_pub.Reference_State.position_ref[1] = state_desired[1];
        Command_to_pub.Reference_State.velocity_ref[0] = 0;
        Command_to_pub.Reference_State.velocity_ref[1] = 0;
    }else{
        Command_to_pub.Reference_State.position_ref[0] = 0;
        Command_to_pub.Reference_State.position_ref[1] = 0;
        Command_to_pub.Reference_State.velocity_ref[0] = state_desired[0];
        Command_to_pub.Reference_State.velocity_ref[1] = state_desired[1];
    }

    if((Move_mode & 0b01) == 0) //z channel
    {
        Command_to_pub.Reference_State.position_ref[2] = state_desired[2];
        Command_to_pub.Reference_State.velocity_ref[2] = 0;
    }else{
        Command_to_pub.Reference_State.position_ref[2] = 0;
        Command_to_pub.Reference_State.velocity_ref[2] = state_desired[2];
    }

    Command_to_pub.Reference_State.acceleration_ref[0] = 0;
    Command_to_pub.Reference_State.acceleration_ref[1] = 0;
    Command_to_pub.Reference_State.acceleration_ref[2] = 0;

    if(Command_to_pub.Reference_State.Yaw_Rate_Mode == 1){
        Command_to_pub.Reference_State.yaw_rate_ref = state_desired[3];
    }else{
        Command_to_pub.Reference_State.yaw_ref = state_desired[3]/180.0*M_PI;
    }
}


void Draw_in_rviz(const prometheus_msgs::PositionReference& pos_ref, bool draw_trajectory){
    geometry_msgs::PoseStamped reference_pose;

    reference_pose.header.stamp = ros::Time::now();
    reference_pose.header.frame_id = "world";

    reference_pose.pose.position.x = pos_ref.position_ref[0];
    reference_pose.pose.position.y = pos_ref.position_ref[1];
    reference_pose.pose.position.z = pos_ref.position_ref[2];

    if(draw_trajectory){
        posehistory_vector_.insert(posehistory_vector_.begin(), reference_pose);
        if(posehistory_vector_.size() > TRA_WINDOW){
            posehistory_vector_.pop_back();
        }

        nav_msgs::Path reference_trajectory;
        reference_trajectory.header.stamp = ros::Time::now();
        reference_trajectory.header.frame_id = "world";
        reference_trajectory.poses = posehistory_vector_;
        ref_trajectory_pub.publish(reference_trajectory);
    }else{
        posehistory_vector_.clear();

        nav_msgs::Path reference_trajectory;
        reference_trajectory.header.stamp = ros::Time::now();
        reference_trajectory.header.frame_id = "world";
        reference_trajectory.poses = posehistory_vector_;
        ref_trajectory_pub.publish(reference_trajectory);
    }
}
