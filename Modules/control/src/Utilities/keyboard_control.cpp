/***************************************************************************************************************************
 * keyboard_control.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2019.01.10
 *
 * 说明: 通过键盘控制无人机运动,可用于仿真测试,真实飞行中推荐使用遥控器.
 *      1. 
 *      2. 
 *      3. 
 *      4. 
 *      5. 
 *
***************************************************************************************************************************/
#include <ros/ros.h>
#include <KeyboardEvent.h>
#include <Eigen/Eigen>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <prometheus_msgs/ControlCommand.h>
#include <controller_test.h>
#include <nav_msgs/Path.h>
#define VEL_XY_STEP_SIZE 0.1
#define VEL_Z_STEP_SIZE 0.1
#define YAW_STEP_SIZE 0.08
#define TRA_WINDOW 2000
#define NODE_NAME "keyboard_control"

float time_trajectory = 0.0;
float trajectory_total_time = 50.0;

using namespace std;
std::vector<geometry_msgs::PoseStamped> posehistory_vector_;
ros::Publisher ref_trajectory_pub;
prometheus_msgs::ControlCommand Command_Now;
mavros_msgs::State current_state;                       //无人机当前状态[包含上锁状态 模式] (从飞控中读取)
void Draw_in_rviz(const prometheus_msgs::PositionReference& pos_ref, bool draw_trajectory);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
void timerCallback(const ros::TimerEvent& e)
{
    cout << ">>>>>>>>>>>>>>>>>>>>>>>> Welcome to Prometheus <<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "ENTER key to control the drone: 1 for switch offboard and arm, Space for Takeoff, L for Land, H for Hold" <<endl;
    cout << "Move mode (XYZ_VEL,BODY_FRAME): w/s for body_x, a/d for body_y, k/m for z, q/e for body_yaw" <<endl;
    cout << "CTRL-C to quit." <<endl;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_control");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    ros::Publisher move_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);
    
    //【发布】参考轨迹
    ref_trajectory_pub = nh.advertise<nav_msgs::Path>("/prometheus/reference_trajectory", 10);

    ros::Timer timer = nh.createTimer(ros::Duration(30.0), timerCallback);

    // 圆形轨迹追踪类
    Controller_Test Controller_Test;
    Controller_Test.printf_param();

    KeyboardEvent keyboardcontrol;

    char key_now;
    char key_last;

    // 初始化命令-
    // 默认设置：Idle模式 电机怠速旋转 等待来自上层的控制指令
    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Idle;
    Command_Now.Command_ID                          = 0;
    Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0]     = 0;
    Command_Now.Reference_State.position_ref[1]     = 0;
    Command_Now.Reference_State.position_ref[2]     = 0;
    Command_Now.Reference_State.velocity_ref[0]     = 0;
    Command_Now.Reference_State.velocity_ref[1]     = 0;
    Command_Now.Reference_State.velocity_ref[2]     = 0;
    Command_Now.Reference_State.acceleration_ref[0] = 0;
    Command_Now.Reference_State.acceleration_ref[1] = 0;
    Command_Now.Reference_State.acceleration_ref[2] = 0;
    Command_Now.Reference_State.yaw_ref             = 0;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(1);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    while (ros::ok())
    {
        keyboardcontrol.RosWhileLoopRun();
        key_now = keyboardcontrol.GetPressedKey();
        switch (key_now)
        {

        //悬停, 应当只发送一次, 不需要循环发送
        case U_KEY_NONE:

          if (key_last != U_KEY_NONE)
          {
              //to be continued.
          }
          sleep(0.5);

          break;

        // 数字1（非小键盘数字）：解锁及切换到OFFBOARD模式
        case U_KEY_1:
          cout << " " <<endl;
          cout << "Arm and Switch to OFFBOARD." <<endl;
      
          Command_Now.header.stamp = ros::Time::now();
          Command_Now.Mode = prometheus_msgs::ControlCommand::Idle;
          Command_Now.Command_ID = Command_Now.Command_ID + 1;
          Command_Now.source = NODE_NAME;
          Command_Now.Reference_State.yaw_ref = 999;
          move_pub.publish(Command_Now);
          sleep(1.0);
          break;

        // 空格：起飞
        case U_KEY_SPACE:
          cout << " " <<endl;
          cout << "Switch to Takeoff Mode." <<endl;

          Command_Now.header.stamp = ros::Time::now();
          Command_Now.Mode = prometheus_msgs::ControlCommand::Takeoff;
          Command_Now.Command_ID = Command_Now.Command_ID + 1;
          Command_Now.Reference_State.yaw_ref = 0.0;
          Command_Now.source = NODE_NAME;
          move_pub.publish(Command_Now);

          sleep(1.0);

          break;

        // 键盘L：降落
        case U_KEY_L:
          cout << " " <<endl;
          cout << "Switch to Land Mode." <<endl;
      
          Command_Now.header.stamp = ros::Time::now();
          Command_Now.Mode = prometheus_msgs::ControlCommand::Land;
          Command_Now.Command_ID = Command_Now.Command_ID + 1;
          Command_Now.source = NODE_NAME;
          move_pub.publish(Command_Now);

          break;

        // 键盘0（非小键盘数字）：紧急停止
        case U_KEY_0:
          cout << " " <<endl;
          cout << "Switch to Disarm Mode." <<endl;
      
          Command_Now.header.stamp = ros::Time::now();
          Command_Now.Mode = prometheus_msgs::ControlCommand::Disarm;
          Command_Now.Command_ID = Command_Now.Command_ID + 1;
          Command_Now.source = NODE_NAME;
          move_pub.publish(Command_Now);

          break;

        //起飞要维持起飞的模式?
        case U_KEY_T:
          cout << " " <<endl;
          cout << "Switch to Takeoff Mode." <<endl;

          Command_Now.header.stamp = ros::Time::now();
          Command_Now.Mode = prometheus_msgs::ControlCommand::Takeoff;
          Command_Now.Command_ID = Command_Now.Command_ID + 1;
          Command_Now.source = NODE_NAME;
          move_pub.publish(Command_Now);

          sleep(2.0);
        
          break;

        //起飞要维持起飞的模式?
        case U_KEY_H:
          cout << " " <<endl;
          cout << "Switch to Hold Mode." <<endl;

          Command_Now.header.stamp = ros::Time::now();
          Command_Now.Mode = prometheus_msgs::ControlCommand::Hold;
          Command_Now.Command_ID = Command_Now.Command_ID + 1;
          Command_Now.source = NODE_NAME;
          Command_Now.Reference_State.position_ref[0]     = 0;
          Command_Now.Reference_State.position_ref[1]     = 0;
          Command_Now.Reference_State.position_ref[2]     = 0;
          Command_Now.Reference_State.velocity_ref[0]     = 0;
          Command_Now.Reference_State.velocity_ref[1]     = 0;
          Command_Now.Reference_State.velocity_ref[2]     = 0;
          Command_Now.Reference_State.acceleration_ref[0] = 0;
          Command_Now.Reference_State.acceleration_ref[1] = 0;
          Command_Now.Reference_State.acceleration_ref[2] = 0;
          move_pub.publish(Command_Now);

          sleep(1.0);
        
          break;

        // 向前匀速运动
        case U_KEY_W:

          Command_Now.header.stamp = ros::Time::now();
          Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
          Command_Now.Command_ID = Command_Now.Command_ID + 1;
          Command_Now.source = NODE_NAME;
          Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_VEL;
          Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_Now.Reference_State.velocity_ref[0]     += VEL_XY_STEP_SIZE;
          move_pub.publish(Command_Now);
          
          cout << " " <<endl;
          cout << "Current Velocity [X Y Z]: " << Command_Now.Reference_State.velocity_ref[0] << " [m/s] " << Command_Now.Reference_State.velocity_ref[1] << " [m/s] " << Command_Now.Reference_State.velocity_ref[2] << " [m/s] "<<endl;

          sleep(0.1);
        
          break;
        
        // 向后匀速运动
        case U_KEY_S:
        

          Command_Now.header.stamp = ros::Time::now();
          Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
          Command_Now.Command_ID = Command_Now.Command_ID + 1;
          Command_Now.source = NODE_NAME;
          Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_VEL;
          Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_Now.Reference_State.velocity_ref[0]     -= VEL_XY_STEP_SIZE;
          move_pub.publish(Command_Now);
          cout << " " <<endl;
          cout << "Current Velocity [X Y Z]: " << Command_Now.Reference_State.velocity_ref[0] << " [m/s] " << Command_Now.Reference_State.velocity_ref[1] << " [m/s] " << Command_Now.Reference_State.velocity_ref[2] << " [m/s] "<<endl;

          sleep(0.1);

          break;

        // 向左匀速运动
        case U_KEY_A:

          Command_Now.header.stamp = ros::Time::now();
          Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
          Command_Now.Command_ID = Command_Now.Command_ID + 1;
          Command_Now.source = NODE_NAME;
          Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_VEL;
          Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_Now.Reference_State.velocity_ref[1]     += VEL_XY_STEP_SIZE;
          move_pub.publish(Command_Now);
        
          cout << " " <<endl;
          cout << "Current Velocity [X Y Z]: " << Command_Now.Reference_State.velocity_ref[0] << " [m/s] " << Command_Now.Reference_State.velocity_ref[1] << " [m/s] " << Command_Now.Reference_State.velocity_ref[2] << " [m/s] "<<endl;

          sleep(0.1);

          break;

        // 向右匀速运动
        case U_KEY_D:
        
          Command_Now.header.stamp = ros::Time::now();
          Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
          Command_Now.Command_ID = Command_Now.Command_ID + 1;
          Command_Now.source = NODE_NAME;
          Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_VEL;
          Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_Now.Reference_State.velocity_ref[1]     -= VEL_XY_STEP_SIZE;
          move_pub.publish(Command_Now);

          cout << " " <<endl;
          cout << "Current Velocity [X Y Z]: " << Command_Now.Reference_State.velocity_ref[0] << " [m/s] " << Command_Now.Reference_State.velocity_ref[1] << " [m/s] " << Command_Now.Reference_State.velocity_ref[2] << " [m/s] "<<endl;

          sleep(0.1);

          break;

        // 向上运动
        case U_KEY_K:


          Command_Now.header.stamp = ros::Time::now();
          Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
          Command_Now.Command_ID = Command_Now.Command_ID + 1;
          Command_Now.source = NODE_NAME;
          Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_VEL;
          Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_Now.Reference_State.velocity_ref[2]     += VEL_Z_STEP_SIZE;
          move_pub.publish(Command_Now);

          cout << " " <<endl;
          cout << "Current Velocity [X Y Z]: " << Command_Now.Reference_State.velocity_ref[0] << " [m/s] " << Command_Now.Reference_State.velocity_ref[1] << " [m/s] " << Command_Now.Reference_State.velocity_ref[2] << " [m/s] "<<endl;

          sleep(0.1);

          break;

        // 向下运动
        case U_KEY_M:


          Command_Now.header.stamp = ros::Time::now();
          Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
          Command_Now.Command_ID = Command_Now.Command_ID + 1;
          Command_Now.source = NODE_NAME;
          Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_VEL;
          Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_Now.Reference_State.velocity_ref[2]     -= VEL_Z_STEP_SIZE;
          move_pub.publish(Command_Now);

          cout << " " <<endl;
          cout << "Current Velocity [X Y Z]: " << Command_Now.Reference_State.velocity_ref[0] << " [m/s] " << Command_Now.Reference_State.velocity_ref[1] << " [m/s] " << Command_Now.Reference_State.velocity_ref[2] << " [m/s] "<<endl;

          sleep(0.1);
        
          break;

        // 偏航运动，左转 （这个里偏航控制的是位置 不是速度）
        case U_KEY_Q:


          Command_Now.header.stamp = ros::Time::now();
          Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
          Command_Now.Command_ID = Command_Now.Command_ID + 1;
          Command_Now.source = NODE_NAME;
          Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_VEL;
          Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_Now.Reference_State.yaw_ref             = YAW_STEP_SIZE;
          move_pub.publish(Command_Now);
          
          cout << " " <<endl;
          cout << "Increase the Yaw angle." <<endl;

          sleep(0.1);
        
          break;

        // 偏航运动，右转
        case U_KEY_E:


          Command_Now.header.stamp = ros::Time::now();
          Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
          Command_Now.Command_ID = Command_Now.Command_ID + 1;
          Command_Now.source = NODE_NAME;
          Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
          Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_Now.Reference_State.yaw_ref             = YAW_STEP_SIZE;
          move_pub.publish(Command_Now);
          
          cout << " " <<endl;
          cout << "Decrease the Yaw angle." <<endl;

          sleep(0.1);
        
          break;
        
        // 圆形追踪
        case U_KEY_9:
          time_trajectory = 0.0;
          trajectory_total_time = 50.0;
          // 需要设置
          while(time_trajectory < trajectory_total_time)
          {
              Command_Now.header.stamp = ros::Time::now();
              Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
              Command_Now.Command_ID = Command_Now.Command_ID + 1;
              Command_Now.source = NODE_NAME;

              Command_Now.Reference_State = Controller_Test.Circle_trajectory_generation(time_trajectory);

              move_pub.publish(Command_Now);
              time_trajectory = time_trajectory + 0.01;

              cout << "Trajectory tracking: "<< time_trajectory << " / " << trajectory_total_time  << " [ s ]" <<endl;

              Draw_in_rviz(Command_Now.Reference_State, true);

              ros::Duration(0.01).sleep();
          }
          break;

        // 8字追踪
        case U_KEY_8:
          time_trajectory = 0.0;
          trajectory_total_time = 50.0;
          // 需要设置
          while(time_trajectory < trajectory_total_time)
          {
              Command_Now.header.stamp = ros::Time::now();
              Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
              Command_Now.Command_ID = Command_Now.Command_ID + 1;
              Command_Now.source = NODE_NAME;

              Command_Now.Reference_State = Controller_Test.Eight_trajectory_generation(time_trajectory);

              move_pub.publish(Command_Now);
              time_trajectory = time_trajectory + 0.01;

              cout << "Trajectory tracking: "<< time_trajectory << " / " << trajectory_total_time  << " [ s ]" <<endl;

              Draw_in_rviz(Command_Now.Reference_State, true);

              ros::Duration(0.01).sleep();
          }
          break;
        }

        key_last = key_now;
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
void Draw_in_rviz(const prometheus_msgs::PositionReference& pos_ref, bool draw_trajectory)
{
    geometry_msgs::PoseStamped reference_pose;

    reference_pose.header.stamp = ros::Time::now();
    reference_pose.header.frame_id = "world";

    reference_pose.pose.position.x = pos_ref.position_ref[0];
    reference_pose.pose.position.y = pos_ref.position_ref[1];
    reference_pose.pose.position.z = pos_ref.position_ref[2];

    if(draw_trajectory)
    {
        posehistory_vector_.insert(posehistory_vector_.begin(), reference_pose);
        if(posehistory_vector_.size() > TRA_WINDOW){
            posehistory_vector_.pop_back();
        }
        
        nav_msgs::Path reference_trajectory;
        reference_trajectory.header.stamp = ros::Time::now();
        reference_trajectory.header.frame_id = "world";
        reference_trajectory.poses = posehistory_vector_;
        ref_trajectory_pub.publish(reference_trajectory);
    }else
    {
        posehistory_vector_.clear();
        
        nav_msgs::Path reference_trajectory;
        reference_trajectory.header.stamp = ros::Time::now();
        reference_trajectory.header.frame_id = "world";
        reference_trajectory.poses = posehistory_vector_;
        ref_trajectory_pub.publish(reference_trajectory);
    }
}