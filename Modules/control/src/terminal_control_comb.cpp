/***************************************************************************************************************************
 * terminal_control_comb.cpp
 *
 * Author: Hzy
 *
 * Update Time: 2023.05.02
 *
 * Introduction:  Sending ControlCommand.msg to Fcu
 ***************************************************************************************************************************/
#include <ros/ros.h>
#include <iostream>
#include <fstream>

#include <prometheus_msgs/ControlCommand.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>

#include <controller_test.h>
#include <KeyboardEvent.h>
// #include <iusc_referee/msg_manage.h>

//#define TIME_TAKEOFF 25.0
#define TIME_CONTROL 20.0
#define THRES_DISTANCE 0.15
#define TRA_WINDOW 2000
#define NODE_NAME "terminal_control_comb"

using namespace std;

// 无人机状态
prometheus_msgs::DroneState _DroneState;
// // 裁判系统指令
// iusc_referee::MissionState _MissionState;
// 规划得到的指令
geometry_msgs::Twist _PlanningCommand;
// 规划状态标识
std_msgs::Int8 _PlanningFlag;
// 即将发布的command
prometheus_msgs::ControlCommand Command_to_pub;

// 无人机状态
Eigen::Vector3f drone_pos;
Eigen::Vector3f drone_vel;
float drone_yaw;
// 规划所得速度指令和标志
float vx_cmd = 0.0;
float vy_cmd = 0.0;
float yaw_cmd = 0.0;
int plan_flag = 0;
// 起飞时间
float TIME_TAKEOFF;
float TIME_TAKEOFF2 = 10.0;
// 起飞高度
float takeoff_height;
// 裁判系统起飞指令
int arm_cmd = 0;

// 发布
ros::Publisher move_pub;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　函数声明　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void mainloop(ros::NodeHandle& nh);
void switch2offboard();
void switch2takeoff();
void switch2disarm();
void switch2land();
void switch2hold();
void switch2vel(float velx, float vely, float velz, float yaw_dis);
void switch2pos(float posx, float posy, float posz);
float cal_dis(const Eigen::Vector3f &pos_drone, const Eigen::Vector3f &pos_target);
float constrain_func(float data, float Max);
void timerCallback(const ros::TimerEvent &e)
{
  cout << ">>>>>>>>>>>>>>>> Welcome to use terminal_control_comb <<<<<<<<<<<<<<<<" << endl;
  cout << "ENTER key to control the drone: " << endl;
  cout << "1 for Arm, 2 for Arm and Takeoff, L for Land, H for Hold, 0 for Disarm" << endl;
  cout << "CTRL-C to quit." << endl;
}
// 订阅无人机状态的回调函数
void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr &msg)
{
  _DroneState = *msg;

  drone_pos[0] = _DroneState.position[0];
  drone_pos[1] = _DroneState.position[1];
  drone_pos[2] = _DroneState.position[2];
  drone_vel[0] = _DroneState.velocity[0];
  drone_vel[1] = _DroneState.velocity[1];
  drone_vel[2] = _DroneState.velocity[2];
  drone_yaw = _DroneState.attitude[2];
}
// // 订阅裁判系统指令的回调函数
// void mission_state_cb(const iusc_referee::MissionState::ConstPtr &mission_msg)
// {
//   _MissionState = *mission_msg;
//   arm_cmd = _MissionState.arm_command;
// }
// 订阅规划指令的回调函数
void planningcommand_cb(const geometry_msgs::Twist::ConstPtr &command_msg)
{
  _PlanningCommand = *command_msg;
  vx_cmd = _PlanningCommand.linear.x;
  vy_cmd = _PlanningCommand.linear.y;
  yaw_cmd = _PlanningCommand.angular.z; //偏航角rad
}
// 订阅规划状态的回调函数
void planningflag_pub(const std_msgs::Int8::ConstPtr &command_msg)
{
  _PlanningFlag = *command_msg;
  plan_flag = (int)_PlanningFlag.data;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　主函数　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
  ros::init(argc, argv, "terminal_control");
  ros::NodeHandle nh("~");
  ros::Rate rate(20);

  // 【发布】控制指令
  move_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

  // 【订阅】无人机当前状态，本话题来自根据需求自定px4_pos_estimator.cpp
  ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);
  // // 【订阅】裁判系统指令
  // ros::Subscriber mission_state_sub = nh.subscribe<iusc_referee::MissionState>("/MissionState", 10, mission_state_cb);
  
  // 【订阅】规划所得的速度指令和偏航指令
  ros::Subscriber planning_command = nh.subscribe<geometry_msgs::Twist>("/planning/control_command", 10, planningcommand_cb);
  // 【订阅】规划状态标志
  ros::Subscriber plannding_flag = nh.subscribe<std_msgs::Int8>("/planning/planning_flag", 10, planningflag_pub);

  // 从launch文件读取参数
  nh.param<float>("takeoff_height", takeoff_height, 1.0);
  nh.param<float>("TIME_TAKEOFF", TIME_TAKEOFF, 15.0);

  // 用于控制器测试的类，功能例如：生成圆形轨迹，８字轨迹等
  Controller_Test Controller_Test; // 打印参数
  Controller_Test.printf_param();

  // 初始化命令 - Idle模式 电机怠速旋转 等待来自上层的控制指令
  Command_to_pub.Mode = prometheus_msgs::ControlCommand::Idle;
  Command_to_pub.source = NODE_NAME;
  Command_to_pub.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_POS;
  Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
  Command_to_pub.Command_ID = 0;
  Command_to_pub.Reference_State.position_ref[0] = 0;
  Command_to_pub.Reference_State.position_ref[1] = 0;
  Command_to_pub.Reference_State.position_ref[2] = 0;
  Command_to_pub.Reference_State.velocity_ref[0] = 0;
  Command_to_pub.Reference_State.velocity_ref[1] = 0;
  Command_to_pub.Reference_State.velocity_ref[2] = 0;
  Command_to_pub.Reference_State.acceleration_ref[0] = 0;
  Command_to_pub.Reference_State.acceleration_ref[1] = 0;
  Command_to_pub.Reference_State.acceleration_ref[2] = 0;
  Command_to_pub.Reference_State.yaw_ref = 0;

  // 固定的浮点显示
  cout.setf(ios::fixed);
  // setprecision(n) 设显示小数精度为n位
  cout << setprecision(2);
  // 左对齐
  cout.setf(ios::left);
  // 强制显示小数点
  cout.setf(ios::showpoint);
  // 强制显示符号
  cout.setf(ios::showpos);

  // 选择键盘控制
  ros::Timer timer = nh.createTimer(ros::Duration(30.0), timerCallback);
  cout << "Keyboard input control mode" << endl;
  mainloop(nh);

  return 0;
}

void mainloop(ros::NodeHandle& nh)
{
  KeyboardEvent keyboardcontrol;
  Controller_Test Controller_Test;
  ros::Rate rate(20);
  bool arm_flag = false;
  // 航点跟踪、起飞开始时间，当前时间，航点跟踪、起飞所用时间
  ros::Time time_begin_takeoff;
  ros::Time time_now;
  float time_sec_takeoff = 0.0;
  // 航点跟踪和起飞控制的执行标志
  int pos_control_flag = 0;
  int take_off_flag = 0;
  int initial_cmd = 0;
  // 按键
  char key_now;
  char key_last;
  // 离目标位置的距离
  float distance = 0.0;
  // 比例常数
  float Kp_x = 0.8;
  float Kp_y = 0.8;
  float Kp_z = 0.6;
  float Kp_yaw = 0.5;
  // 速度幅值
  float X_VEL_MAX = 0.7;
  float Y_VEL_MAX = 0.7;
  float Z_VEL_MAX = 0.4;
  float YAW_VEL_MAX = 0.4;
  float xyratio;
  // 初始高度，跟踪航点的起始位置，目标位置，速度指令
  float initial_height = 0.0;
  float yaw_dis;
  Eigen::Vector3f initial_pos;
  Eigen::Vector3f target;
  Eigen::Vector3f vel_dis;
  initial_pos[0] = 0.0;
  initial_pos[1] = 0.0;
  initial_pos[2] = 0.0;
  target[0] = 0.0;
  target[1] = 0.0;
  target[2] = 0.0;
  vel_dis[0] = 0.0;
  vel_dis[1] = 0.0;
  vel_dis[2] = 0.0;
  yaw_dis = 0.0;

  cout << ">>>>>>>>>>>>>>>> Welcome to use terminal_control_comb <<<<<<<<<<<<<<<<" << endl;
  cout << "ENTER key to control the drone: " << endl;
  cout << "1 for Arm, 2 for Arm and Takeoff, L for Land, H for Hold, 0 for Disarm" << endl;
  cout << "CTRL-C to quit." << endl;

  while (ros::ok())
  {
    keyboardcontrol.RosWhileLoopRun();
    key_now = keyboardcontrol.GetPressedKey();
    
    switch (key_now)
    {
    case U_KEY_NONE:
      if (key_last != U_KEY_NONE)
      {
        // to be continued.
      }
      sleep(1.0);
      break;

    // 数字1（非小键盘数字）：解锁及切换到OFFBOARD模式
    case U_KEY_1:
      switch2offboard();
      break;

    // 数字2（非小键盘数字）：一键起飞
    case U_KEY_2:
      pos_control_flag = 0;
      take_off_flag = 0;
      switch2pos(drone_pos[0], drone_pos[1], drone_pos[2]);
      sleep(1.0);
      switch2offboard();
      sleep(2.0);
      //起飞
      switch2takeoff();
      time_begin_takeoff = ros::Time::now();
      time_sec_takeoff = 0;
      // 进入一级起飞阶段
      take_off_flag = 0;
      initial_height = drone_pos[2];
      break;

    // 键盘L：降落
    case U_KEY_L:
      pos_control_flag = 0;
      take_off_flag = 0;
      switch2land();
      sleep(1.0);
      break;

    // 键盘H：悬停
    case U_KEY_H:
      pos_control_flag = 0;
      take_off_flag = 0;
      switch2hold();      
      sleep(1.0);
      break;

    // 键盘0（非小键盘数字）：紧急停止
    case U_KEY_0:
      pos_control_flag = 0;
      take_off_flag = 0;
      switch2disarm();
      break;
    }

    arm_flag = nh.param("/ready_to_cross_forest",false);
    // 裁判系统有指令时起飞
    if (arm_flag && initial_cmd == 1)
    {
      // 起飞
      pos_control_flag = 0;
      take_off_flag = 0;
      switch2pos(drone_pos[0], drone_pos[1], drone_pos[2]);
      sleep(1.0);
      switch2offboard();
      sleep(2.0);
      //起飞
      switch2takeoff();
      time_begin_takeoff = ros::Time::now();
      time_sec_takeoff = 0;
      // 进入一级起飞阶段
      take_off_flag = 0;
      initial_height = drone_pos[2];
      // 起飞指令关闭
      initial_cmd = 0;
    }

    // 一级起飞阶段
    if (take_off_flag == 1)
    {
        time_now = ros::Time::now();
        time_sec_takeoff = time_now.sec - time_begin_takeoff.sec;
        cout << "take off for: " << time_sec_takeoff << "[s]. " << _DroneState.mode << endl;
        if (time_sec_takeoff >= TIME_TAKEOFF)
        {
            // 进入二级起飞阶段
            take_off_flag = 2;
            time_begin_takeoff = ros::Time::now();
            time_sec_takeoff = 0;
            // 给定航点
            initial_pos[0] = drone_pos[0];
            initial_pos[1] = drone_pos[1];
            initial_pos[2] = drone_pos[2] + takeoff_height;
            target[0] = initial_pos[0];
            target[1] = initial_pos[1];
            target[2] = initial_pos[2];
        }
    }

    // 二级起飞阶段
    if (take_off_flag == 2)
    {
        time_now = ros::Time::now();
        time_sec_takeoff = time_now.sec - time_begin_takeoff.sec;
        cout << "take off again for: " << time_sec_takeoff << "[s]. " << _DroneState.mode << endl;
        
        vel_dis[0] = 0;
        vel_dis[1] = 0;
        yaw_dis = 0;
        vel_dis[2] = Kp_z * (target[2] - drone_pos[2]);
        vel_dis[0] = constrain_func(vel_dis[0], X_VEL_MAX);
        vel_dis[1] = constrain_func(vel_dis[1], Y_VEL_MAX);
        vel_dis[2] = constrain_func(vel_dis[2], Z_VEL_MAX);
        yaw_dis = constrain_func(yaw_dis, YAW_VEL_MAX);
        // 发布
        switch2vel(vel_dis[0], vel_dis[1], vel_dis[2], yaw_dis);

        if (time_sec_takeoff >= TIME_TAKEOFF2)
        {
            // 进入运动控制状态
            take_off_flag = 0;
            pos_control_flag = 1;
            // 跟踪航点
            initial_pos[0] = drone_pos[0];
            initial_pos[1] = drone_pos[1];
            initial_pos[2] = drone_pos[2];
            target[0] = initial_pos[0];
            target[1] = initial_pos[1];
            target[2] = initial_pos[2];
        }
    }

    // 规划阶段结束降落
    if (plan_flag == 2 && pos_control_flag == 1)
    {
      pos_control_flag = 2;
      plan_flag = 0;
      //switch2land();
    }

    // 执行控制（避障）
    if (pos_control_flag == 1)
    {
      // 获取期望速度
      if (plan_flag == 0) 
      {
        vel_dis[0] = 0;
        vel_dis[1] = 0;
        yaw_dis = 0;
      } else if (plan_flag == 1)
      {
        vel_dis[0] = vx_cmd;
        vel_dis[1] = vy_cmd;
        yaw_dis = Kp_yaw * (yaw_cmd - drone_yaw);
      }
      vel_dis[2] = Kp_z * (target[2] - drone_pos[2]);
      // 限幅
      if (vel_dis[0] < 1e-5 || vel_dis[1] < 1e-5)
      {
        vel_dis[0] = constrain_func(vel_dis[0], X_VEL_MAX);
        vel_dis[1] = constrain_func(vel_dis[1], Y_VEL_MAX); 
      } else {
        xyratio = vel_dis[0] / vel_dis[1];
        if (abs(vel_dis[0]) >= abs(vel_dis[1]))
        { 
          if (abs(vel_dis[0]) > X_VEL_MAX)
          {
            vel_dis[0] = (vel_dis[0]>0)?X_VEL_MAX:-X_VEL_MAX;
            vel_dis[1] = vel_dis[0] / xyratio;
          }  
        } else {
          if (abs(vel_dis[1]) > Y_VEL_MAX)
          {
            vel_dis[1] = (vel_dis[1]>0)?Y_VEL_MAX:-Y_VEL_MAX;
            vel_dis[0] = vel_dis[1] * xyratio;
          } 
        }
      }
      vel_dis[2] = constrain_func(vel_dis[2], Z_VEL_MAX);
      yaw_dis = constrain_func(yaw_dis, YAW_VEL_MAX);
      // 发布
      switch2vel(vel_dis[0], vel_dis[1], vel_dis[2], yaw_dis);
      // 打印
      cout << "Desired Velocity:" << " vx=" << vel_dis[0] << " vy=" << vel_dis[1] << " vz=" << vel_dis[2] << endl;
      cout << "Desired Yaw Rate:" << yaw_cmd << endl;
      cout << "UAV Position:" << " x=" << drone_pos[0] << " y=" << drone_pos[1] << " z=" << drone_pos[2] << endl;
      cout << "UAV Velocity:" << " vx=" << drone_vel[0] << " vy=" << drone_vel[1] << " vz=" << drone_vel[2] << endl;
      cout << "UAV yaw:" << drone_yaw << endl;

      ros::spinOnce();
      rate.sleep();
    }

    // 执行控制（航点）
    if (pos_control_flag == 2)
    {
      // 获取目标点
      target[0] = nh.param("/target_pos_x", drone_pos[0]);
      target[1] = nh.param("/target_pos_y", drone_pos[1]);
      target[2] = nh.param("/target_pos_z", drone_pos[2]);
      
      // P控制
      vel_dis[0] = Kp_x * (target[0] - drone_pos[0]);
      vel_dis[1] = Kp_y * (target[1] - drone_pos[1]);
      vel_dis[2] = Kp_z * (target[2] - drone_pos[2]);
      yaw_dis = Kp_yaw * (0 - drone_yaw);
      // 限幅
      vel_dis[0] = constrain_func(vel_dis[0], X_VEL_MAX);
      vel_dis[1] = constrain_func(vel_dis[1], Y_VEL_MAX);
      vel_dis[2] = constrain_func(vel_dis[2], Z_VEL_MAX);
      yaw_dis = constrain_func(yaw_dis, YAW_VEL_MAX);
      // 发布
      switch2vel(vel_dis[0], vel_dis[1], vel_dis[2], yaw_dis);
      // 打印
      cout << "Desired Position:" << " x=" << target[0] << " y=" << target[1] << " z=" << target[2] << endl; 
      cout << "Desired Velocity:" << " vx=" << vel_dis[0] << " vy=" << vel_dis[1] << " vz=" << vel_dis[2] << endl;
      cout << "Desired Yaw Rate:" << yaw_cmd << endl;
      cout << "UAV Position:" << " x=" << drone_pos[0] << " y=" << drone_pos[1] << " z=" << drone_pos[2] << endl;
      cout << "UAV Velocity:" << " vx=" << drone_vel[0] << " vy=" << drone_vel[1] << " vz=" << drone_vel[2] << endl;
      cout << "UAV yaw:" << drone_yaw << endl;

      ros::spinOnce();
      rate.sleep();
    }

    key_last = key_now;
    ros::spinOnce();
    sleep(0.1);
  }
}


// 计算距离
float cal_dis(const Eigen::Vector3f &pos_drone, const Eigen::Vector3f &pos_target)
{
  Eigen::Vector3f relative;
  relative = pos_target - pos_drone;
  return relative.norm();
}

// 限幅
float constrain_func(float data, float Max)
{
  if (abs(data) > Max)
  {
    return (data > 0) ? Max : -Max;
  }
  else
  {
    return data;
  }
}

// 切换至offboard
void switch2offboard()
{	
  cout << " " << endl;
  cout << "Arm and Switch to OFFBOARD." << endl;
  Command_to_pub.header.stamp = ros::Time::now();
  Command_to_pub.Mode = prometheus_msgs::ControlCommand::Idle;
  Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
  Command_to_pub.source = NODE_NAME;
  Command_to_pub.Reference_State.yaw_ref = 999;
  move_pub.publish(Command_to_pub);
  return ;
}

// 切换至takeoff
void switch2takeoff()
{
  cout << " " << endl;
  cout << "Switch to Takeoff Mode" << endl;
  Command_to_pub.header.stamp = ros::Time::now();
  Command_to_pub.Mode = prometheus_msgs::ControlCommand::Takeoff;
  Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
  Command_to_pub.Reference_State.yaw_ref = 0.0;
  Command_to_pub.source = NODE_NAME;
  move_pub.publish(Command_to_pub);
  return ;
}

// 切换至disarm
void switch2disarm()
{
  cout << " " << endl;
  cout << "Switch to Disarm Mode." << endl;
  Command_to_pub.header.stamp = ros::Time::now();
  Command_to_pub.Mode = prometheus_msgs::ControlCommand::Disarm;
  Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
  Command_to_pub.Reference_State.velocity_ref[0] = 0;
  Command_to_pub.Reference_State.velocity_ref[1] = 0;
  Command_to_pub.Reference_State.velocity_ref[2] = 0;
  Command_to_pub.source = NODE_NAME;
  move_pub.publish(Command_to_pub);
  return ;
}

// 切换至land
void switch2land()
{
  cout << " " << endl;
  cout << "Switch to Land Mode." << endl;
  Command_to_pub.header.stamp = ros::Time::now();
  Command_to_pub.Mode = prometheus_msgs::ControlCommand::Land;
  Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
  Command_to_pub.Reference_State.velocity_ref[0] = 0;
  Command_to_pub.Reference_State.velocity_ref[1] = 0;
  Command_to_pub.Reference_State.velocity_ref[2] = 0;
  Command_to_pub.source = NODE_NAME;
  move_pub.publish(Command_to_pub);
  return ;
}

// 切换至hold
void switch2hold()
{
  cout << " " << endl;
  cout << "Switch to Hold Mode." << endl;
  Command_to_pub.header.stamp = ros::Time::now();
  Command_to_pub.Mode = prometheus_msgs::ControlCommand::Hold;
  Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
  Command_to_pub.Reference_State.velocity_ref[0] = 0;
  Command_to_pub.Reference_State.velocity_ref[1] = 0;
  Command_to_pub.Reference_State.velocity_ref[2] = 0;
  Command_to_pub.source = NODE_NAME;
  move_pub.publish(Command_to_pub);
  return ;
}

// 切换至velctl
void switch2vel(float velx, float vely, float velz, float yaw_dis)
{
  Command_to_pub.header.stamp = ros::Time::now();
  Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
  Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
  Command_to_pub.source = NODE_NAME;
  Command_to_pub.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_VEL;
  Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
  Command_to_pub.Reference_State.velocity_ref[0] = velx;
  Command_to_pub.Reference_State.velocity_ref[1] = vely;
  Command_to_pub.Reference_State.velocity_ref[2] = velz;
  Command_to_pub.Reference_State.yaw_ref = yaw_dis;
  move_pub.publish(Command_to_pub);
  return ;
}

//切换至posctl
void switch2pos(float posx, float posy, float posz)
{
  Command_to_pub.header.stamp = ros::Time::now();
  Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
  Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
  Command_to_pub.source = NODE_NAME;
  Command_to_pub.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_POS;
  Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
  Command_to_pub.Reference_State.position_ref[0] = posx;
  Command_to_pub.Reference_State.position_ref[1] = posy;
  Command_to_pub.Reference_State.position_ref[2] = posz;
  move_pub.publish(Command_to_pub);
  return ;
}
