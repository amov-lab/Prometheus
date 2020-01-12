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


# define VEL_XY 0.2
# define VEL_Z 0.2
# define VEL_YAW 0.1

using namespace std;

prometheus_msgs::ControlCommand Command_Now;
mavros_msgs::State current_state;                       //无人机当前状态[包含上锁状态 模式] (从飞控中读取)
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_control");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    ros::Publisher move_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);
    
    // 【订阅】无人机当前状态 - 来自飞控
    //  本话题来自飞控(通过/plugins/sys_status.cpp)
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);

    // 【服务】修改系统模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    mavros_msgs::SetMode mode_cmd;

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

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

    while (ros::ok())
    {
        keyboardcontrol.RosWhileLoopRun();
        key_now = keyboardcontrol.GetPressedKey();
        switch (key_now)
        {
        //悬停, 应当只发送一次, 不需要循环发送
        case U_KEY_NONE:

            if (key_last != U_KEY_NONE && current_state.mode == "OFFBOARD")
            {
              cout << "[keyboard_control]: The drone is in Hold Mode. Waiting for keyboard command" <<endl;

              Command_Now.header.stamp = ros::Time::now();
              Command_Now.Mode = prometheus_msgs::ControlCommand::Hold;
              Command_Now.Command_ID = Command_Now.Command_ID + 1;
              move_pub.publish(Command_Now);
            }
        
          break;

        //起飞要维持起飞的模式?
        case U_KEY_T:

          cout << "[keyboard_control]: The drone is in Takeoff Mode." <<endl;

          Command_Now.header.stamp = ros::Time::now();
          Command_Now.Mode = prometheus_msgs::ControlCommand::Takeoff;
          Command_Now.Command_ID = Command_Now.Command_ID + 1;
          move_pub.publish(Command_Now);

          sleep(5.0);
        
          break;

        // 运动逻辑需要修改,应当是模拟遥控器的定点模式,但是如何同时接收多个键盘按键?
        // 向前匀速运动
        case U_KEY_W:

          cout << "[keyboard_control]: Moving Forward." <<endl;

          Command_Now.header.stamp = ros::Time::now();
          Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
          Command_Now.Command_ID = Command_Now.Command_ID + 1;
          Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
          Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_Now.Reference_State.position_ref[0]     = VEL_XY;
          Command_Now.Reference_State.position_ref[1]     = 0;
          Command_Now.Reference_State.position_ref[2]     = 0;
          Command_Now.Reference_State.velocity_ref[0]     = 0;
          Command_Now.Reference_State.velocity_ref[1]     = 0;
          Command_Now.Reference_State.velocity_ref[2]     = 0;
          Command_Now.Reference_State.yaw_ref             = 0;
          move_pub.publish(Command_Now);

          sleep(1.0);
        
          break;
        
        // 向后匀速运动
        case U_KEY_S:
        
          cout << "[keyboard_control]: Moving Back." <<endl;
          Command_Now.header.stamp = ros::Time::now();
          Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
          Command_Now.Command_ID = Command_Now.Command_ID + 1;
          Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
          Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_Now.Reference_State.position_ref[0]     = -VEL_XY;
          Command_Now.Reference_State.position_ref[1]     = 0;
          Command_Now.Reference_State.position_ref[2]     = 0;
          Command_Now.Reference_State.velocity_ref[0]     = 0;
          Command_Now.Reference_State.velocity_ref[1]     = 0;
          Command_Now.Reference_State.velocity_ref[2]     = 0;
          Command_Now.Reference_State.yaw_ref             = 0;
          move_pub.publish(Command_Now);

          sleep(1.0);

          break;

        // 向左匀速运动
        case U_KEY_A:
        
          cout << "[keyboard_control]: Moving Left." <<endl;
          Command_Now.header.stamp = ros::Time::now();
          Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
          Command_Now.Command_ID = Command_Now.Command_ID + 1;
          Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
          Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_Now.Reference_State.position_ref[0]     = 0;
          Command_Now.Reference_State.position_ref[1]     = VEL_XY;
          Command_Now.Reference_State.position_ref[2]     = 0;
          Command_Now.Reference_State.velocity_ref[0]     = 0;
          Command_Now.Reference_State.velocity_ref[1]     = 0;
          Command_Now.Reference_State.velocity_ref[2]     = 0;
          Command_Now.Reference_State.yaw_ref             = 0;
          move_pub.publish(Command_Now);

          sleep(1.0);

          break;

        // 向右匀速运动
        case U_KEY_D:
        
          cout << "[keyboard_control]: Moving Right." <<endl;
          Command_Now.header.stamp = ros::Time::now();
          Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
          Command_Now.Command_ID = Command_Now.Command_ID + 1;
          Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
          Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_Now.Reference_State.position_ref[0]     = 0;
          Command_Now.Reference_State.position_ref[1]     = -VEL_XY;
          Command_Now.Reference_State.position_ref[2]     = 0;
          Command_Now.Reference_State.velocity_ref[0]     = 0;
          Command_Now.Reference_State.velocity_ref[1]     = 0;
          Command_Now.Reference_State.velocity_ref[2]     = 0;
          Command_Now.Reference_State.yaw_ref             = 0;
          move_pub.publish(Command_Now);

          sleep(1.0);

          break;

        // 向上运动
        case U_KEY_P:

          cout << "[keyboard_control]: Moving Up." <<endl;

          Command_Now.header.stamp = ros::Time::now();
          Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
          Command_Now.Command_ID = Command_Now.Command_ID + 1;
          Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
          Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_Now.Reference_State.position_ref[0]     = 0;
          Command_Now.Reference_State.position_ref[1]     = 0;
          Command_Now.Reference_State.position_ref[2]     = VEL_Z;
          Command_Now.Reference_State.velocity_ref[0]     = 0;
          Command_Now.Reference_State.velocity_ref[1]     = 0;
          Command_Now.Reference_State.velocity_ref[2]     = 0;
          Command_Now.Reference_State.yaw_ref             = 0;
          move_pub.publish(Command_Now);
        
          sleep(1.0);

          break;

        // 向下运动
        case U_KEY_L:

          cout << "[keyboard_control]: Moving Down." <<endl;

          Command_Now.header.stamp = ros::Time::now();
          Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
          Command_Now.Command_ID = Command_Now.Command_ID + 1;
          Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
          Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_Now.Reference_State.position_ref[0]     = 0;
          Command_Now.Reference_State.position_ref[1]     = 0;
          Command_Now.Reference_State.position_ref[2]     = -VEL_Z;
          Command_Now.Reference_State.velocity_ref[0]     = 0;
          Command_Now.Reference_State.velocity_ref[1]     = 0;
          Command_Now.Reference_State.velocity_ref[2]     = 0;
          Command_Now.Reference_State.yaw_ref             = 0;
          move_pub.publish(Command_Now);

          sleep(1.0);
        
          break;

        // 偏航运动，左转 （这个里偏航控制的是位置 不是速度）
        case U_KEY_Q:

          cout << "[keyboard_control]: Turn Left." <<endl;

          Command_Now.header.stamp = ros::Time::now();
          Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
          Command_Now.Command_ID = Command_Now.Command_ID + 1;
          Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
          Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_Now.Reference_State.position_ref[0]     = 0;
          Command_Now.Reference_State.position_ref[1]     = 0;
          Command_Now.Reference_State.position_ref[2]     = 0;
          Command_Now.Reference_State.velocity_ref[0]     = 0;
          Command_Now.Reference_State.velocity_ref[1]     = 0;
          Command_Now.Reference_State.velocity_ref[2]     = 0;
          Command_Now.Reference_State.yaw_ref             = VEL_YAW;
          move_pub.publish(Command_Now);

          sleep(1.0);
        
          break;

        // 偏航运动，右转
        case U_KEY_E:

          cout << "[keyboard_control]: Turn RIGHT." <<endl;

          Command_Now.header.stamp = ros::Time::now();
          Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
          Command_Now.Command_ID = Command_Now.Command_ID + 1;
          Command_Now.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
          Command_Now.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_Now.Reference_State.position_ref[0]     = 0;
          Command_Now.Reference_State.position_ref[1]     = 0;
          Command_Now.Reference_State.position_ref[2]     = 0;
          Command_Now.Reference_State.velocity_ref[0]     = 0;
          Command_Now.Reference_State.velocity_ref[1]     = 0;
          Command_Now.Reference_State.velocity_ref[2]     = 0;
          Command_Now.Reference_State.yaw_ref             = -VEL_YAW;
          move_pub.publish(Command_Now);

          sleep(1.0);
        
          break;


        //切换到offboard模式
        case U_KEY_O:
        
          if(current_state.mode != "OFFBOARD")
          {
              mode_cmd.request.custom_mode = "OFFBOARD";
              set_mode_client.call(mode_cmd);
              cout << "[keyboard_control]: Setting to OFFBOARD Mode..." <<endl;

          }else
          {
              cout << "[keyboard_control]: Set to OFFBOARD Mode Susscess!!!" <<endl;
          }

          break;
        

        // 解锁
        case U_KEY_SPACE:
        
          if(!current_state.armed)
          {
              arm_cmd.request.value = true;
              arming_client.call(arm_cmd);

              cout << "[keyboard_control]: Arming..." <<endl;

          }else
          {
              cout << "[keyboard_control]: Arm Susscess!!!" <<endl;
          }

          break;

        }

        key_last = key_now;
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
