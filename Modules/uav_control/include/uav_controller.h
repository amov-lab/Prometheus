#ifndef UAV_CONTROLLER_H
#define UAV_CONTROLLER_H

#include <ros/ros.h>
#include <bitset>
#include <Eigen/Eigen>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/CommandLong.h>

#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/MultiAgentState.h>
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/MavrosInterface.h>

#include <nav_msgs/Odometry.h>

#include "message_utils.h"
#include "geometry_utils.h"
#include "printf_utils.h"
#include "rc_input.h"
#include "math_utils.h"
#include "controller_utils.h"
#include "pos_controller_PID.h"
#include "pos_controller_UDE.h"
#include "pos_controller_NE.h"

using namespace std;

// 宏定义
#define NUM_POINT 2 // 打印小数点
#define CMD_TIMEOUT 0.1

class UAV_controller
{
    public:
        UAV_controller(ros::NodeHandle& nh);
        void mainloop();

        // 订阅话题
        ros::Subscriber uav_cmd_sub;
        ros::Subscriber uav_state_sub;
        ros::Subscriber px4_position_target_sub;
        ros::Subscriber px4_attitude_target_sub;
        ros::Subscriber px4_rc_sub;
        // 发布话题
        ros::Publisher px4_setpoint_raw_local_pub;
        ros::Publisher px4_setpoint_raw_attitude_pub;
        // 服务
        ros::ServiceClient px4_arming_client;
        ros::ServiceClient px4_set_mode_client;
        ros::ServiceClient px4_reboot_client;
        ros::ServiceClient px4_emergency_client;
        // 定时器
        ros::Timer debug_timer;

        ros::Time get_cmd_time{0};
        prometheus_msgs::UAVCommand uav_command;             // 指令
        prometheus_msgs::UAVCommand uav_command_last;        // 上一时刻指令
        bool get_valid_command{false};
        prometheus_msgs::UAVState uav_state;                 // 无人机状态
        prometheus_msgs::UAVState uav_state_last;                 // 无人机状态
        RC_Input rc_input;
        // bool arming_res = true;
        
        enum CONTOLLER_FLAG
        {
            PX4_ORIGIN = 0,     // PX4原生的控制算法
            PID = 1,            // PID算法
            UDE = 2,            // UDE算法    
            NE = 3              // NE算法
        };

        struct geo_fence
        {
            float x_min;
            float x_max;
            float y_min;
            float y_max;
            float z_min;
            float z_max;
        };
        geo_fence uav_geo_fence;

        // 执行状态
        enum EXEC_STATE
        {
            MANUAL_CONTROL,            // 手动定点控制
            HOVER_CONTROL,                  // 悬停状态
            COMMAND_CONTROL,            // 指令控制
            LAND_CONTROL                    // 降落
        };
        EXEC_STATE exec_state;
        EXEC_STATE last_exec_state;

        // 基本变量
        int uav_id;                   // 无人机编号
        string uav_name;                // 无人机名字
        string node_name;                
        int controller_flag;
        bool enable_external_control;
        bool sim_mode;
        bool flag_printf;                               // 是否打印
        bool quick_land;
        float Takeoff_height;                           // 默认起飞高度
        float Disarm_height;                            // 自动上锁高度
        float Land_speed;                               // 降落速度
        
        // 无人机状态量
        Eigen::Vector3d pos_drone;                      // 无人机位置
        Eigen::Vector3d vel_drone;                      // 无人机速度
        Eigen::Quaterniond q_drone;                     // 无人机四元数
        double yaw_drone;

        // 目标设定值
        Eigen::Vector3d pos_des;         
        Eigen::Vector3d vel_des;  
        Eigen::Vector3d acc_des;  
        double yaw_des; 
        Eigen::Vector3d throttle_sp;   

        // 辅助点                   
        Eigen::Vector3d Takeoff_position;  
        double Takeoff_yaw;             
        Eigen::Vector3d Hover_position;            
        double Hover_yaw;
        ros::Time last_set_hover_pose_time;

        // 无人机控制算法相关
        Eigen::Quaterniond u_q_des;   // 期望姿态角（四元数）
        Eigen::Vector4d u_att;                  // 期望姿态角（rad）+期望油门（0-1）

        pos_controller_PID pos_controller_pid;
        pos_controller_UDE pos_controller_ude;
        pos_controller_NE pos_controller_ne;

        // PX4中的位置设定值（用于验证控制指令是否正确发送）
        Eigen::Vector3d px4_pos_target;
        // PX4中的速度设定值（用于验证控制指令是否正确发送）
        Eigen::Vector3d px4_vel_target;
        // PX4中的加速度设定值（用于验证控制指令是否正确发送）
        Eigen::Vector3d px4_acc_target;
        // PX4中的姿态设定值（用于验证控制指令是否正确发送）
        Eigen::Vector3d px4_att_target;
        Eigen::Vector3d px4_rates_target;
        // PX4中的推力设定值（用于验证控制指令是否正确发送）
        float px4_thrust_target;
    private:
    
        void uav_cmd_cb(const prometheus_msgs::UAVCommand::ConstPtr& msg);
        void uav_state_cb(const prometheus_msgs::UAVState::ConstPtr& msg);
        void px4_rc_cb(const mavros_msgs::RCIn::ConstPtr& msg);
        void px4_pos_target_cb(const mavros_msgs::PositionTarget::ConstPtr& msg);
        void px4_att_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg);
        void debug_cb(const ros::TimerEvent &e);

        int check_failsafe();
        void send_idle_cmd();
        void send_pos_setpoint(const Eigen::Vector3d& pos_sp, float yaw_sp);
        void send_vel_setpoint(const Eigen::Vector3d& vel_sp, float yaw_sp);
        void send_vel_xy_pos_z_setpoint(const Eigen::Vector3d& pos_sp, const Eigen::Vector3d& vel_sp, float yaw_sp);
        void send_pos_vel_xyz_setpoint(const Eigen::Vector3d& pos_sp, const Eigen::Vector3d& vel_sp, float yaw_sp);
        void send_acc_xyz_setpoint(const Eigen::Vector3d& accel_sp, float yaw_sp);
        void send_attitude_setpoint(Eigen::Vector4d& u_att);
        
        void printf_param();
        void set_hover_pose_with_odom();
        void set_hover_pose_with_rc();
        void enable_offboard_mode();
        void enable_manual_mode();
        void arm_disarm_func(bool on_or_off);
        void enable_emergency_func();
        void reboot_PX4();
        
        void send_pos_cmd_to_px4_original_controller();
        void send_att_cmd_to_px4_attitude_controller();
        void rotation_yaw(double yaw_angle, float body_frame[2], float enu_frame[2]);

        // bool arming_cb(mavros_msgs::CommandBool::Request &req,mavros_msgs::CommandBool::Response &res);
};

#endif