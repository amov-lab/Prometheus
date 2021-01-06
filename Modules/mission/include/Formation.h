/*******************************************************************
 * 文件名:Formation.h
 * 
 * 作者: BOSHEN97
 * 
 * 更新时间: 2020.10.12
 * 
 * 介绍:该头文件主要声明动捕系统定位下的无人机集群功能当中的变量以及函数等,包含以下功能模块:
 * 1. 模式切换
 * 2. 队形变换
 * 3. 集群控制
 * ****************************************************************/
#ifndef FORMATION_H
#define FORMATION_H

#include "command_to_mavros.h"
#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <prometheus_msgs/ControlCommand.h>
#include <prometheus_msgs/Formation.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <Eigen/Eigen>

class formation
{
    public:

        //初始化函数,创建发布者,订阅者,变量初始化等
        void init();

        //无人机集群解锁以及切入offboard模式(px4)
        void set_formation_px4_offboard();

        //无人机集群解锁以及切入guided模式并起飞(apm)
        void set_formation_apm_guided();

        //无人机集群切入LAND模式(px4)
        void set_formation_px4_land();

        //无人机集群切入LAND模式(apm)
        void set_formation_apm_land();
        //无人机集群上锁
        void set_formation_disarmed();

        //程序是否等待函数
        void is_wait(int time);

        //模式设置函数
        void set_mode();

        //队形变换函数
        void change();

        //获取动捕系统位置数据函数
        void getpose();

        //是否为仿真函数
        void is_sim();

        //状态函数
        void state();

        //终端输入控制数据函数
        void move();

        //切换为三角队形函数
        void set_triangle();

        //切换为一字队形函数
        void set_horizontal();

        //切换为菱形队形函数
        void set_diamond();

        //切换为菱形队形过渡队形函数
        void set_diamond_stage1();

        //打印集群队形函数
        void printf_formation_type(std::string type_name);

        //集群位置控制发布函数
        void formation_pos_pub();

        //打印飞机状态函数
        void printf_formation_state();

        //集群控制函数
        void control();

        //四边形绕圈函数
        void square();

        //参数检查函数
        bool check_param();

        //获取单台无人机控制数据
        void get_uav_cmd(Eigen::Vector3d offset_pose, mavros_msgs::PositionTarget& desired_pose);

        //集群控制命令回调函数
        void ControlCallBack(const prometheus_msgs::ControlCommandConstPtr& control_msgs);

        //队形变换命令回调函数
        void FormationChangeCallBack(const prometheus_msgs::FormationConstPtr& change_msgs);

        //1号机状态数据回调函数
        void Uav1StateCallBack(const mavros_msgs::StateConstPtr &state_msgs);

        //2号机状态数据回调函数
        void Uav2StateCallBack(const mavros_msgs::StateConstPtr &state_msgs);

        //3号机状态数据回调函数
        void Uav3StateCallBack(const mavros_msgs::StateConstPtr &state_msgs);

        //4号机状态数据回调函数
        void Uav4StateCallBack(const mavros_msgs::StateConstPtr &state_msgs);

        //5号机状态数据回调函数
        void Uav5StateCallBack(const mavros_msgs::StateConstPtr &state_msgs);

        //1号机动捕位置数据回调函数
        void Uav1MocapPoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs);

        //2号机动捕位置数据回调函数
        void Uav2MocapPoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs);

        //3号机动捕位置数据回调函数
        void Uav3MocapPoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs);

        //4号机动捕位置数据回调函数
        void Uav4MocapPoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs);

        //5号机动捕位置数据回调函数
        void Uav5MocapPoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs);

        //1号机位置数据回调函数
        void Uav1PoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs);

        //2号机位置数据回调函数
        void Uav2PoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs);

        //3号机位置数据回调函数
        void Uav3PoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs);

        //4号机位置数据回调函数
        void Uav4PoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs);

        //5号机位置数据回调函数
        void Uav5PoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs);

    private:

        //创建句柄
        ros::NodeHandle n;

/*******************订阅者*******************/

        //控制命令订阅者
        ros::Subscriber cmd_sub;

        //队形变换订阅者
        ros::Subscriber formation_type_sub;

        //1号机状态数据订阅者
        ros::Subscriber uav1_state_sub;
        
        //2号机状态数据订阅者
        ros::Subscriber uav2_state_sub;

        //3号机状态数据订阅者
        ros::Subscriber uav3_state_sub;

        //4号机状态数据订阅者
        ros::Subscriber uav4_state_sub;

        //5号机状态数据订阅者
        ros::Subscriber uav5_state_sub;

        //1号机动捕位置数据订阅者
        ros::Subscriber uav1_mocap_pose_sub;

        //2号机动捕位置数据订阅者
        ros::Subscriber uav2_mocap_pose_sub;

        //3号机动捕位置数据订阅者
        ros::Subscriber uav3_mocap_pose_sub;

        //4号机动捕位置数据订阅者
        ros::Subscriber uav4_mocap_pose_sub;

        //5号机动捕位置数据订阅者
        ros::Subscriber uav5_mocap_pose_sub;

        //1号机位置数据订阅者
        ros::Subscriber uav1_pose_sub;

        //2号机位置数据订阅者
        ros::Subscriber uav2_pose_sub;

        //3号机位置数据订阅者
        ros::Subscriber uav3_pose_sub;

        //4号机位置数据订阅者
        ros::Subscriber uav4_pose_sub;

        //5号机位置数据订阅者
        ros::Subscriber uav5_pose_sub;

/*******************客户端*******************/

        ros::ServiceClient uav1_takeoff_client;

        ros::ServiceClient uav2_takeoff_client;

        ros::ServiceClient uav3_takeoff_client;

        ros::ServiceClient uav4_takeoff_client;

        ros::ServiceClient uav5_takeoff_client;

/*******************发布者*******************/

        //队形变换发布者
        ros::Publisher formation_type_pub;

        //控制命令发布者
        ros::Publisher cmd_pub;

        //1号机动捕位置数据发布者
        ros::Publisher uav1_mocap_pose_pub;

        //2号机动捕位置数据发布者
        ros::Publisher uav2_mocap_pose_pub;
        
        //3号机动捕位置数据发布者
        ros::Publisher uav3_mocap_pose_pub;

        //4号机动捕位置数据发布者
        ros::Publisher uav4_mocap_pose_pub;

        //5号机动捕位置数据发布者
        ros::Publisher uav5_mocap_pose_pub;

        //1号机控制指令发布者
        ros::Publisher uav1_local_pub;

        //1号机控制指令发布者
        ros::Publisher uav2_local_pub;

        //1号机控制指令发布者
        ros::Publisher uav3_local_pub;

        //1号机控制指令发布者
        ros::Publisher uav4_local_pub;

        //1号机控制指令发布者
        ros::Publisher uav5_local_pub;

/*******************变量*******************/

        //1号机期望位置
        mavros_msgs::PositionTarget uav1_desired_pose;
        
        //2号机期望位置
        mavros_msgs::PositionTarget uav2_desired_pose;

        //3号机期望位置
        mavros_msgs::PositionTarget uav3_desired_pose;

        //4号机期望位置
        mavros_msgs::PositionTarget uav4_desired_pose;

        //5号机期望位置
        mavros_msgs::PositionTarget uav5_desired_pose;

        //1号机当前位置
        geometry_msgs::PoseStamped uav1_current_pose;

        //2号机当前位置
        geometry_msgs::PoseStamped uav2_current_pose;

        //3号机当前位置
        geometry_msgs::PoseStamped uav3_current_pose;

        //4号机当前位置
        geometry_msgs::PoseStamped uav4_current_pose;

        //5号机当前位置
        geometry_msgs::PoseStamped uav5_current_pose;

        //1号机集群队形位置差值
        Eigen::Vector3d uav1_offset_pose;

        //2号机集群队形位置差值
        Eigen::Vector3d uav2_offset_pose;

        //3号机集群队形位置差值
        Eigen::Vector3d uav3_offset_pose;

        //4号机集群队形位置差值
        Eigen::Vector3d uav4_offset_pose;

        //5号机集群队形位置差值
        Eigen::Vector3d uav5_offset_pose;

        //1号机仿真位置差值
        Eigen::Vector3d uav1_gazebo_offset_pose;

        //2号机仿真位置差值
        Eigen::Vector3d uav2_gazebo_offset_pose;

        //3号机仿真位置差值
        Eigen::Vector3d uav3_gazebo_offset_pose;

        //4号机仿真位置差值
        Eigen::Vector3d uav4_gazebo_offset_pose;

        //5号机仿真位置差值
        Eigen::Vector3d uav5_gazebo_offset_pose;

        //1号机当前状态
        mavros_msgs::State uav1_state;

        //2号机当前状态
        mavros_msgs::State uav2_state;

        //3号机当前状态
        mavros_msgs::State uav3_state;

        //4号机当前状态
        mavros_msgs::State uav4_state;

        //5号机当前状态
        mavros_msgs::State uav5_state;

        //1号机起飞客户端变量(apm)
        mavros_msgs::CommandTOL uav1_takeoff_cmd;

        //2号机起飞客户端变量(apm)
        mavros_msgs::CommandTOL uav2_takeoff_cmd;

        //3号机起飞客户端变量(apm)
        mavros_msgs::CommandTOL uav3_takeoff_cmd;

        //4号机起飞客户端变量(apm)
        mavros_msgs::CommandTOL uav4_takeoff_cmd;

        //5号机起飞客户端变量(apm)
        mavros_msgs::CommandTOL uav5_takeoff_cmd;

        //集群队形X轴间隔距离:一字队形以及三角队形飞机之间X轴间隔距离为1倍该变量,菱形队形为2倍
        double formation_distance_x;

        //集群队形Y轴间隔距离:一字队形以及三角队形飞机之间Y轴间隔距离为1倍该变量,菱形队形为2倍
        double formation_distance_y;

        //集群队形Z轴间隔距离:Z轴距离未设置,暂留该接口
        double formation_distance_z;

        //集群解锁切入offboard模式时,无人机之间的时间间隔
        int offboard_intervals;

        //集群降落时,无人机之间的时间间隔
        int land_intervals;

        //集群切换为菱形队形或由菱形队形切换为其他队形会存在碰撞风险,因此需加入中间过渡队形,该变量为过渡队形保持的时间
        int diamond_intervals;

        //集群队形
        prometheus_msgs::Formation formation_data;

        //集群控制
        prometheus_msgs::ControlCommand control_data;

        //集群模式
        int formation_mode;

        //是否为仿真.true代表为仿真,false代表真机
        bool sim;

        //飞行控制系统:px4 或者 apm
        std::string flight_controller;

        //定位数据来源
        std::string location_source;

        //起飞高度(apm)
        double takeoff_height;

        //程序运行初始时间
        ros::Time begin_time;

/***************四机正方形绕圈变量***************/

        //正方形边长
        double square_length;

        //1,3号点飞行高度
        double point13_height;

        //2,4号点飞行高度
        double point24_height;

        //发布目标点的保持时间
        double hold_time;

        //发布第一个目标点的延长时间
        double stage1_time;


};

#endif //FORMATION_H
