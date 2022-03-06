#ifndef RAPID_QUAD_TRAJECTORY
#define RAPID_QUAD_TRAJECTORY

#include <ros/ros.h>

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/UAVCommand.h>

#include "RapidTrajectoryGenerator.h"
#include "printf_utils.h"

using namespace std;
using namespace RapidQuadrocopterTrajectoryGenerator;

#define NODE_NAME "rapid_main"
#define MIN_DIS 0.2

class RapidQuadTrajectory
{
public:
	RapidQuadTrajectory(ros::NodeHandle &nh);
	ros::NodeHandle global_planner_nh;

private:
	// 参数
	int uav_id;
	bool sim_mode;
	double velocity_mean;
	double replan_time;

	// 订阅无人机状态、目标点
	ros::Subscriber goal_sub;
	ros::Subscriber uav_state_sub;

	// 发布控制指令
	ros::Publisher uav_cmd_pub;
	ros::Publisher optimal_path_pub;
	ros::Timer mainloop_timer;
	ros::Timer trajectory_tracking_timer;
	ros::Timer debug_timer;

	prometheus_msgs::UAVState uav_state; // 无人机状态
	nav_msgs::Odometry uav_odom;
	Eigen::Vector3d uav_pos;	 // 无人机位置
	Eigen::Vector3d uav_vel;	 // 无人机速度
	Eigen::Quaterniond uav_quat; // 无人机四元数
	double uav_yaw;

	prometheus_msgs::UAVCommand uav_command;

	nav_msgs::Path optimal_path;

	double distance_to_goal;

	// 规划器状态
	bool odom_ready;
	bool drone_ready;
	bool goal_ready;
	bool path_ok;

	// 规划初始状态及终端状态
	Eigen::Vector3d goal_pos;

	Vec3 gravity = Vec3(0, 0, -9.81); //[m/s**2]

	// 初始状态（RapidQuad库自定义数据格式）
	Vec3 pos0 = Vec3(0, 0, 0); //position
	Vec3 vel0 = Vec3(0, 0, 0); //velocity
	Vec3 acc0 = Vec3(0, 0, 0); //acceleration

	//目标状态（RapidQuad库自定义数据格式）
	Vec3 posf = Vec3(0, 0, 0); //position
	Vec3 velf = Vec3(0, 0, 0); //velocity
	Vec3 accf = Vec3(0, 0, 0); //acceleration

	double time_now;
	double total_time = 5.0; // 期望时间
	// 轨迹参数
	float Alpha[3], Beta[3], Gamma[3];

	ros::Time tra_start_time;
	float tra_running_time;

	// 五种状态机
	enum EXEC_STATE
	{
		WAIT_GOAL,
		PLANNING,
		TRACKING,
		LANDING,
	};
	EXEC_STATE exec_state;

	void goal_cb(const geometry_msgs::PoseStampedConstPtr &msg);
	void uav_state_cb(const prometheus_msgs::UAVStateConstPtr &msg);
	void mainloop_cb(const ros::TimerEvent &e);
	void trajectory_tracking_cb(const ros::TimerEvent &e);
	void debug_cb(const ros::TimerEvent &e);

	void generate_trajectory();
	void pub_optimal_path();
	const char* GetInputFeasibilityResultName(RapidTrajectoryGenerator::InputFeasibilityResult fr);
	const char* GetStateFeasibilityResultName(RapidTrajectoryGenerator::StateFeasibilityResult fr);
};

#endif