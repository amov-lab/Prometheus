/***************************************************************************************************************************
* traj_server.cpp
*
* Author: Tao JIANG, Yuhua QI
*
* Update Time: 2021.01.05
*
***************************************************************************************************************************/
#include <ros/ros.h>
#include "prometheus_plan_manage/Bspline.h"
#include "bspline_opt/non_uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"

#include "prometheus_msgs/PositionReference.h"
#include "prometheus_msgs/ControlCommand.h"

using namespace dyn_planner;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>变量声明及定义<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
ros::Publisher state_pub, pos_cmd_pub, traj_pub;

nav_msgs::Odometry odom;

bool sim_mode;

// 控制接口
prometheus_msgs::PositionReference cmd;

bool receive_traj = false;
vector<NonUniformBspline> traj;
ros::Time time_traj_start;
int traj_id;
double traj_duration;
double t_cmd_start, t_cmd_end;

vector<Eigen::Vector3d> traj_cmd, traj_real;

Eigen::Vector3d hover_pt;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明与定义<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id) 
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;

  traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  // 发布真实轨迹
  traj_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

void drawState(Eigen::Vector3d pos, Eigen::Vector3d vec, int id,
               Eigen::Vector4d color) {
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "map";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW; // 箭头
  mk_state.action = visualization_msgs::Marker::ADD;
  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;
  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);
  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);
  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);
  // 发布当前机器人状态
  state_pub.publish(mk_state);
}

// 【订阅】处理bspline数据，生成traj：pos,vel,acc
void bsplineCallback(prometheus_plan_manage::BsplineConstPtr msg) {
  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }

  Eigen::MatrixXd ctrl_pts(msg->pts.size(), 3);
  for (int i = 0; i < msg->pts.size(); ++i) {
    Eigen::Vector3d pt;
    pt(0) = msg->pts[i].x;
    pt(1) = msg->pts[i].y;
    pt(2) = msg->pts[i].z;
    ctrl_pts.row(i) = pt.transpose();
  }

  NonUniformBspline bspline(ctrl_pts, msg->order, 0.1);
  bspline.setKnot(knots);

  time_traj_start = msg->start_time;
  traj_id = msg->traj_id;

  traj.clear();
  traj.push_back(bspline);
  traj.push_back(traj[0].getDerivative());
  traj.push_back(traj[1].getDerivative());

  traj[0].getTimeSpan(t_cmd_start, t_cmd_end);
  traj_duration = t_cmd_end - t_cmd_start;

  receive_traj = true;
}

// 【订阅】replan出现的话，更新时间，在0.25s后或轨迹运行完后轨迹发布停止
void replanCallback(std_msgs::Empty msg) {
  /* reset duration */
  const double time_out = 0.25;
  ros::Time time_now = ros::Time::now();
  double t_stop = (time_now - time_traj_start).toSec() + time_out;  //在0.25s后停止发布轨迹
  traj_duration = min(t_stop, traj_duration);
  t_cmd_end = t_cmd_start + traj_duration;
}

// 【订阅】只是用于显示
void odomCallbck(const nav_msgs::Odometry& msg) {
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O") return;

  odom = msg;

  traj_real.push_back(Eigen::Vector3d(odom.pose.pose.position.x,
                                      odom.pose.pose.position.y,
                                      odom.pose.pose.position.z));
  // 只存储最多10000个轨迹点
  if (traj_real.size() > 10000)
    traj_real.erase(traj_real.begin(), traj_real.begin() + 1000);
}

void visCallback(const ros::TimerEvent& e) 
{
  // 可视化机器人真实运动轨迹（odom）
  displayTrajWithColor(traj_real, 0.03, Eigen::Vector4d(0.925, 0.054, 0.964, 1),
                       1);
  // 可视化轨迹指令
  displayTrajWithColor(traj_cmd, 0.03, Eigen::Vector4d(1, 1, 0, 1), 2);
}

// 【发布】根据轨迹生成控制指令
void cmdCallback(const ros::TimerEvent& e) 
{
  /* no publishing before receive traj */
  if (!receive_traj) return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - time_traj_start).toSec();

  Eigen::Vector3d pos, vel, acc;

  if (t_cur < traj_duration && t_cur >= 0.0) 
  {
    pos = traj[0].evaluateDeBoor(t_cmd_start + t_cur);
    vel = traj[1].evaluateDeBoor(t_cmd_start + t_cur);
    acc = traj[2].evaluateDeBoor(t_cmd_start + t_cur);
  } else if (t_cur >= traj_duration) {
    /* hover when finish traj */
    // 如果replan超时，就悬停
    pos = traj[0].evaluateDeBoor(t_cmd_end);
    vel.setZero();
    acc.setZero();
  } else {
    cout << "[Traj server]: invalid time." << endl;
  }

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "map";

  cmd.Move_mode = prometheus_msgs::PositionReference::TRAJECTORY;  //TRAJECTORY
  cmd.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME; //ENU_FRAME
  cmd.time_from_start = t_cur;

  cmd.position_ref[0] = pos(0);
  cmd.position_ref[1] = pos(1);
  cmd.position_ref[2] = pos(2);

  cmd.velocity_ref[0] = vel(0);
  cmd.velocity_ref[1] = vel(1);
  cmd.velocity_ref[2] = vel(2);

  cmd.acceleration_ref[0] = acc(0);
  cmd.acceleration_ref[1] = acc(1);
  cmd.acceleration_ref[2] = acc(2);

  cmd.yaw_ref = 0.0;

  // 发布控制指令
  pos_cmd_pub.publish(cmd);

  drawState(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
  drawState(pos, acc, 1, Eigen::Vector4d(0, 0, 1, 1));

  traj_cmd.push_back(pos);
  if (pos.size() > 10000)
    traj_cmd.erase(traj_cmd.begin(), traj_cmd.begin() + 1000);
}

// 主函数
int main(int argc, char** argv) 
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;

  // 是否为仿真模式
  node.param("sim_mode", sim_mode, false); 

  // 订阅bspline, replan标志， odom信息（只用于显示）
  ros::Subscriber bspline_sub = node.subscribe("/prometheus/planning/bspline", 10, bsplineCallback);

  ros::Subscriber replan_sub = node.subscribe("/prometheus/fast_planning/replan", 10, replanCallback);

  ros::Subscriber odom_sub = node.subscribe("/prometheus/drone_odom", 50, odomCallbck);

  // 发布当前机器人指令状态
  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
  
  state_pub = node.advertise<visualization_msgs::Marker>("/prometheus/planning/state", 10);
  
  pos_cmd_pub = node.advertise<prometheus_msgs::PositionReference>("/prometheus/fast_planner/position_cmd", 50);
  
  // 发布轨迹控制指令，无人机实际轨迹
  ros::Timer vis_timer = node.createTimer(ros::Duration(0.2), visCallback);
  traj_pub = node.advertise<visualization_msgs::Marker>("/prometheus/planning/traj", 10);

  ros::Duration(1.0).sleep();

  cout << "[Traj server]: ready." << endl;

  ros::spin();

  return 0;
}
