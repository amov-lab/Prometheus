#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <queue>
#include <string>
#include <unordered_map>
#include <boost/functional/hash.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>

#include "occupy_map.h"
#include "tools.h"
#include "message_utils.h"
#include "global_planning_alg.h"

#define NODE_NAME "Global_Planner [Hybrid Astar]"


using namespace std;

namespace Global_Planning
{
extern ros::Publisher message_pub;

class KinodynamicAstar: public global_planning_alg
{
private:
  // 备选路径点指针容器
  std::vector<PathNodePtr> path_node_pool_;
  // 使用节点个数,迭代次数
  int use_node_num_, iter_num_;
  // 扩展的节点
  NodeHashTable expanded_nodes_;
  //　定义队列　open_set
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> open_set_;
  // 最终路径点容器
  std::vector<PathNodePtr> path_nodes_;

  // 参数
  // 启发式参数
  double lambda_heu_;
  // 最大搜索次数
  int max_search_num;
  // tie breaker
  double tie_breaker_;
  int is_2D;
  double fly_height;

  /* ---------- record data ---------- */
  //　起始速度，起始加速度
  Eigen::Vector3d start_vel_, start_acc_;
  // 终点位置，终点速度
  Eigen::Vector3d goal_pos, end_vel_;
  //　状态转移矩阵？
  Eigen::Matrix<double, 6, 6> phi_;  // state transit matrix
  //　全局点云
  sensor_msgs::PointCloud2ConstPtr global_env_;

  // 地图相关
  //　占据图容器
  std::vector<int> occupancy_buffer_;  // 0 is free, 1 is occupied
  // 地图分辨率，及分辨率的倒数
  double resolution_, inv_resolution_;
  // 时间分辨率？
  double time_resolution_, inv_time_resolution_;
  // 地图最小的点，及地图尺寸
  Eigen::Vector3d origin_, map_size_3d_;
  bool has_global_point;

  bool is_shot_succ_ = false;
  Eigen::MatrixXd coef_shot_;
  double t_shot_;
  bool has_path_ = false;

  /* ---------- parameter ---------- */
  /* search */
  double max_tau_ = 0.25;
  double init_max_tau_ = 0.8;
  double max_vel_ = 3.0;
  double max_acc_ = 3.0;
  double w_time_ = 10.0;
  double horizon_;

  double margin_;
  int check_num_;
  double time_origin_;
  
  //　辅助函数
  Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
  int timeToIndex(double time);
  void retrievePath(PathNodePtr end_node);

  /* shot trajectory */
  vector<double> cubic(double a, double b, double c, double d);
  vector<double> quartic(double a, double b, double c, double d, double e);
  bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal);
  double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time);

  /* state propagation */
  void stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1,
                    Eigen::Vector3d um, double tau);

public:

  KinodynamicAstar(){}
  ~KinodynamicAstar();

  //初始化
  void init(ros::NodeHandle& nh);
  //重置
  void reset();
  // 检查安全性
  bool check_safety(Eigen::Vector3d &cur_pos, double safe_distance);
  // 搜索
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
             Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool init, bool dynamic = false,
             double time_start = -1.0);
  // 返回路径
  std::vector<Eigen::Vector3d> getPath(double delta_t);
  // 返回ros消息格式的路径
  nav_msgs::Path get_ros_path();
  // 返回访问过的节点
  std::vector<PathNodePtr> getVisitedNodes();

  

  typedef shared_ptr<KinodynamicAstar> Ptr;
};

} 

#endif
