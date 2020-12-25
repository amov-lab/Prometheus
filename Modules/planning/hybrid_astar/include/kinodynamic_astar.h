#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <string>
#include <unordered_map>

#include <boost/functional/hash.hpp>
#include <queue>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include "occupy_map.h"
#include "tools.h"
#include "message_utils.h"

#define NODE_NAME "Global_Planner [Hybrid Astar]"

namespace Global_Planning
{

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30


extern ros::Publisher message_pub;

class PathNode
{
public:
  /* -------------------- */
  Eigen::Vector3i index;
  Eigen::Matrix<double, 6, 1> state;
  double g_score, f_score;
  Eigen::Vector3d input;
  double duration;
  double time;  // dyn
  int time_idx;

  PathNode* parent;
  char node_state;

  /* -------------------- */
  PathNode()
  {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~PathNode(){};
};
typedef PathNode* PathNodePtr;


class NodeComparator
{
public:
  bool operator()(PathNodePtr node1, PathNodePtr node2) 
  { 
    return node1->f_score > node2->f_score; 
  }
};

template <typename T>
struct matrix_hash : std::unary_function<T, size_t>
{
  std::size_t operator()(T const& matrix) const
  {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i)
    {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable
{
private:
  /* data */
  std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>> data_3d_;
  std::unordered_map<Eigen::Vector4i, PathNodePtr, matrix_hash<Eigen::Vector4i>> data_4d_;

public:
  NodeHashTable(/* args */) {}
  ~NodeHashTable() {}
  void insert(Eigen::Vector3i idx, PathNodePtr node) { data_3d_.insert(make_pair(idx, node)); }
  void insert(Eigen::Vector3i idx, int time_idx, PathNodePtr node)
  {
    data_4d_.insert(make_pair(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
  }

  PathNodePtr find(Eigen::Vector3i idx)
  {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }
  PathNodePtr find(Eigen::Vector3i idx, int time_idx)
  {
    auto iter = data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
    return iter == data_4d_.end() ? NULL : iter->second;
  }

  void clear()
  {
    data_3d_.clear();
    data_4d_.clear();
  }
};

class KinodynamicAstar
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
  KinodynamicAstar(){};
  ~KinodynamicAstar();

  enum
  {
    REACH_HORIZON = 1,
    REACH_END = 2,
    NO_PATH = 3
  };

  Occupy_map::Ptr Occupy_map_ptr;

  // 重置
  void reset();
  // 初始化
  void init(ros::NodeHandle& nh);
  // 检查安全性
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
             Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool init, bool dynamic = false,
             double time_start = -1.0);
  bool check_safety(Eigen::Vector3d &cur_pos, double safe_distance);
  // 返回路径
  std::vector<Eigen::Vector3d> getKinoTraj(double delta_t);
  // 返回ros消息格式的路径
  nav_msgs::Path get_ros_path();
  
  // 返回访问过的节点
  std::vector<PathNodePtr> getVisitedNodes();

  typedef std::shared_ptr<KinodynamicAstar> Ptr;
};

} 

#endif