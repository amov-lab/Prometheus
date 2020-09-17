#ifndef _ASTAR_H
#define _ASTAR_H

// #include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>


// #include "grad_spline/sdf_map.h"
// #include "plan_env/edt_environment.h"
#include <boost/functional/hash.hpp>
#include <queue>

#include <sensor_msgs/PointCloud2.h>

#include "occupy_map.h"
#include "tools.h"
#include "message_utils.h"

namespace global_planner
{
#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

extern ros::Publisher message_pub;

class Node
{
public:
  /* -------------------- */
  Eigen::Vector3i index;
  Eigen::Vector3d position;
  double g_score, f_score;
  Node* parent;
  char node_state;

  double time;  // dyn
  int time_idx;

  /* -------------------- */
  Node()
  {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~Node(){};
};
typedef Node* NodePtr;

class NodeComparator0
{
public:
  bool operator()(NodePtr node1, NodePtr node2)
  {
    return node1->f_score > node2->f_score;
  }
};

template <typename T>
struct matrix_hash0 : std::unary_function<T, size_t>
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

class NodeHashTable0
{
private:
  /* data */
  std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash0<Eigen::Vector3i>> data_3d_;

public:
  NodeHashTable0(/* args */)
  {
  }
  ~NodeHashTable0()
  {
  }
  void insert(Eigen::Vector3i idx, NodePtr node)
  {
    data_3d_.insert(std::make_pair(idx, node));
  }

  NodePtr find(Eigen::Vector3i idx)
  {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }

  void clear()
  {
    data_3d_.clear();
  }
};


class Astar{
private:

  /* ---------- main data structure ---------- */
  std::vector<NodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  NodeHashTable0 expanded_nodes_;
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;
  std::vector<NodePtr> path_nodes_;  // retrive the final path

  /* ---------- parameter ---------- */
  /* search */
  double lambda_heu_;
  double margin_;
  int allocate_num_;
  double tie_breaker_;
  int is_2D;
  double fly_height;

  /* ---------- record data ---------- */
  bool has_path_ = false;
  Eigen::Vector3d goal_pos;

  /* map */
  sensor_msgs::PointCloud2ConstPtr global_env_;
  std::vector<int> occupancy_buffer_;  // 0 is free, 1 is occupied
  double resolution_, inv_resolution_;
  Eigen::Vector3d origin_, map_size_3d_;
  bool has_global_point;


  /* helper */
  Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
  void indexToPos(Eigen::Vector3i id, Eigen::Vector3d &pos);
  void retrievePath(NodePtr end_node);

  /* heuristic function */
  double getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
  double getManhHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
  double getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);

  Occupy_map::Ptr Occupy_map_ptr;

public:
  Astar() {}
  ~Astar();

  enum
  {
    REACH_END = 1,
    NO_PATH = 2
  };

  /* main API */
  void setParam(ros::NodeHandle& nh);
  void init(ros::NodeHandle& nh);
  void reset();
  bool check_safety(Eigen::Vector3d &cur_pos, double safe_distance);
  
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

  void setEnvironment(const sensor_msgs::PointCloud2ConstPtr & global_point);
  std::vector<Eigen::Vector3d> getPath();
  std::vector<NodePtr> getVisitedNodes();

  typedef std::shared_ptr<Astar> Ptr;

};


}

#endif