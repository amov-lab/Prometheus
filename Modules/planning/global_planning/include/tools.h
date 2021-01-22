#ifndef _TOOLS_H
#define _TOOLS_H

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
#include "message_utils.h"

#include "prometheus_msgs/Message.h"

namespace Global_Planning
{
    extern ros::Publisher message_pub;

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30


class PathNode
{
public:
	/* -------------------- */
	Eigen::Vector3i index;
        Eigen::Vector3d position;
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
}
#endif
