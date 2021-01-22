#ifndef GLOBAL_PLANNING_ALG
#define GLOBAL_PLANNING_ALG

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>

#include "occupy_map.h"
#include "tools.h"
#include "message_utils.h"

using namespace std;

namespace Global_Planning{



class global_planning_alg{
public:

  enum
  {
    REACH_END = 1,
    NO_PATH = 2,
    REACH_HORIZON = 3
  };

    // 占据图类
    Occupy_map::Ptr Occupy_map_ptr;

    // 重置
    virtual void reset()=0;
    // 初始化
    virtual void init(ros::NodeHandle& nh)=0;
    // 检查安全性
    virtual bool check_safety(Eigen::Vector3d &cur_pos, double safe_distance)=0;
    // 搜索
    virtual int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
             Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool init, bool dynamic = false,
             double time_start = -1.0)=0;
    // 返回ros消息格式的路径
    virtual nav_msgs::Path get_ros_path()=0;

    typedef shared_ptr<global_planning_alg> Ptr;
};

}

#endif 
