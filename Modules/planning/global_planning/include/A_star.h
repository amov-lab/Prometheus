#ifndef _ASTAR_H
#define _ASTAR_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <queue>
#include <string>
#include <unordered_map>
#include <sstream>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>

#include "occupy_map.h"
#include "tools.h"
#include "message_utils.h"
#include "global_planning_alg.h"

#define NODE_NAME "Global_Planner [Astar]"


using namespace std;

namespace Global_Planning
{

extern ros::Publisher message_pub;

class Astar: public global_planning_alg
{
    private:
        // 备选路径点指针容器
        std::vector<PathNodePtr> path_node_pool_;
        // 使用节点计数器、迭代次数计数器
        int use_node_num_, iter_num_;
        // 扩展的节点
        NodeHashTable expanded_nodes_;
        // open set （根据规则已排序好）
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
        // 目标点
        Eigen::Vector3d goal_pos;

        // 地图相关
        std::vector<int> occupancy_buffer_;  
        double resolution_, inv_resolution_;
        Eigen::Vector3d origin_, map_size_3d_;
        bool has_global_point;

        // 辅助函数
        Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
        void indexToPos(Eigen::Vector3i id, Eigen::Vector3d &pos);
        void retrievePath(PathNodePtr end_node);

        // 搜索启发函数，三种形式，选用其中一种即可
        double getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
        double getManhHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
        double getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);

    public:

        Astar(){}
	~Astar();
	
	//初始化
	void init(ros::NodeHandle& nh);
	//重置
	void reset();
        // 检查安全性
        bool check_safety(Eigen::Vector3d &cur_pos, double safe_distance);
        // 搜索
        int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
             Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool init = false, bool dynamic = false,
             double time_start = -1.0);
        // 返回路径
        std::vector<Eigen::Vector3d> getPath();
        // 返回ros消息格式的路径
        nav_msgs::Path get_ros_path();
        // 返回访问过的节点
        std::vector<PathNodePtr> getVisitedNodes();



        typedef shared_ptr<Astar> Ptr;

};


}

#endif
