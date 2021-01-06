#include "A_star.h"

using namespace std;
using namespace Eigen;

namespace Global_Planning
{

Astar::~Astar()
{
  for (int i = 0; i < max_search_num; i++)
  {
    // delete表示释放堆内存
    delete path_node_pool_[i];
  }
}

void Astar::init(ros::NodeHandle& nh)
{
  // 2d参数
  nh.param("global_planner/is_2D", is_2D, 0);  // 1代表2D平面规划及搜索,0代表3D
  nh.param("global_planner/2D_fly_height", fly_height, 1.5);  // 2D规划时,定高高度
  // 规划搜索相关参数
  nh.param("astar/lambda_heu", lambda_heu_, 2.0);  // 加速引导参数
  nh.param("astar/allocate_num", max_search_num, 100000); //最大搜索节点数
  // 地图参数
  nh.param("map/resolution", resolution_, 0.2);  // 地图分辨率

  tie_breaker_ = 1.0 + 1.0 / max_search_num;

  this->inv_resolution_ = 1.0 / resolution_;

  has_global_point = false;
  path_node_pool_.resize(max_search_num);

  // 新建
  for (int i = 0; i < max_search_num; i++)
  {
    path_node_pool_[i] = new Node;
  }

  use_node_num_ = 0;
  iter_num_ = 0;

  // 初始化占据地图
  Occupy_map_ptr.reset(new Occupy_map);
  Occupy_map_ptr->init(nh);

  // 读取地图参数
  origin_ =  Occupy_map_ptr->min_range_;
  map_size_3d_ = Occupy_map_ptr->max_range_ - Occupy_map_ptr->min_range_;
}

void Astar::reset()
{
  // 重置与搜索相关的变量
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++)
  {
    NodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
}

// 搜索函数，输入为：起始点及终点
// 将传输的数组通通变为指针！！！！ 以后改
int Astar::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{
  // 首先检查目标点是否可到达
  if(Occupy_map_ptr->getOccupancy(end_pt))
  {
    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Astar can't find path: goal point is occupied.");
    return NO_PATH;
  }

  // 计时
  ros::Time time_astar_start = ros::Time::now();
  
  goal_pos = end_pt;
  Eigen::Vector3i end_index = posToIndex(end_pt);

  // 初始化,将起始点设为第一个路径点
  NodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->position = start_pt;
  cur_node->index = posToIndex(start_pt);
  cur_node->g_score = 0.0;
  cur_node->f_score = lambda_heu_ * getEuclHeu(cur_node->position, end_pt);
  cur_node->node_state = IN_OPEN_SET;
   
  // 将当前点推入open set
  open_set_.push(cur_node);
  // 迭代次数+1
  use_node_num_ += 1;
  // 记录当前为已扩展
  expanded_nodes_.insert(cur_node->index, cur_node);

  NodePtr terminate_node = NULL;

  // 搜索主循环
  while (!open_set_.empty())
  {
    // 获取f_score最低的点
    cur_node = open_set_.top();

    // 判断终止条件
    bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 && 
                     abs(cur_node->index(1) - end_index(1)) <= 1 &&
                     abs(cur_node->index(2) - end_index(2)) <= 1;

    if (reach_end)
    {
      // 将当前点设为终止点，并往回形成路径
      terminate_node = cur_node;
      retrievePath(terminate_node);

      // 时间一般很短，远远小于膨胀点云的时间
      printf("Astar take time %f s. \n", (ros::Time::now()-time_astar_start).toSec());

      return REACH_END;
    }

    /* ---------- pop node and add to close set ---------- */
    open_set_.pop();
    // 将当前点推入close set
    cur_node->node_state = IN_CLOSE_SET;  // in expand set
    iter_num_ += 1;

    /* ---------- init neighbor expansion ---------- */
    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d expand_node_pos;

    vector<Eigen::Vector3d> inputs;
    Eigen::Vector3d d_pos;

    /* ---------- expansion loop ---------- */
    // 扩展： 3*3*3 - 1 = 26种可能
    for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
    {
      for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
      {
        for (double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_)
        {
            
          d_pos << dx, dy, dz;
          // 对于2d情况，不扩展z轴
          if (is_2D == 1)
          {
            d_pos(2) = 0.0;
          }

          // 跳过自己那个格子
          if (d_pos.norm() < 1e-3)
          {
            continue;
          }
          
          // 扩展节点的位置
          expand_node_pos = cur_pos + d_pos;

          // 确认该点在地图范围内
          if(!Occupy_map_ptr->isInMap(expand_node_pos))
          {
            continue;
          }

          // 计算扩展节点的index
          Eigen::Vector3i d_pos_id;
          d_pos_id << int(dx/resolution_), int(dy/resolution_), int(dz/resolution_);
          Eigen::Vector3i expand_node_id = d_pos_id + cur_node->index;

          //检查当前扩展的点是否在close set中，如果是则跳过
          NodePtr expand_node = expanded_nodes_.find(expand_node_id);
          if (expand_node != NULL && expand_node->node_state == IN_CLOSE_SET)
          {
            continue;
          }

          // 检查当前扩展点是否被占据,如果是则跳过
          bool is_occupy = Occupy_map_ptr->getOccupancy(expand_node_pos);
          if (is_occupy)
          {                  
            continue;
          }

          // 如果能通过上述检查则
          double tmp_g_score, tmp_f_score;
          tmp_g_score = d_pos.squaredNorm() + cur_node->g_score;
          tmp_f_score = tmp_g_score + lambda_heu_ * getEuclHeu(expand_node_pos, end_pt);

          // 如果扩展的当前节点为NULL，即未扩展过
          if (expand_node == NULL)
          {
            expand_node = path_node_pool_[use_node_num_];
            expand_node->index = expand_node_id;
            expand_node->position = expand_node_pos;
            expand_node->f_score = tmp_f_score;
            expand_node->g_score = tmp_g_score;
            expand_node->parent = cur_node;
            expand_node->node_state = IN_OPEN_SET;

            open_set_.push(expand_node);
            expanded_nodes_.insert(expand_node_id, expand_node);

            use_node_num_ += 1;
            // 超过最大搜索次数
            if (use_node_num_ == max_search_num)
            {
                pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Astar can't find path: reach the max_search_num.\n");
                return NO_PATH;
            }
          }
          // 如果当前节点已被扩展过，则更新其状态
          else if (expand_node->node_state == IN_OPEN_SET)
          {
            if (tmp_g_score < expand_node->g_score)
            {
                // expand_node->index = expand_node_id;
                expand_node->position = expand_node_pos;
                expand_node->f_score = tmp_f_score;
                expand_node->g_score = tmp_g_score;
                expand_node->parent = cur_node;
            }
          }
        }
      }
    }       
  
  }

  // 搜索完所有可行点，即使没达到最大搜索次数，也没有找到路径
  // 这种一般是因为无人机周围被占据，或者无人机与目标点之间无可通行路径造成的
  pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "max_search_num: open set empty.");
  return NO_PATH;
}

// 由最终点往回生成路径
void Astar::retrievePath(NodePtr end_node)
{
  NodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  while (cur_node->parent != NULL)
  {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  // 反转顺序
  reverse(path_nodes_.begin(), path_nodes_.end());


  // 直接在这里生成路径？
}

std::vector<Eigen::Vector3d> Astar::getPath()
{
  vector<Eigen::Vector3d> path;
  for (uint i = 0; i < path_nodes_.size(); ++i)
  {
    path.push_back(path_nodes_[i]->position);
  }
  path.push_back(goal_pos);
  return path;
}

nav_msgs::Path Astar::get_ros_path()
{
  nav_msgs::Path path;

  path.header.frame_id = "world";
  path.header.stamp = ros::Time::now();
  path.poses.clear();

  geometry_msgs::PoseStamped path_i_pose;
  for (uint i=0; i<path_nodes_.size(); ++i)
  {   
    path_i_pose .header.frame_id = "world";
    path_i_pose.pose.position.x = path_nodes_[i]->position[0];
    path_i_pose.pose.position.y = path_nodes_[i]->position[1];
    path_i_pose.pose.position.z = path_nodes_[i]->position[2];
    path.poses.push_back(path_i_pose);
  }

  path_i_pose .header.frame_id = "world";
  path_i_pose.pose.position.x = goal_pos[0];
  path_i_pose.pose.position.y = goal_pos[1];
  path_i_pose.pose.position.z = goal_pos[2];
  path.poses.push_back(path_i_pose);

  return path;
}

double Astar::getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2)
{
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));

  double h = 0;

  int diag = min(min(dx, dy), dz);
  dx -= diag;
  dy -= diag;
  dz -= diag;

  if (dx < 1e-4)
  {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
  }
  if (dy < 1e-4)
  {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
  }
  if (dz < 1e-4)
  {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
  }

  return tie_breaker_ * h;
}


double Astar::getManhHeu(Eigen::Vector3d x1, Eigen::Vector3d x2)
{
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));

  return tie_breaker_ * (dx + dy + dz);
}

double Astar::getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2)
{
  return tie_breaker_ * (x2 - x1).norm();
}

std::vector<NodePtr> Astar::getVisitedNodes()
{
  vector<NodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

Eigen::Vector3i Astar::posToIndex(Eigen::Vector3d pt)
{
  Vector3i idx ;
  idx << floor((pt(0) - origin_(0)) * inv_resolution_), floor((pt(1) - origin_(1)) * inv_resolution_),
      floor((pt(2) - origin_(2)) * inv_resolution_);

  return idx;
}

void Astar::indexToPos(Eigen::Vector3i id, Eigen::Vector3d &pos) 
{
  for (int i = 0; i < 3; ++i)
      pos(i) = (id(i) + 0.5) * resolution_ + origin_(i);
}

// 检查cur_pos是否安全
bool Astar::check_safety(Eigen::Vector3d &cur_pos, double safe_distance)
{
  bool is_safety;
  is_safety = Occupy_map_ptr->check_safety(cur_pos, safe_distance);
  return is_safety;
}


}