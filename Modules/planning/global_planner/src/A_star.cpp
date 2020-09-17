// #include "A_star.h"
#include "A_star.h"
#include <sstream>

using namespace std;
using namespace Eigen;


namespace global_planner{

Astar::~Astar()
{
  for (int i = 0; i < allocate_num_; i++)
  {
    delete path_node_pool_[i];
  }
}


void Astar::setParam(ros::NodeHandle& nh)
{
  nh.param("astar/resolution_astar", resolution_, 0.2);  // 地图分辨率
  nh.param("astar/lambda_heu", lambda_heu_, 2.0);  // 加速引导
  nh.param("astar/allocate_num", allocate_num_, 100000); //最大节点数
  nh.param("astar/is_2D", is_2D, 0);  // 1代表2D平面规划及搜索,0代表3D
  nh.param("astar/2D_fly_height", fly_height, 1.5);  // 2D规划时,定高高度
  
  tie_breaker_ = 1.0 + 1.0 / allocate_num_;

}

void Astar::retrievePath(NodePtr end_node)
{
  NodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  while (cur_node->parent != NULL)
  {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}

std::vector<Eigen::Vector3d> Astar::getPath()
{
  vector<Eigen::Vector3d> path;
  for (int i = 0; i < path_nodes_.size(); ++i)
  {
    path.push_back(path_nodes_[i]->position);
  }
  path.push_back(goal_pos);
  return path;
}



int Astar::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{
  /* ---------- initialize ---------- */
  // printf("[A starr search]-------- \n");
  NodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->position = start_pt;

  cur_node->index = posToIndex(start_pt);
  cur_node->g_score = 0.0;

  ros::Time time_astar_start = ros::Time::now();

  Eigen::Vector3i end_index;
  double time_to_goal;
  if(Occupy_map_ptr->getOccupancy(end_pt)){
      pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "end point is occupied, pls reset the goal");
      return NO_PATH;
  }
  goal_pos = end_pt;
  end_index = posToIndex(end_pt);
  cur_node->f_score = lambda_heu_ * getEuclHeu(cur_node->position, end_pt);
  cur_node->node_state = IN_OPEN_SET;
   
  open_set_.push(cur_node);
  use_node_num_ += 1;


  expanded_nodes_.insert(cur_node->index, cur_node);

  NodePtr neighbor = NULL;
  NodePtr terminate_node = NULL;

  /* ---------- search loop ---------- */
  while (!open_set_.empty())
  {
    /* ---------- get lowest f_score node ---------- */
    cur_node = open_set_.top();
    // printf("cur pos ind: [%d, %d, %d], end pos index: [%d, %d, %d]\n", cur_node->index(0), cur_node->index(1), cur_node->index(2),
    //                                                                                                            end_index(0), end_index(1), end_index(2) );
    
    /* ---------- determine termination ---------- */
    bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 && abs(cur_node->index(1) - end_index(1)) <= 1 &&
                     abs(cur_node->index(2) - end_index(2)) <= 1;

    if (reach_end)
    {
      terminate_node = cur_node;
      retrievePath(terminate_node);
      has_path_ = true;
      
      // printf("a star take time  %f \n", (ros::Time::now()-time_astar_start).toSec());
      return REACH_END;
    }

    /* ---------- pop node and add to close set ---------- */
    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;  // in expand set
    iter_num_ += 1;

    /* ---------- init neighbor expansion ---------- */

    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d pro_pos;
    // double pro_t;

    vector<Eigen::Vector3d> inputs;
    Eigen::Vector3d d_pos;

    /* ---------- expansion loop ---------- */
    for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
    {
      for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
      {
        for (double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_)
        {
            
          d_pos << dx, dy, dz;

          if (is_2D == 1)
          {
            d_pos(2) = 0.0;
          }

          if (d_pos.norm() < 1e-3)
          {
            continue;
          }
              
          pro_pos = cur_pos + d_pos;

          /* ---------- check if in feasible space ---------- */
          if (pro_pos(0) <= origin_(0) || pro_pos(0) >= map_size_3d_(0) || pro_pos(1) <= origin_(1) ||
              pro_pos(1) >= map_size_3d_(1) || pro_pos(2) <= origin_(2) || pro_pos(2) >= map_size_3d_(2))
          {
            pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "outside map.");
            continue;
          }

            Eigen::Vector3i d_pos_id;
            d_pos_id << int(dx/resolution_), int(dy/resolution_), int(dz/resolution_);
            Eigen::Vector3i pro_id = d_pos_id + cur_node->index;

            /* not in close set */
            Eigen::Vector3i last_pro_id = posToIndex(cur_pos);

            NodePtr pro_node = expanded_nodes_.find(pro_id);

            if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
            {
                continue;
            }

          /* collision free */

            bool is_occupy = Occupy_map_ptr->getOccupancy(pro_pos);

            if (is_occupy)
            {
                //  if(pro_id(2)==8) cout << "in occupy" << endl;
                    
                continue;
            }

            /* ---------- compute cost ---------- */
            double time_to_goal, tmp_g_score, tmp_f_score;
            tmp_g_score = d_pos.squaredNorm() + cur_node->g_score;
            tmp_f_score = tmp_g_score + lambda_heu_ * getEuclHeu(pro_pos, end_pt);

            if (pro_node == NULL)
            {
                pro_node = path_node_pool_[use_node_num_];
                pro_node->index = pro_id;
                pro_node->position = pro_pos;
                pro_node->f_score = tmp_f_score;
                pro_node->g_score = tmp_g_score;
                pro_node->parent = cur_node;
                pro_node->node_state = IN_OPEN_SET;

                open_set_.push(pro_node);
    
                expanded_nodes_.insert(pro_id, pro_node);

                use_node_num_ += 1;
                if (use_node_num_ == allocate_num_)
                {
                    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "a star run out of memory.\n");
                    return NO_PATH;
                }
            }
            else if (pro_node->node_state == IN_OPEN_SET)
            {
                if (tmp_g_score < pro_node->g_score)
                {
                    // pro_node->index = pro_id;
                    pro_node->position = pro_pos;
                    pro_node->f_score = tmp_f_score;
                    pro_node->g_score = tmp_g_score;
                    pro_node->parent = cur_node;

                }
                else
                {
                    // cout << "error type in searching: " << pro_node->node_state << endl;
                    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "error type in searching.\n");
                }
              }
          }
      }
    }       
  
  
  
  
  }

  pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "open set empty, no path!");
  return NO_PATH;

}





double Astar::getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2)
{
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));

  double h;
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

void Astar::init(ros::NodeHandle& nh)
{
  /* ---------- map params ---------- */
  this->inv_resolution_ = 1.0 / resolution_;
  has_global_point = false;
//   /* ---------- pre-allocated node ---------- */
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++)
  {
    path_node_pool_[i] = new Node;
  }

  use_node_num_ = 0;
  iter_num_ = 0;

  Occupy_map_ptr.reset(new Occupy_map);
  Occupy_map_ptr->setparam(nh);
  Occupy_map_ptr->init();

}

void Astar::reset()
{
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


std::vector<NodePtr> Astar::getVisitedNodes()
{
  vector<NodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

Eigen::Vector3i Astar::posToIndex(Eigen::Vector3d pt)
{
//   Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
  Vector3i idx ;
  idx << floor((pt(0) - origin_(0)) * inv_resolution_), floor((pt(1) - origin_(1)) * inv_resolution_),
      floor((pt(2) - origin_(2)) * inv_resolution_);

  return idx;
}

void Astar::indexToPos(Eigen::Vector3i id, Eigen::Vector3d &pos) {
    for (int i = 0; i < 3; ++i)
        pos(i) = (id(i) + 0.5) * resolution_ + origin_(i);
}


void Astar::setEnvironment(const sensor_msgs::PointCloud2ConstPtr & global_point){
    Occupy_map_ptr->setEnvironment(global_point);
    // 对地图进行膨胀
    Occupy_map_ptr->inflate_point_cloud();
    origin_ =  Occupy_map_ptr->origin_;
    map_size_3d_ = Occupy_map_ptr->map_size_3d_;
    // printf("map origin: [%f, %f, %f], map size: [%f, %f, %f]\n", origin_(0), origin_(1),origin_(2), 
    //                                                                             map_size_3d_(0), map_size_3d_(1), map_size_3d_(2));

}

bool Astar::check_safety(Eigen::Vector3d &cur_pos, double safe_distance){
  bool is_safety;
  is_safety = Occupy_map_ptr->check_safety(cur_pos, safe_distance);
  return is_safety;
}


}