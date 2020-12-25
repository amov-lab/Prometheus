#include <kinodynamic_astar.h>
#include <sstream>

using namespace std;
using namespace Eigen;

namespace Global_Planning
{

KinodynamicAstar::~KinodynamicAstar()
{
  for (int i = 0; i < max_search_num; i++)
  {
    // delete表示释放堆内存
    delete path_node_pool_[i];
  }
}

void KinodynamicAstar::init(ros::NodeHandle& nh)
{
  // 2d参数
  nh.param("global_planner/is_2D", is_2D, 0);  // 1代表2D平面规划及搜索,0代表3D
  nh.param("global_planner/2D_fly_height", fly_height, 1.5);  // 2D规划时,定高高度
  // 地图参数
  nh.param("map/resolution", resolution_, 0.2);  // 地图分辨率

  // 规划搜索相关参数
  nh.param("kinodynamic_astar/lambda_heu", lambda_heu_, 2.0);  // 加速引导参数
  nh.param("kinodynamic_astar/allocate_num", max_search_num, 100000); //最大搜索节点数
  nh.param("kinodynamic_astar/max_tau", max_tau_, -1.0);
  nh.param("kinodynamic_astar/init_max_tau", init_max_tau_, -1.0);
  nh.param("kinodynamic_astar/max_vel", max_vel_, -1.0);
  nh.param("kinodynamic_astar/max_acc", max_acc_, -1.0);
  nh.param("kinodynamic_astar/w_time", w_time_, -1.0);
  nh.param("kinodynamic_astar/horizon", horizon_, -1.0);
  
  nh.param("kinodynamic_astar/time_resolution", time_resolution_, -1.0);
  nh.param("kinodynamic_astar/margin", margin_, -1.0);
  nh.param("kinodynamic_astar/check_num", check_num_, -1);
  

  tie_breaker_ = 1.0 + 1.0 / max_search_num;
  this->inv_resolution_ = 1.0 / resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;

  phi_ = Eigen::MatrixXd::Identity(6, 6);

  has_global_point = false;
  path_node_pool_.resize(max_search_num);

  for (int i = 0; i < max_search_num; i++)
  {
    path_node_pool_[i] = new PathNode;
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

void KinodynamicAstar::reset()
{
  // 重置与搜索相关的变量
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++)
  {
    PathNodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;
  
}

// 搜索函数，输入为：起始点及终点
// 将传输的数组通通变为指针！！！！ 以后改
int KinodynamicAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
                             Eigen::Vector3d end_pt, Eigen::Vector3d end_v, bool init, bool dynamic, double time_start)
{
  // 首先检查目标点是否可到达
  if(Occupy_map_ptr->getOccupancy(end_pt))
  {
    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Astar can't find path: goal point is occupied.");
    return NO_PATH;
  }

  // 计时
  ros::Time time_astar_start = ros::Time::now();

  start_vel_ = start_v;
  start_acc_ = start_a;

  goal_pos = end_pt;
  Eigen::VectorXd end_state(6);
  Eigen::Vector3i end_index = posToIndex(end_pt);
  //　
  double time_to_goal;
  end_state.head(3) = end_pt;
  end_state.tail(3) = end_v;

  // 初始化,将起始点设为第一个路径点
  PathNodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->state.head(3) = start_pt;
  cur_node->state.tail(3) = start_v;
  cur_node->index = posToIndex(start_pt);
  cur_node->g_score = 0.0;
  // 计算最优，并对time_to_goal赋值
  cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);
  cur_node->node_state = IN_OPEN_SET;

  // 将当前点推入open set
  open_set_.push(cur_node);
  // 迭代次数+1
  use_node_num_ += 1;

  if (dynamic)
  {
    time_origin_ = time_start;
    cur_node->time = time_start;
    cur_node->time_idx = timeToIndex(time_start);
    expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
  }
  else
  {
    expanded_nodes_.insert(cur_node->index, cur_node);
  }
  
  PathNodePtr neighbor = NULL;
  PathNodePtr terminate_node = NULL;
  bool init_search = init;
  const int tolerance = ceil(1 / resolution_);

  /* ---------- search loop ---------- */
  while (!open_set_.empty())
  {
    // 获取f_score最低的点
    cur_node = open_set_.top();
  
    //　判断终止条件
    bool reach_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                    abs(cur_node->index(1) - end_index(1)) <= tolerance &&
                    abs(cur_node->index(2) - end_index(2)) <= tolerance;
 
    bool reach_horizon = (cur_node->state.head(3) - start_pt).norm() >= horizon_;

    if (reach_horizon || reach_end)
    {
      // 将当前点设为终止点，并往回形成路径
      terminate_node = cur_node;
      retrievePath(terminate_node);
      has_path_ = true;

      // 时间一般很短，远远小于膨胀点云的时间
      printf("Astar take time %f s. \n", (ros::Time::now()-time_astar_start).toSec());

      if (reach_end)
      {
        /* one shot trajectory */
        estimateHeuristic(cur_node->state, end_state, time_to_goal);
        computeShotTraj(cur_node->state, end_state, time_to_goal);

        if (terminate_node->parent == NULL && !is_shot_succ_)
          return NO_PATH;
        else
          return REACH_END;
      }
      else if (reach_horizon)
      {
        return REACH_HORIZON;
      }
    }

    /* ---------- pop node and add to close set ---------- */
    open_set_.pop();
    // 将当前点推入close set
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;

    /* ---------- init state propagation ---------- */
    double res = 1 / 2.0, time_res = 1 / 1.0, time_res_init = 1 / 8.0;

    Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
    Eigen::Matrix<double, 6, 1> pro_state;
    vector<PathNodePtr> tmp_expand_nodes;
    Eigen::Vector3d um;
    double pro_t;

    vector<Eigen::Vector3d> inputs;
    vector<double> durations;

    if (init_search)
    {
      inputs.push_back(start_acc_);
      for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_; tau += time_res_init * init_max_tau_)
        durations.push_back(tau);
    }
    else
    {
      for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
        for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
          for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res)
          {
            um << ax, ay, 0.5 * az;
            inputs.push_back(um);
          }
      
      for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
        durations.push_back(tau);
    }

    /* ---------- state propagation loop ---------- */
    // cout << "cur state:" << cur_state.head(3).transpose() << endl;
    for (int i = 0; i < inputs.size(); ++i)
      for (int j = 0; j < durations.size(); ++j)
      {
        init_search = false;
        um = inputs[i];
        double tau = durations[j];
        stateTransit(cur_state, pro_state, um, tau);
        pro_t = cur_node->time + tau;

        /* ---------- check if in free space ---------- */

        /* inside map range */
        if (pro_state(0) <= origin_(0) || pro_state(0) >= map_size_3d_(0) || pro_state(1) <= origin_(1) ||
            pro_state(1) >= map_size_3d_(1) || pro_state(2) <= origin_(2) || pro_state(2) >= map_size_3d_(2))
        {
#ifdef DEBUG
              cout << "outside map" << endl;
#endif   
          continue;
        }

        /* not in close set */
        Eigen::Vector3i pro_id = posToIndex(pro_state.head(3));
        int pro_t_id = timeToIndex(pro_t);

        PathNodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);

        if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
        {
          // cout << "in closeset" << endl;
          continue;
        }

        /* vel feasibe */
        Eigen::Vector3d pro_v = pro_state.tail(3);
        if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_ || fabs(pro_v(2)) > max_vel_)
        {
          // cout << "vel infeasible" << endl;
          continue;
        }

        /* not in the same voxel */
        Eigen::Vector3i diff = pro_id - cur_node->index;
        int diff_time = pro_t_id - cur_node->time_idx;
        if (diff.norm() == 0 && ((!dynamic) || diff_time == 0))
        {
          continue;
        }

        /* collision free */
        Eigen::Vector3d pos;
        Eigen::Matrix<double, 6, 1> xt;
        bool is_occ = false;

        for (int k = 1; k <= check_num_; ++k)
        {
          double dt = tau * double(k) / double(check_num_);
          stateTransit(cur_state, xt, um, dt);
          pos = xt.head(3);

          is_occ = Occupy_map_ptr->getOccupancy(pos);
          if(is_occ){
            break;
          }
        }

        if (is_occ)
        {
#ifdef DEBUG
          printf("A star pos: [%f,  %f,  %f]\n", pos(0), pos(1), pos(2));
          cout << "collision" << endl;
#endif      
          continue;
        }

        /* ---------- compute cost ---------- */
        double time_to_goal, tmp_g_score, tmp_f_score;
        tmp_g_score = (um.squaredNorm() + w_time_) * tau + cur_node->g_score;
        tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);

        /* ---------- compare expanded node in this loop ---------- */

        bool prune = false;
        for (int j = 0; j < tmp_expand_nodes.size(); ++j)
        {
          PathNodePtr expand_node = tmp_expand_nodes[j];
          if ((pro_id - expand_node->index).norm() == 0 && ((!dynamic) || pro_t_id == expand_node->time_idx))
          {
            prune = true;
            if (tmp_f_score < expand_node->f_score)
            {
              expand_node->f_score = tmp_f_score;
              expand_node->g_score = tmp_g_score;
              expand_node->state = pro_state;
              expand_node->input = um;
              expand_node->duration = tau;
              if (dynamic)
                expand_node->time = cur_node->time + tau;
            }
            break;
          }
        }

        /* ---------- new neighbor in this loop ---------- */

        if (!prune)
        {
          if (pro_node == NULL)
          {
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->state = pro_state;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->input = um;
            pro_node->duration = tau;
            pro_node->parent = cur_node;
            pro_node->node_state = IN_OPEN_SET;
            if (dynamic)
            {
              pro_node->time = cur_node->time + tau;
              pro_node->time_idx = timeToIndex(pro_node->time);
            }
            open_set_.push(pro_node);

            if (dynamic)
              expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
            else
              expanded_nodes_.insert(pro_id, pro_node);

            tmp_expand_nodes.push_back(pro_node);

            use_node_num_ += 1;
            if (use_node_num_ == max_search_num)
            {
              cout << "run out of memory." << endl;
              return NO_PATH;
            }
          }
          else if (pro_node->node_state == IN_OPEN_SET)
          {
            if (tmp_g_score < pro_node->g_score)
            {
              // pro_node->index = pro_id;
              pro_node->state = pro_state;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->input = um;
              pro_node->duration = tau;
              pro_node->parent = cur_node;
              if (dynamic)
                pro_node->time = cur_node->time + tau;
            }
          }
          else
          {
            cout << "error type in searching: " << pro_node->node_state << endl;
          }
        }

        /* ----------  ---------- */
      }
  }

  // 搜索完所有可行点，即使没达到最大搜索次数，也没有找到路径
  // 这种一般是因为无人机周围被占据，或者无人机与目标点之间无可通行路径造成的
  pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "max_search_num: open set empty.");
  return NO_PATH;
}

// 由最终点往回生成路径
void KinodynamicAstar::retrievePath(PathNodePtr end_node)
{
  PathNodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  while (cur_node->parent != NULL)
  {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}



double KinodynamicAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time)
{
  const Vector3d dp = x2.head(3) - x1.head(3);
  const Vector3d v0 = x1.segment(3, 3);
  const Vector3d v1 = x2.segment(3, 3);

  double c1 = -36 * dp.dot(dp);
  double c2 = 24 * (v0 + v1).dot(dp);
  double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
  double c4 = 0;
  double c5 = w_time_;

  std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

  double v_max = max_vel_;
  double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;
  ts.push_back(t_bar);

  double cost = 100000000;
  double t_d = t_bar;

  for (auto t : ts)
  {
    if (t < t_bar)
      continue;
    double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
    if (c < cost)
    {
      cost = c;
      t_d = t;
    }
  }

  optimal_time = t_d;

  return 1.0 * (1 + tie_breaker_) * cost;
}

bool KinodynamicAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal)
{
  /* ---------- get coefficient ---------- */
  const Vector3d p0 = state1.head(3);
  const Vector3d dp = state2.head(3) - p0;
  const Vector3d v0 = state1.segment(3, 3);
  const Vector3d v1 = state2.segment(3, 3);
  const Vector3d dv = v1 - v0;
  double t_d = time_to_goal;
  MatrixXd coef(3, 4);
  end_vel_ = v1;

  Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
  Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
  Vector3d c = v0;
  Vector3d d = p0;

  // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
  // a*t^3 + b*t^2 + v0*t + p0
  coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

  Vector3d coord, vel, acc;
  VectorXd poly1d, t, polyv, polya;
  Vector3i index;

  Eigen::MatrixXd Tm(4, 4);
  Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

  /* ---------- forward checking of trajectory ---------- */
  double t_delta = t_d / 10;
  for (double time = t_delta; time <= t_d; time += t_delta)
  {
    t = VectorXd::Zero(4);
    for (int j = 0; j < 4; j++)
      t(j) = pow(time, j);

    for (int dim = 0; dim < 3; dim++)
    {
      poly1d = coef.row(dim);
      coord(dim) = poly1d.dot(t);
      vel(dim) = (Tm * poly1d).dot(t);
      acc(dim) = (Tm * Tm * poly1d).dot(t);

      if (fabs(vel(dim)) > max_vel_ || fabs(acc(dim)) > max_acc_)
      {
        // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
        // return false;
      }
    }

    if (coord(0) < origin_(0) || coord(0) >= map_size_3d_(0) || coord(1) < origin_(1) || coord(1) >= map_size_3d_(1) ||
        coord(2) < origin_(2) || coord(2) >= map_size_3d_(2))
    {
      return false;
    }
  }
  coef_shot_ = coef;
  t_shot_ = t_d;
  is_shot_succ_ = true;
  return true;
}

vector<double> KinodynamicAstar::cubic(double a, double b, double c, double d)
{
  vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0)
  {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  }
  else if (D == 0)
  {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  }
  else
  {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

vector<double> KinodynamicAstar::quartic(double a, double b, double c, double d, double e)
{
  vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0)
    return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0)
  {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  }
  else
  {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D))
  {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E))
  {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}


bool KinodynamicAstar::check_safety(Eigen::Vector3d &cur_pos, double safe_distance)
{
  bool is_safety;
  is_safety = Occupy_map_ptr->check_safety(cur_pos, safe_distance);
  return is_safety;
}

std::vector<Eigen::Vector3d> KinodynamicAstar::getKinoTraj(double delta_t)
{
  vector<Vector3d> state_list;

  /* ---------- get traj of searching ---------- */
  PathNodePtr node = path_nodes_.back();
  Matrix<double, 6, 1> x0, xt;

  while (node->parent != NULL)
  {
    Vector3d ut = node->input;
    double duration = node->duration;
    x0 = node->parent->state;

    for (double t = duration; t >= -1e-5; t -= delta_t)
    {
      stateTransit(x0, xt, ut, t);
      state_list.push_back(xt.head(3));
    }
    node = node->parent;
  }
  reverse(state_list.begin(), state_list.end());

  /* ---------- get traj of one shot ---------- */
  if (is_shot_succ_)
  {
    Vector3d coord;
    VectorXd poly1d, time(4);

    for (double t = delta_t; t <= t_shot_; t += delta_t)
    {
      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 3; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      state_list.push_back(coord);
    }
  }

  return state_list;
}


nav_msgs::Path KinodynamicAstar::get_ros_path()
{

  std::vector<Eigen::Vector3d> path = getKinoTraj(0.1);

  nav_msgs::Path A_star_path_cmd;
  geometry_msgs::PoseStamped path_i_pose;

  A_star_path_cmd.header.frame_id = "world";
  A_star_path_cmd.header.stamp = ros::Time::now();
  A_star_path_cmd.poses.clear();
  for (int i=0; i<path.size(); ++i)
  {
      path_i_pose .header.frame_id = "world";
      path_i_pose.pose.position.x = path[i](0);
      path_i_pose.pose.position.y = path[i](1);
      path_i_pose.pose.position.z = path[i](2);
      A_star_path_cmd.poses.push_back(path_i_pose);
  }

  path_i_pose .header.frame_id = "world";
  path_i_pose.pose.position.x = goal_pos[0];
  path_i_pose.pose.position.y = goal_pos[1];
  path_i_pose.pose.position.z = goal_pos[2];
  A_star_path_cmd.poses.push_back(path_i_pose);

  return A_star_path_cmd;
}

std::vector<PathNodePtr> KinodynamicAstar::getVisitedNodes()
{
  vector<PathNodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

Eigen::Vector3i KinodynamicAstar::posToIndex(Eigen::Vector3d pt)
{
  Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();

  // idx << floor((pt(0) - origin_(0)) * inv_resolution_), floor((pt(1) - origin_(1)) * inv_resolution_),
  //     floor((pt(2) - origin_(2)) * inv_resolution_);

  return idx;
}

int KinodynamicAstar::timeToIndex(double time)
{
  int idx = floor((time - time_origin_) * inv_time_resolution_);
}

void KinodynamicAstar::stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1,
                                    Eigen::Vector3d um, double tau)
{
  for (int i = 0; i < 3; ++i)
    phi_(i, i + 3) = tau;

  Eigen::Matrix<double, 6, 1> integral;
  integral.head(3) = 0.5 * pow(tau, 2) * um;
  integral.tail(3) = tau * um;

  state1 = phi_ * state0 + integral;
}

}  // namespace global_planner
