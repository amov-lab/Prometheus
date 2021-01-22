#include <plan_manage/dyn_planner_manager.h>
#include <fstream>
#define DEBUG 1
namespace dyn_planner
{
DynPlannerManager::~DynPlannerManager()
{
  traj_id_ = 0;
}

void DynPlannerManager::setParam(ros::NodeHandle& nh)
{
  nh.param("manager/time_sample", time_sample_, -1.0);
  nh.param("manager/max_vel", max_vel_, -1.0);
  nh.param("manager/dynamic", dynamic_, -1);
  nh.param("manager/margin", margin_, -1.0);
}

void DynPlannerManager::setPathFinder0(const Astar::Ptr& finder)
{
  path_finder0_ = finder;// useless
}

void DynPlannerManager::setPathFinder(const KinodynamicAstar::Ptr& finder)
{
  path_finder_ = finder;
}

void DynPlannerManager::setOptimizer(const BsplineOptimizer::Ptr& optimizer)
{
  bspline_optimizer_ = optimizer;
}

void DynPlannerManager::setEnvironment(const EDTEnvironment::Ptr& env)
{
  edt_env_ = env;
}

bool DynPlannerManager::checkTrajCollision()
{
  /* check collision */
  for (double t = t_start_; t <= t_end_; t += 0.02)
  {
    Eigen::Vector3d pos = traj_pos_.evaluateDeBoor(t);
    double dist = dynamic_ ? edt_env_->evaluateCoarseEDT(pos, time_start_ + t - t_start_) :
                             edt_env_->evaluateCoarseEDT(pos, -1.0); // time useless

    if (dist < margin_)
    {
      return false;
    }
  }

  return true;
}

void DynPlannerManager::retrieveTrajectory()
{
  // 轨迹的速度和加速度由微分得到？
  traj_vel_ = traj_pos_.getDerivative();
  traj_acc_ = traj_vel_.getDerivative();

  traj_pos_.getTimeSpan(t_start_, t_end_);
  pos_traj_start_ = traj_pos_.evaluateDeBoor(t_start_);
  traj_duration_ = t_end_ - t_start_;

  traj_id_ += 1;
}

void DynPlannerManager::getSolvingTime(double& ts, double& to, double& ta)
{
  ts = time_search_;
  to = time_optimize_;
  ta = time_adjust_;
}

bool DynPlannerManager::generateTrajectory(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                           Eigen::Vector3d start_acc, Eigen::Vector3d end_pt, Eigen::Vector3d end_vel)
{
#ifdef DEBUG
  std::cout << "==================================\n" << "[planner]: Searching Starts Now!" << std::endl;
    cout << "start: \t pt: " << start_pt.transpose() << "\n\t vel: " << start_vel.transpose() << "\n\t acc: " << start_acc.transpose()
       << "\ngoal: \t pt: " << end_pt.transpose() << "\n\t vel: " << end_vel.transpose() << endl;
#endif

  if ((start_pt - end_pt).norm() < 0.2)
  {
#ifdef DEBUG
cout << "Close goal" << endl;
#endif
    return false;
  }

  //轨迹生成开始时间
  time_start_ = -1.0;

  double t_search = 0.0, t_sample = 0.0, t_axb = 0.0, t_opt = 0.0, t_adjust = 0.0;

  Eigen::Vector3d init_pos = start_pt;
  Eigen::Vector3d init_vel = start_vel;
  Eigen::Vector3d init_acc = start_acc;

  ros::Time start_time, end_time;
  start_time = ros::Time::now();


  /* ---------- search kino path ---------- */
  // 全局规划算法清零
  path_finder_->reset();

  // 全局规划算法搜索可行路径
  int status = path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true, dynamic_, time_start_);
  if (status == KinodynamicAstar::NO_PATH)
  {
#ifdef DEBUG
    cout << "[planner]: init search fail!" << endl;
#endif

    path_finder_->reset();
    status = path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false, dynamic_, time_start_);
    if (status == KinodynamicAstar::NO_PATH)
    {
#ifdef DEBUG
      cout << "[planner]: Can't find path." << endl;
#endif
      return false;
    }
    else
    {
#ifdef DEBUG
      cout << "[planner]: retry search success." << endl;
#endif     
      
    }
  }
  else
  {
#ifdef DEBUG
    cout << "[planner]: init search success." << endl;
#endif     
  }

  end_time = ros::Time::now();
  t_search = (end_time - start_time).toSec(); // 全局搜索时间


  /* ---------- bspline parameterization ---------- */
  start_time = ros::Time::now();

  int K;
  double ts = time_sample_ / max_vel_;
  Eigen::MatrixXd vel_acc;

  // 选取全局路径样本点
  Eigen::MatrixXd samples = path_finder_->getSamples(ts, K);

  // cout << "[planner]: ts: " <<  ts << endl << " sample:\n" << samples.transpose() << endl;

  end_time = ros::Time::now();
  t_sample = (end_time - start_time).toSec();// 样本点采样时间


  // 拟合全局路径样本点
  start_time = ros::Time::now();
  Eigen::MatrixXd control_pts;

  NonUniformBspline::getControlPointEqu3(samples, ts, control_pts);

  // cout << "ctrl pts:" << control_pts << endl;

  NonUniformBspline init = NonUniformBspline(control_pts, 3, ts);// 优化前拟合路径

  end_time = ros::Time::now();
  t_axb = (end_time - start_time).toSec();// 样本点拟合时间


  /* ---------- optimize trajectory ---------- */
  start_time = ros::Time::now();
  bspline_optimizer_->setControlPoints(control_pts);// 设置控制点
  bspline_optimizer_->setBSplineInterval(ts);// 设置时间间隔

  if (status != KinodynamicAstar::REACH_END){
    // cout << "Kinodynamic Astar not reach end!" <<endl;
    bspline_optimizer_->optimize(BsplineOptimizer::SOFT_CONSTRAINT, dynamic_, time_start_); // time useless
  }
  else{
    // cout << "Kinodynamic Astar reach end!" <<endl;
    bspline_optimizer_->optimize(BsplineOptimizer::HARD_CONSTRAINT, dynamic_, time_start_);
  }
  control_pts = bspline_optimizer_->getControlPoints();

  // cout << "optimal ctrl pts:" << control_pts << endl;

  end_time = ros::Time::now();
  t_opt = (end_time - start_time).toSec();// 轨迹优化时间


  /* ---------- time adjustment ---------- */
  start_time = ros::Time::now();
  NonUniformBspline pos = NonUniformBspline(control_pts, 3, ts);// 优化后路径

  double tm, tmp, to, tn;
  pos.getTimeSpan(tm, tmp);
  to = tmp - tm;

  bool feasible = pos.checkFeasibility(false, ts);

  int iter_num = 0;
  while (!feasible && ros::ok())
  {
    ++iter_num;

    feasible = pos.reallocateTime(false, ts);
    /* actually this not needed, converges within 10 iteration */
    if (iter_num >= 50)
      break;
  }

  // cout << "[Main]: iter num: " << iter_num << endl;
  pos.getTimeSpan(tm, tmp);
  tn = tmp - tm;

  // cout << "[planner]: Reallocate ratio: " << tn / to << endl;

  end_time = ros::Time::now();
  t_adjust = (end_time - start_time).toSec();// 运动动力学可行性修正时间

  pos.checkFeasibility(true, ts);
  // drawVelAndAccPro(pos);


  /* ---------- save result ---------- */
  traj_pos_ = pos;

  double t_total = t_search + t_sample + t_axb + t_opt + t_adjust;// 总消耗时间

  cout << "[planner]: time: " << t_total << ", search: " << t_search << ", optimize: " << t_sample + t_axb + t_opt
       << ", adjust time:" << t_adjust << endl;

  time_search_ = t_search;
  time_optimize_ = t_sample + t_axb + t_opt;
  time_adjust_ = t_adjust;


  time_traj_start_ = ros::Time::now();
  // time_start_ = -1.0;

  return true;
}

}  // namespace dyn_planner
