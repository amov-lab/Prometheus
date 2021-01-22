#include <plan_env/edt_environment.h>

namespace dyn_planner
{
/* ============================== edt_environment ============================== */
void EDTEnvironment::init()
{
}

void EDTEnvironment::setMap(shared_ptr<SDFMap> map)
{
  EDTEnv_mode = LOCAL_MODE;
  this->sdf_map_ = map;
  resolution_inv_ = 1 / sdf_map_->getResolution();
  ROS_INFO("--- EDTEnvironment: set local sdf_map finished! ---");
}

void EDTEnvironment::setMap(shared_ptr<SDFMap_Global> map){

    EDTEnv_mode = GLOBAL_MODE;
    this->sdf_map_global = map;
    resolution_inv_ = 1 / sdf_map_global->getResolution();
    ROS_INFO("--- EDTEnvironment: set global sdf_map finished! ---");
}

void EDTEnvironment::evaluateEDTWithGrad(const Eigen::Vector3d& pos, const double& time, double& dist,
                                         Eigen::Vector3d& grad)
{
  if(EDTEnv_mode == LOCAL_MODE){

      dist = sdf_map_->getDistWithGradTrilinear(pos, grad);

  } else if(EDTEnv_mode == GLOBAL_MODE){
      sdf_map_global->evaluateEDTWithGrad(pos, dist, grad);

  } else{
      printf("mode error!\n");
      return;
  }
  }


double EDTEnvironment::evaluateCoarseEDT(const Eigen::Vector3d& pos, const double& time)
{
  if(EDTEnv_mode == LOCAL_MODE){
      double d1 = sdf_map_->getDistance(pos);
      return d1;
  } else if(EDTEnv_mode == GLOBAL_MODE){
      double d1 = sdf_map_global->getDistance(pos);
      return d1;
  } else{
      printf("mode error!!\n");
      return -1.0;
  }

}

// jiangtao add
//bool EDTEnvironment::get_push_force(const Eigen::Matrix<double, 3, 1> current_odom, const double distance,
//        Eigen::Matrix<double, 3, 1> &push_force){
//
////    sdf_map_
////    vector<Eigen::Vector3d> pos_vec;
////    Eigen::Vector3d diff;
//    sdf_map_->get_arround_force(current_odom, distance, push_force);
//    return true;
//
//}

// EDTEnvironment::
}  // namespace dyn_planner
