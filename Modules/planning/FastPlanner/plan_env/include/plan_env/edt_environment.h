#ifndef _EDT_ENVIRONMENT_H_
#define _EDT_ENVIRONMENT_H_

#include <Eigen/Eigen>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <plan_env/sdf_map.h>
// sdf_tools
#include "sdf_tools/collision_map.hpp"
#include "sdf_tools/sdf.hpp"
#include <plan_env/global_point_sdf.h>



using std::cout;
using std::endl;
using std::list;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace dyn_planner
{
class EDTEnvironment
{
   enum SDF_MODE{
     LOCAL_MODE=0, 
     GLOBAL_MODE
   };
private:
  /* data */
  SDFMap::Ptr sdf_map_;

  double resolution_inv_;

//    0 for the original fast_planner; 1 is our mode (receive the global point cloud, use sdf_tool)
    int EDTEnv_mode{0};
    SDFMap_Global::Ptr sdf_map_global;

public:
  EDTEnvironment(/* args */) {}
  ~EDTEnvironment() {}

  void init();
  void setMap(SDFMap::Ptr map);

  void setMap(SDFMap_Global::Ptr map);

  void evaluateEDTWithGrad(const Eigen::Vector3d& pos, const double& time, double& dist,
                           Eigen::Vector3d& grad);

  double evaluateCoarseEDT(const Eigen::Vector3d& pos, const double& time);

  bool odomValid() {
    if(EDTEnv_mode == LOCAL_MODE){
      return sdf_map_->odomValid(); 
    } else if(EDTEnv_mode == GLOBAL_MODE){
      return sdf_map_global->odomValid();
    } else{
      printf("mode error!!\n");
      return false;
    }  
  }

  bool mapValid() { 
    // return sdf_map_->mapValid(); 
    if(EDTEnv_mode==LOCAL_MODE){
      return sdf_map_->mapValid();
    }else if(EDTEnv_mode == GLOBAL_MODE){
      return sdf_map_global->mapValid();
    }else{
      printf("mode error!!\n");
      return false;
    }  
  }

  nav_msgs::Odometry getOdom() { 
    // return sdf_map_->getOdom(); 
    if(EDTEnv_mode==LOCAL_MODE){
      return sdf_map_->getOdom(); 
    }else if(EDTEnv_mode == GLOBAL_MODE){
      return sdf_map_global->getOdom();
    }else{
      printf("mode error!!\n");
      return nav_msgs::Odometry();
    }  
  }
  void getMapRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) { 
    // sdf_map_->getRegion(ori, size); 
    if(EDTEnv_mode==LOCAL_MODE){
      return sdf_map_->getRegion(ori, size); 
    }else if(EDTEnv_mode == GLOBAL_MODE){
      return sdf_map_global->getRegion(ori, size);
    }else{
      printf("mode error!!\n");
      return;
    } 
  }

  void set_mode(int mode){ EDTEnv_mode=mode; }

    //  jiangtao add
//    bool get_push_force(const Eigen::Matrix<double, 3, 1> current_odom, const double distance,
//                        Eigen::Matrix<double, 3, 1> &push_force);

  typedef shared_ptr<EDTEnvironment> Ptr;

};

}  // namespace dyn_planner

#endif