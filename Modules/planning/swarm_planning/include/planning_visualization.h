#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>


using std::vector;
namespace Swarm_Planning
{
class PlanningVisualization
{
private:
  enum DRAW_ID
  {
    GOAL = 1,
    PATH = 200,
    BSPLINE = 300,
    BSPLINE_CTRL_PT = 400,
    PREDICTION = 500
  };

  /* data */
  ros::NodeHandle node;
  ros::Publisher traj_pub;

  void displaySphereList(vector<Eigen::Vector3d> list, double resolution, Eigen::Vector4d color, int id);

public:
  PlanningVisualization(void)
  {}
  ~PlanningVisualization()
  {}

  PlanningVisualization(ros::NodeHandle& nh);

  void drawPath(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id = 0);

  void drawrosPath(nav_msgs::Path ros_path, double resolution, Eigen::Vector4d color, int id = 0);

  void drawGoal(Eigen::Vector3d goal, double resolution, Eigen::Vector4d color, int id = 0);

  void drawVel(Eigen::Vector3d pos, Eigen::Vector3d vec, Eigen::Vector4d color, int id = 0);

  typedef std::shared_ptr<PlanningVisualization> Ptr;
};
}  // namespace dyn_planner
#endif