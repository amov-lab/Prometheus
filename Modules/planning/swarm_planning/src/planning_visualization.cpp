#include <planning_visualization.h>

using std::cout;
using std::endl;
namespace Swarm_Planning
{

PlanningVisualization::PlanningVisualization(ros::NodeHandle& nh)
{
  node = nh;

  traj_pub = node.advertise<visualization_msgs::Marker>("/planning_vis/trajectory", 10);
}

void PlanningVisualization::displaySphereList(vector<Eigen::Vector3d> list, double resolution, Eigen::Vector4d color,
                                              int id)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::DELETE;
  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  mk.color.r = color(0), mk.color.g = color(1), mk.color.b = color(2), mk.color.a = color(3);
  mk.scale.x = resolution, mk.scale.y = resolution, mk.scale.z = resolution;
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++)
  {
    pt.x = list[i](0), pt.y = list[i](1), pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  traj_pub.publish(mk);
}


void PlanningVisualization::drawGoal(Eigen::Vector3d goal, double resolution, Eigen::Vector4d color, int id)
{
  vector<Eigen::Vector3d> goal_vec = { goal };

  displaySphereList(goal_vec, resolution, color, GOAL + id % 100);
}

void PlanningVisualization::drawPath(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id)
{
  displaySphereList(path, resolution, color, PATH + id % 100);
}

void PlanningVisualization::drawrosPath(nav_msgs::Path ros_path, double resolution, Eigen::Vector4d color, int id)
{
  vector<Eigen::Vector3d> path;

  int size = ros_path.poses.size();

  Eigen::Vector3d path_point;

  for (int i = 0; i < size; ++i)
  {
    path_point[0] = ros_path.poses[i].pose.position.x;
    path_point[1] = ros_path.poses[i].pose.position.y;
    path_point[2] = ros_path.poses[i].pose.position.z;
    path.push_back(path_point);
  }
  
  displaySphereList(path, resolution, color, PATH + id % 100);
}



void PlanningVisualization::drawVel(Eigen::Vector3d pos, Eigen::Vector3d vec, Eigen::Vector4d color, int id){
    visualization_msgs::Marker mk_state;
    mk_state.header.frame_id = "world";
    mk_state.header.stamp = ros::Time::now();
    mk_state.id = id;
    mk_state.type = visualization_msgs::Marker::ARROW;
    mk_state.action = visualization_msgs::Marker::ADD;
    mk_state.pose.orientation.w = 1.0;
    mk_state.pose.orientation.x = 0.0;
    mk_state.pose.orientation.y = 0.0;
    mk_state.pose.orientation.z = 0.0;
    mk_state.scale.x = 0.1;
    mk_state.scale.y = 0.2;
    mk_state.scale.z = 0.3;
    geometry_msgs::Point pt;
    pt.x = pos(0);
    pt.y = pos(1);
    pt.z = pos(2);
    mk_state.points.push_back(pt);
    pt.x = pos(0) + vec(0);
    pt.y = pos(1) + vec(1);
    pt.z = pos(2) + vec(2);
    mk_state.points.push_back(pt);
    mk_state.color.r = color(0);
    mk_state.color.g = color(1);
    mk_state.color.b = color(2);
    mk_state.color.a = color(3);

    traj_pub.publish(mk_state);

}

// PlanningVisualization::
}  