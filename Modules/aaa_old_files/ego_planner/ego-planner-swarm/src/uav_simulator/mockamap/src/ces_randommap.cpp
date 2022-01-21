#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <math.h>

#include <random>
#include <sys/time.h>
#include <time.h>

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

//! @todo historical above
#include "maps.hpp"

using namespace std;
using namespace mocka;

#if MAP_OR_WORLD
const string kFrameIdNs_ = "map";
#else
const string kFrameIdNs_ = "world";
#endif

pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;
vector<int>                        pointIdxRadiusSearch;
vector<float>                      pointRadiusSquaredDistance;

ros::Publisher _local_map_pub;
ros::Publisher _local_map_inflate_pub;
ros::Publisher _global_map_pub;

ros::Subscriber _map_sub;
ros::Subscriber _odom_sub;

deque<nav_msgs::Odometry> _odom_queue;
vector<double>            _state;
const size_t              _odom_queue_size = 200;
nav_msgs::Odometry        _odom;

double z_limit;
double _SenseRate;
double _sensing_range;

// ros::Timer vis_map;
bool map_ok    = false;
bool _has_odom = false;

sensor_msgs::PointCloud2       globalMap_pcd;
sensor_msgs::PointCloud2       localMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;
ros::Time                      begin_time = ros::TIME_MAX;

typedef Eigen::Vector3d ObsPos;
typedef Eigen::Vector3d ObsSize; // x, y, height --- z
typedef pair<ObsPos, ObsPos> Obstacle;
std::vector<Obstacle> obstacle_list;

void
fixedMapGenerate()
{
  double _resolution = 1.0;

  cloudMap.points.clear();
  obstacle_list.push_back(
    make_pair(ObsPos(-7.0, 1.0, 0.0), ObsSize(1.0, 3.0, 5.0)));
  obstacle_list.push_back(
    make_pair(ObsPos(-1.0, 1.0, 0.0), ObsSize(1.0, 3.0, 5.0)));
  obstacle_list.push_back(
    make_pair(ObsPos(10.0, 1.0, 0.0), ObsSize(1.0, 3.0, 5.0)));
  obstacle_list.push_back(
    make_pair(ObsPos(16.0, 1.0, 0.0), ObsSize(1.0, 3.0, 5.0)));
  obstacle_list.push_back(
    make_pair(ObsPos(-4.0, -1.0, 0.0), ObsSize(1.0, 3.0, 5.0)));
  obstacle_list.push_back(
    make_pair(ObsPos(13.0, -1.0, 0.0), ObsSize(1.0, 3.0, 5.0)));

  obstacle_list.push_back(
    make_pair(ObsPos(5.0, 2.5, 0.0), ObsSize(30.0, 1.0, 5.0)));
  obstacle_list.push_back(
    make_pair(ObsPos(5.0, -2.5, 0.0), ObsSize(30.0, 1.0, 5.0)));

  int           num_total_obs = obstacle_list.size();
  pcl::PointXYZ pt_insert;

  for (int i = 0; i < num_total_obs; i++)
  {
    double x, y, z;
    double lx, ly, lz;
    x  = (obstacle_list[i].first)[0];
    y  = (obstacle_list[i].first)[1];
    z  = (obstacle_list[i].first)[2];
    lx = (obstacle_list[i].second)[0];
    ly = (obstacle_list[i].second)[1];
    lz = (obstacle_list[i].second)[2];

    int num_mesh_x = ceil(lx / _resolution);
    int num_mesh_y = ceil(ly / _resolution);
    int num_mesh_z = ceil(lz / _resolution);

    int left_x, right_x, left_y, right_y, left_z, right_z;
    left_x  = -num_mesh_x / 2;
    right_x = num_mesh_x / 2;
    left_y  = -num_mesh_y / 2;
    right_y = num_mesh_y / 2;
    left_z  = 0;
    right_z = num_mesh_z;

    for (int r = left_x; r < right_x; r++)
      for (int s = left_y; s < right_y; s++)
      {
        for (int t = left_z; t < right_z; t++)
        {
          if ((r - left_x) * (r - right_x + 1) * (s - left_y) *
                (s - right_y + 1) * (t - left_z) * (t - right_z + 1) ==
              0)
          {
            pt_insert.x = x + r * _resolution;
            pt_insert.y = y + s * _resolution;
            pt_insert.z = z + t * _resolution;
            cloudMap.points.push_back(pt_insert);
          }
        }
      }
  }

  cloudMap.width    = cloudMap.points.size();
  cloudMap.height   = 1;
  cloudMap.is_dense = true;

  ROS_WARN("Finished generate random map ");
  cout << cloudMap.size() << endl;
  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());
  map_ok = true;
}

void
rcvOdometryCallbck(const nav_msgs::Odometry odom)
{
  if (odom.child_frame_id == "X" || odom.child_frame_id == "O")
    return;
  _odom     = odom;
  _has_odom = true;

  _state = { _odom.pose.pose.position.x,
             _odom.pose.pose.position.y,
             _odom.pose.pose.position.z,
             _odom.twist.twist.linear.x,
             _odom.twist.twist.linear.y,
             _odom.twist.twist.linear.z,
             0.0,
             0.0,
             0.0 };

  _odom_queue.push_back(odom);
  while (_odom_queue.size() > _odom_queue_size)
    _odom_queue.pop_front();
}

int frequence_division_global = 40;

void
publishAllPoints()
{
  if (!map_ok)
    return;

  if ((ros::Time::now() - begin_time).toSec() > 7.0)
    return;

  frequence_division_global--;
  if (frequence_division_global == 0)
  {
    pcl::toROSMsg(cloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = kFrameIdNs_;
    _global_map_pub.publish(globalMap_pcd);
    frequence_division_global = 40;
    ROS_ERROR("[SERVER]Publish one global map");
  }
}

void
pubSensedPoints()
{
  if (!map_ok || !_has_odom)
    return;

  // ros::Time time_bef_sensing = ros::Time::now();

  pcl::PointCloud<pcl::PointXYZ> localMap;

  pcl::PointXYZ searchPoint(_state[0], _state[1], _state[2]);
  pointIdxRadiusSearch.clear();
  pointRadiusSquaredDistance.clear();

  pcl::PointXYZ ptInNoflation;

  if (kdtreeLocalMap.radiusSearch(searchPoint, _sensing_range,
                                  pointIdxRadiusSearch,
                                  pointRadiusSquaredDistance) > 0)
  {
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
    {
      ptInNoflation = cloudMap.points[pointIdxRadiusSearch[i]];
      localMap.points.push_back(ptInNoflation);
    }
  }
  else
  {
    // ROS_ERROR("[Map server] No obstacles .");
    // cout<<searchPoint.x<<" , "<<searchPoint.y<<" , "<<searchPoint.z<<endl;
    // return;
  }

  pcl::PointXYZ pt_fix;
  pt_fix.x = _state[0];
  pt_fix.y = _state[1];
  pt_fix.z = 0.0;
  localMap.points.push_back(pt_fix);

  localMap.width    = localMap.points.size();
  localMap.height   = 1;
  localMap.is_dense = true;

  pcl::toROSMsg(localMap, localMap_pcd);

  localMap_pcd.header.frame_id = kFrameIdNs_;
  _local_map_pub.publish(localMap_pcd);

  ros::Time time_aft_sensing = ros::Time::now();

  if ((time_aft_sensing - begin_time).toSec() > 5.0)
    return;

  frequence_division_global--;
  if (frequence_division_global == 0)
  {
    pcl::toROSMsg(cloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = kFrameIdNs_;
    _global_map_pub.publish(globalMap_pcd);
    frequence_division_global = 40;
    ROS_INFO("[SERVER]Publish one global map");
  }
}
