#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <cmath>

using std::vector;

ros::Publisher _pub_cloud;
ros::Subscriber _odom_sub;
ros::Subscriber _global_map_sub;

ros::Timer _local_sensing_timer;

bool _has_global_map(false);
bool _has_odom(false);
bool _use_resolution_filter(true);

nav_msgs::Odometry _odom;

double _sensing_horizon, _sensing_rate, _pc_resolution;

int _hrz_laser_line_num, _vtc_laser_line_num;
double _hrz_resolution_rad, _vtc_resolution_rad, _vtc_laser_range_rad;
double _half_vtc_resolution_and_half_range;

// line_p must be a normalized vector
// d can be negtive, indicating ...
inline double lineIntersectPlane(Eigen::Vector3d &inter_p, const Eigen::Vector3d &line_p, const Eigen::Vector3d &line_dir,
                                 const Eigen::Vector3d &plane_p, const Eigen::Vector3d &plane_normal)
{
  double d = (plane_p - line_p).dot(plane_normal) / line_dir.dot(plane_normal);
  inter_p = line_p + d * line_dir;
  return d;
}

// laser_R must be normalized in rach column
inline bool pt2LaserIdx(Eigen::Vector2i &idx, const Eigen::Vector3d &pt, const Eigen::Vector3d &laser_t, const Eigen::Matrix3d &laser_R)
{
  Eigen::Vector3d inter_p;
  double dis_pt_to_plane = lineIntersectPlane(inter_p, pt, laser_R.col(2), laser_t, laser_R.col(2));
  double dis_laser_to_inter_p = (inter_p - laser_t).norm();
  double vtc_rad = std::atan2((pt - laser_t).dot(laser_R.col(2)), dis_laser_to_inter_p);

  // ROS_WARN_STREAM("vtc_rad" << vtc_rad);

  if (std::fabs(vtc_rad) >= _half_vtc_resolution_and_half_range)
    return false;
  vtc_rad = vtc_rad + _half_vtc_resolution_and_half_range;
  int vtc_idx = std::floor(vtc_rad / _vtc_resolution_rad);
  // ROS_WARN_STREAM("vtc_idx: " << vtc_idx);
  if (vtc_idx >= _vtc_laser_line_num)
    vtc_idx = 0;

  double x_in_roll_pitch_plane = (inter_p - laser_t).dot(laser_R.col(0));
  double y_in_roll_pitch_plane = (inter_p - laser_t).dot(laser_R.col(1));
  double hrz_rad = std::atan2(y_in_roll_pitch_plane, x_in_roll_pitch_plane) + M_PI + _hrz_resolution_rad / 2.0;
  int hrz_idx = std::floor(hrz_rad / _hrz_resolution_rad);
  if (hrz_idx >= _hrz_laser_line_num)
    hrz_idx = 0;

  idx << hrz_idx, vtc_idx;
  return true;
}

inline void idx2Pt(int x, int y, const Eigen::Vector3d &t, const Eigen::Matrix3d &R, double dis, Eigen::Vector3d &pt)
{
  double vtc_rad = y * _vtc_resolution_rad - _vtc_laser_range_rad / 2.0;
  double hrz_rad = x * _hrz_resolution_rad - M_PI;
  double z_in_laser_coor = sin(vtc_rad) * dis;
  double xy_square_in_laser_coor = cos(vtc_rad) * dis;
  double x_in_laser_coor = cos(hrz_rad) * xy_square_in_laser_coor;
  double y_in_laser_coor = sin(hrz_rad) * xy_square_in_laser_coor;
  Eigen::Vector3d pt_in_laser_coor(x_in_laser_coor, y_in_laser_coor, z_in_laser_coor);
  pt = R * pt_in_laser_coor + t;
}

void rcvOdometryCallbck(const nav_msgs::Odometry &odom)
{
  /*if(!_has_global_map)
    return;*/
  _has_odom = true;
  _odom = odom;
}

pcl::PointCloud<pcl::PointXYZ> _cloud_all_map, _local_map;
pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
sensor_msgs::PointCloud2 _local_map_pcd;

pcl::search::KdTree<pcl::PointXYZ> _kdtreeLocalMap;
vector<int> _pointIdxRadiusSearch;
vector<float> _pointRadiusSquaredDistance;

void rcvGlobalPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map)
{
  if (_has_global_map)
    return;

  ROS_WARN("Global Pointcloud received..");

  pcl::PointCloud<pcl::PointXYZ> cloud_input;
  pcl::fromROSMsg(pointcloud_map, cloud_input);

  _voxel_sampler.setLeafSize(_pc_resolution, _pc_resolution, _pc_resolution);
  _voxel_sampler.setInputCloud(cloud_input.makeShared());
  _voxel_sampler.filter(_cloud_all_map);

  _kdtreeLocalMap.setInputCloud(_cloud_all_map.makeShared());

  _has_global_map = true;
}

void renderSensedPoints(const ros::TimerEvent &event)
{
  double this_time = ros::Time::now().toSec();
  if (!_has_global_map || !_has_odom)
    return;

  Eigen::Quaterniond q;
  q.x() = _odom.pose.pose.orientation.x;
  q.y() = _odom.pose.pose.orientation.y;
  q.z() = _odom.pose.pose.orientation.z;
  q.w() = _odom.pose.pose.orientation.w;
  Eigen::Matrix3d rot;
  rot = q;

  Eigen::Vector3d laser_t(_odom.pose.pose.position.x, _odom.pose.pose.position.y, _odom.pose.pose.position.z);

  _local_map.points.clear();
  pcl::PointXYZ searchPoint(_odom.pose.pose.position.x, _odom.pose.pose.position.y, _odom.pose.pose.position.z);
  _pointIdxRadiusSearch.clear();
  _pointRadiusSquaredDistance.clear();

  pcl::PointXYZ pt;
  if (_kdtreeLocalMap.radiusSearch(searchPoint, _sensing_horizon, _pointIdxRadiusSearch, _pointRadiusSquaredDistance) > 0)
  {
    Eigen::MatrixXi idx_map = Eigen::MatrixXi::Constant(_hrz_laser_line_num, _vtc_laser_line_num, -1);
    Eigen::MatrixXd dis_map = Eigen::MatrixXd::Constant(_hrz_laser_line_num, _vtc_laser_line_num, 9999.0);

    for (size_t i = 0; i < _pointIdxRadiusSearch.size(); ++i)
    {
      pt = _cloud_all_map.points[_pointIdxRadiusSearch[i]];

      Eigen::Vector2i idx;
      bool in_range = pt2LaserIdx(idx, Eigen::Vector3d(pt.x, pt.y, pt.z), laser_t, rot);
      if (!in_range)
        continue;
      Eigen::Vector3d pt_vec(pt.x - _odom.pose.pose.position.x, pt.y - _odom.pose.pose.position.y, pt.z - _odom.pose.pose.position.z);
      double dis_curr_pt = pt_vec.norm();
      
      if (_use_resolution_filter)
      {
        double vtc_rad = idx[1] * _vtc_resolution_rad - _vtc_laser_range_rad / 2.0;
        double mesh_len_hrz = dis_curr_pt * cos(vtc_rad) * _hrz_resolution_rad;
        double mesh_len_vtc = dis_curr_pt * cos(vtc_rad) * _vtc_resolution_rad;
        int hrz_occ_grid_num = std::min((int)floor(_pc_resolution / mesh_len_hrz), _hrz_laser_line_num);
        int vtc_occ_grid_num = std::min((int)floor(_pc_resolution / mesh_len_vtc), _vtc_laser_line_num);
        // ROS_INFO_STREAM("hrz_occ_grid_num " << hrz_occ_grid_num / 2 << ", vtc_occ_grid_num " << vtc_occ_grid_num / 2);
        int tmp1 = hrz_occ_grid_num, tmp2 = vtc_occ_grid_num;
        for (int d_hrz_idx = -tmp1; d_hrz_idx <= tmp1; ++d_hrz_idx)
          for (int d_vtc_idx = -tmp2; d_vtc_idx <= tmp2; ++d_vtc_idx)
          {
            int hrz_idx = (idx[0] + d_hrz_idx + _hrz_laser_line_num) % _hrz_laser_line_num;// it's a ring in hrz coordiante
            int vtc_idx = idx[1] + d_vtc_idx;
            if (vtc_idx >= _vtc_laser_line_num) continue;
            if (vtc_idx < 0) continue;
            // ROS_INFO_STREAM("hrz_idx " << hrz_idx << ", vtc_idx " << vtc_idx);
            if (dis_curr_pt < dis_map(hrz_idx, vtc_idx))
            {
              idx_map(hrz_idx, vtc_idx) = i;
              dis_map(hrz_idx, vtc_idx) = dis_curr_pt;
            }
          } 
      }
      else
      {
        if (dis_curr_pt < dis_map(idx[0], idx[1]))
        {
          idx_map(idx[0], idx[1]) = i;
          dis_map(idx[0], idx[1]) = dis_curr_pt;
        }
      }
    }
    // ROS_INFO("render cost %lf ms.", (ros::Time::now().toSec() - this_time) * 1000.0f);
    for (int x = 0; x < _hrz_laser_line_num; ++x)
      for (int y = 0; y < _vtc_laser_line_num; ++y)
      {
        // if (idx_map(x, y) == -1)
        //   continue;
        // // ROS_WARN_STREAM("idx_map(x, y): " << idx_map(x, y));
        // // ROS_WARN_STREAM("_pointIdxRadiusSearch[idx_map(x, y)]: " << _pointIdxRadiusSearch[idx_map(x, y)]);
        // pt = _cloud_all_map.points[_pointIdxRadiusSearch[idx_map(x, y)]];
        // _local_map.points.push_back(pt);
        
        Eigen::Vector3d p;
        if (idx_map(x, y) == -1)
        {
          idx2Pt(x, y, laser_t, rot, _sensing_horizon + _pc_resolution, p);
          pt.x = p[0]; pt.y = p[1]; pt.z = p[2];
          _local_map.points.push_back(pt);
        }
        else
        {
          // idx2Pt(x, y, laser_t, rot, dis_map(x, y), p);
          // pt.x = p[0]; pt.y = p[1]; pt.z = p[2];
          pt = _cloud_all_map.points[_pointIdxRadiusSearch[idx_map(x, y)]];
          _local_map.points.push_back(pt);
        }
      }
  }
  else
  {
    return;
  }

  _local_map.width = _local_map.points.size();
  _local_map.height = 1;
  _local_map.is_dense = true;

  pcl::toROSMsg(_local_map, _local_map_pcd);
  _local_map_pcd.header.frame_id = "map";

  _pub_cloud.publish(_local_map_pcd);
  // ROS_INFO("all cost %lf ms.", (ros::Time::now().toSec() - this_time) * 1000.0f);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_simulator");
  ros::NodeHandle nh("~");

  nh.getParam("sensing_horizon", _sensing_horizon);
  nh.getParam("sensing_rate", _sensing_rate);
  nh.getParam("pc_resolution", _pc_resolution);
  nh.getParam("use_resolution_filter", _use_resolution_filter);

  nh.getParam("hrz_laser_line_num", _hrz_laser_line_num);
  nh.getParam("vtc_laser_line_num", _vtc_laser_line_num);
  double vtc_laser_range_dgr;
  nh.getParam("vtc_laser_range_dgr", vtc_laser_range_dgr);
  _vtc_laser_range_rad = vtc_laser_range_dgr / 180.0 * M_PI;
  _vtc_resolution_rad = _vtc_laser_range_rad / (double)(_vtc_laser_line_num - 1);
  _hrz_resolution_rad = 2 * M_PI / (double)_hrz_laser_line_num;
  _half_vtc_resolution_and_half_range = (_vtc_laser_range_rad + _vtc_resolution_rad) / 2.0;

  //subscribe point cloud
  _global_map_sub = nh.subscribe("global_map", 1, rcvGlobalPointCloudCallBack);
  _odom_sub = nh.subscribe("odometry", 50, rcvOdometryCallbck);

  //publisher depth image and color image
  _pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("local_pointcloud", 10);
  double sensing_duration = 1.0 / _sensing_rate;
  _local_sensing_timer = nh.createTimer(ros::Duration(sensing_duration), renderSensedPoints);

  ros::spin();
}
