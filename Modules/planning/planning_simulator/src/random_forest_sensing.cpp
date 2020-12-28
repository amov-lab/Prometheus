// 生成随机地图，并模拟无人机检测，模拟得到局部地图与全局地图

#include <ros/ros.h>
#include <ros/console.h>
#include <math.h>
#include <iostream>
#include <Eigen/Eigen>
#include <random>
#include <tf/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include "prometheus_msgs/DroneState.h"
using namespace std;

//无人机状态 [位置、速度、姿态]
vector<double> _state;

// 障碍物点云地图
pcl::PointCloud<pcl::PointXYZ> cloudMap;

// kdtree地图，为了索引方便？
pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;

sensor_msgs::PointCloud2 localMap_pcd;
sensor_msgs::PointCloud2 globalMap_pcd;


vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

random_device rd;
default_random_engine eng(rd());
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_h;

ros::Publisher local_map_pub;
ros::Publisher global_map_pub;
ros::Subscriber drone_state_sub;

int i = 0;

int _obs_num;
double _x_size, _y_size, _z_size;
double map_x_0, map_x_1, map_y_0, map_y_1, _w_l, _w_h, _h_l, _h_h;
double _z_limit, sense_range, _resolution, _sense_rate, _init_x, _init_y;

bool _map_ok = false;
bool _has_odom = false;

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr &msg)
{
  _has_odom = true;

  _state = { msg->position[0],
                     msg->position[1],
                    msg->position[2],
                    msg->velocity[0],
                     msg->velocity[1],
                    msg->velocity[2],
                    msg->attitude_q.w,
                     msg->attitude_q.x,
                    msg->attitude_q.y,
                    msg->attitude_q.z,
             0.0,
             0.0 };
}

void pubSensedPoints();
void RandomMapGenerate();

//  主函数
int main(int argc, char** argv)
{
  ros::init(argc, argv, "random_map_sensing");
  ros::NodeHandle n("~");

  // 订阅无人机位置信息
  drone_state_sub = n.subscribe("/prometheus/drone_state", 50, drone_state_cb);

  // 发布 模拟的局部点云
  local_map_pub = n.advertise<sensor_msgs::PointCloud2>("/prometheus/planning/local_pcl_sim", 1);
  
  // 发布 模拟的全局点云
  global_map_pub = n.advertise<sensor_msgs::PointCloud2>("/prometheus/planning/global_pcl_sim", 1);

  // 参数读取
  // 初始位置
  n.param("init_pos_x", _init_x, 0.0);
  n.param("init_pos_y", _init_y, 0.0);

  // 地图尺寸、分辨率
  n.param("map/x_size", _x_size, 10.0);
  n.param("map/y_size", _y_size, 10.0);
  n.param("map/z_size", _z_size, 5.0);
  n.param("map/resolution", _resolution, 0.2);

  // 障碍物
  n.param("obs/obs_num", _obs_num, 30);
  n.param("obs/min_radius", _w_l, 0.3);
  n.param("obs/max_radius", _w_h, 0.8);
  n.param("obs/min_height", _h_l, 3.0);
  n.param("obs/max_height", _h_h, 7.0);

  // 传感器感知半径与感知频率
  n.param("sensing/sense_range", sense_range, 10.0);
  n.param("sensing/sense_rate", _sense_rate, 10.0);

  // 运行频率 = 传感器更新频率
  ros::Rate loop_rate(_sense_rate);

  // 地图坐标 [x_0,  x_1] - [y_0, y_1]
  map_x_0 = -_x_size / 2.0;
  map_x_1 = +_x_size / 2.0;

  map_y_0 = -_y_size / 2.0;
  map_y_1 = +_y_size / 2.0;

  _z_limit = _z_size;

  _obs_num = min(_obs_num, (int)_x_size * 10);

  ros::Duration(0.1).sleep();

  //生成随机地图
  RandomMapGenerate();

  while (ros::ok())
  {
    // 更新无人机位置
    ros::spinOnce();

    // 发布感知数据
    pubSensedPoints();
    
    loop_rate.sleep();
  }
}


void RandomMapGenerate()
{
  pcl::PointXYZ pt_random;

  // uniform_real_distribution函数：
  rand_x = uniform_real_distribution<double>(map_x_0, map_x_1);
  rand_y = uniform_real_distribution<double>(map_y_0, map_y_1);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);

  //  循环生成随机障碍物
  for (int i = 0; i < _obs_num; i++)
  {
    // 得到障碍物的随机xy位置和大小，高度
    double x, y, w, h;
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);
    h = rand_h(eng);

    // 如果障碍物距离无人机初始位置 过近，则跳过
    if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 1.5)
      continue;

    // 障碍物中心点坐标？
    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    // 障碍物占据的格子数
    int widNum = ceil(w / _resolution);

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
      {
        int heiNum = ceil(h / _resolution);
        for (int t = -0; t < heiNum; t++)
        {
          pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
          pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
          pt_random.z = (t + 0.5) * _resolution + 1e-2;
          // 将障碍物点推入cloudMap
          cloudMap.points.push_back(pt_random);
        }
      }
  }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  ROS_WARN("Finished generate random map ");

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}


void pubSensedPoints()
{
  // 定时更新全局点云
  static int num = 0;

  if (num  == 10) 
  {
    pcl::toROSMsg(cloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = "world";
    global_map_pub.publish(globalMap_pcd);
    num=0;
  }else
  {
    num++;
  }

  if (!_map_ok || !_has_odom)
  {
    printf("[generate random map]: don't have the odom ");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> localMap;

  pcl::PointXYZ searchPoint(_state[0], _state[1], _state[2]);
  pointIdxRadiusSearch.clear();
  pointRadiusSquaredDistance.clear();

  pcl::PointXYZ pt;
  pcl::PointXYZ pt_new;

  if (isnan(searchPoint.x) || isnan(searchPoint.y) || isnan(searchPoint.z)){
    printf("[generate random map]: the odom state is bad, can't generate the local map!\n");
    return;
  }
    
Eigen::Vector3d p3d;
Eigen::Matrix<double, 3,3> rotaion_mat_global_to_local = Eigen::Quaterniond(_state[6], 
                                                                                                                                                                  _state[7],
                                                                                                                                                                  _state[8],
                                                                                                                                                                  _state[9]).toRotationMatrix().transpose();

  if (kdtreeLocalMap.radiusSearch(searchPoint, sense_range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
  {
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
    {
      pt = cloudMap.points[pointIdxRadiusSearch[i]];
      {
        p3d(0)=pt.x - searchPoint.x;
        p3d(1)=pt.y - searchPoint.y;
        p3d(2)=pt.z - searchPoint.z;
        p3d = rotaion_mat_global_to_local *(p3d);
        pt_new.x = p3d(0);
        pt_new.y = p3d(1);
        pt_new.z = p3d(2);
      }
      localMap.points.push_back(pt_new);
      // localMap.points.push_back(pt);
    }
  }
  else
  {
    ROS_ERROR("[Map server] No obstacles .");
    return;
  }

  localMap.width = localMap.points.size();
  localMap.height = 1;
  localMap.is_dense = true;

  // 发布局部点云,此处需要发布 base_link与 world的关系
  pcl::toROSMsg(localMap, localMap_pcd);
  localMap_pcd.header.frame_id = "base_link";
  local_map_pub.publish(localMap_pcd);

  //发布tf
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(_state[0], _state[1], _state[2]) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));

}
