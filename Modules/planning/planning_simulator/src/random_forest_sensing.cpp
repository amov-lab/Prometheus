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

// 初始状态量
bool _map_ok = false;
bool _has_odom = false;
// 无人机初始位置
double _init_x, _init_y;
// 传感器感知半径与频率
double sense_range, sense_rate;
// 地图尺寸
double map_size_x, map_size_y, map_size_z;
// 地图xy范围
double map_x_0, map_x_1, map_y_0, map_y_1;
double _z_limit;
// 地图分辨率
double _resolution;
//  障碍物最小间隔
double _min_dist;
// 障碍物数量
int _pillar_num,_circle_num;
// 柱子形状范围
double  _w_l, _w_h, _h_l, _h_h;
// 圆形形状范围
double radius_l_, radius_h_, z_l_, z_h_;
double theta_;


random_device rd;
default_random_engine eng(rd());
// 产生障碍物随机位置
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_z;
// 产生障碍物随机形状
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_h;
uniform_real_distribution<double> rand_inf;
uniform_real_distribution<double> rand_radius;
uniform_real_distribution<double> rand_radius2;
uniform_real_distribution<double> rand_theta;

//无人机状态 [位置、速度、姿态]
ros::Subscriber drone_state_sub;
vector<double> _state;

// kdtree地图，为了索引方便？
pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

// 障碍物点云地图
pcl::PointCloud<pcl::PointXYZ> cloudMap;
// 用于发布的局部点云和全局点云
sensor_msgs::PointCloud2 localMap_pcd;
sensor_msgs::PointCloud2 globalMap_pcd;

ros::Publisher local_map_pub;
ros::Publisher global_map_pub;

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
  n.param("map/x_size", map_size_x, 10.0);
  n.param("map/y_size", map_size_y, 10.0);
  n.param("map/z_size", map_size_z, 5.0);
  n.param("map/resolution", _resolution, 0.2);
  // 障碍物 - 两种障碍物：柱子与圆
  n.param("map/circle_num", _circle_num, 30);
  n.param("map/pillar_num", _pillar_num, 30);
  // 障碍物最小距离
  n.param("map/min_distance", _min_dist, 1.0);

  n.param("pillar_shape/min_radius", _w_l, 0.3);
  n.param("pillar_shape/max_radius", _w_h, 0.8);
  n.param("pillar_shape/min_height", _h_l, 3.0);
  n.param("pillar_shape/max_height", _h_h, 7.0);
  n.param("circle_shape/min_radius", radius_l_, 7.0);
  n.param("circle_shape/max_radius", radius_h_, 7.0);
  n.param("circle_shape/min_height", z_l_, 7.0);
  n.param("circle_shape/max_height", z_h_, 7.0);
  n.param("circle_shape/theta", theta_, 7.0);

  // 传感器感知半径与感知频率
  n.param("sensing/sense_range", sense_range, 10.0);
  n.param("sensing/sense_rate", sense_rate, 10.0);
  

  // 运行频率 = 传感器更新频率
  ros::Rate loop_rate(sense_rate);

  // 地图坐标范围 [map_x_0,  map_x_1] - [map_y_0, map_y_1]
  map_x_0 = -map_size_x / 2.0;
  map_x_1 = +map_size_x / 2.0;

  map_y_0 = -map_size_y / 2.0;
  map_y_1 = +map_size_y / 2.0;

  _z_limit = map_size_z;

  _pillar_num = min(_pillar_num, (int)map_size_x * 10);

  ros::Duration(1.0).sleep();

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
  vector<Eigen::Vector2d> obs_position;

  // uniform_real_distribution函数：
  rand_x = uniform_real_distribution<double>(map_x_0, map_x_1);
  rand_y = uniform_real_distribution<double>(map_y_0, map_y_1);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);
  // 随机膨胀系数 暂时不使用
  rand_inf = uniform_real_distribution<double>(0.5, 1.5);

  rand_radius = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2 = uniform_real_distribution<double>(radius_l_, 1.2);
  rand_theta = uniform_real_distribution<double>(-theta_, theta_);
  rand_z = uniform_real_distribution<double>(z_l_, z_h_);

  //  循环生成柱子障碍物
  for (int i = 0; i < _pillar_num; i++)
  {
    // 得到障碍物的随机xy位置和大小，高度
    double x, y, w, h;
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);
    h = rand_h(eng);

    // 如果障碍物距离无人机初始位置 过近，则跳过
    if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < _min_dist)
      continue;

    // 如果障碍物与之前障碍物距离过近，则跳过
    bool flag_continue = false;
    for ( auto p : obs_position )
      if ( (Eigen::Vector2d(x,y) - p).norm() < _min_dist)
      {
        i--;
        flag_continue = true;
        break;
      }
    if ( flag_continue ) continue;

    obs_position.push_back( Eigen::Vector2d(x,y) );

    // 障碍物中心点坐标？
    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    // 障碍物占据的格子数
    int widNum = ceil(w / _resolution);
    double radius = (w) / 2;

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
      {
        int heiNum = ceil(h / _resolution);
        for (int t = -0; t < heiNum; t++)
        {
          double temp_x = x + (r + 0.5) * _resolution + 1e-2;
          double temp_y = y + (s + 0.5) * _resolution + 1e-2;
          double temp_z = (t + 0.5) * _resolution + 1e-2;
          if ( (Eigen::Vector2d(temp_x,temp_y) - Eigen::Vector2d(x,y)).norm() <= radius )
          {
            pt_random.x = temp_x;
            pt_random.y = temp_y;
            pt_random.z = temp_z;
            // 将障碍物点推入cloudMap
            cloudMap.points.push_back(pt_random);
          }
        }
      }
  }

  //  生成随机圆形障碍物
  for (int i = 0; i < _circle_num; ++i) 
  {
    double x, y, z;
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z(eng);

    // 坐标
    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    Eigen::Vector3d translate(x, y, z);

    // 偏转角度？
    double theta = rand_theta(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
        1;

    // 内、外径
    double radius1 = rand_radius(eng);
    double radius2 = rand_radius2(eng);

    // draw a circle centered at (x,y,z)
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) 
    {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);
      cpt(2) = radius2 * sin(angle);

      // inflate
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz) {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                           ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            cloudMap.push_back(pt_random);
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
