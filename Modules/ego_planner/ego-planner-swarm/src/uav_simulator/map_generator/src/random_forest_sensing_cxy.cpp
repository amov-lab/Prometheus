#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <Eigen/Eigen>
#include <random>

using namespace std;
using namespace Eigen;

const int MAX_SWRAM_NUM = 40;

ros::Publisher _all_map_pub;
ros::Timer global_pcl_timer;
ros::Publisher pub_cloud[MAX_SWRAM_NUM+1];
ros::Subscriber _odom_sub[MAX_SWRAM_NUM+1];
nav_msgs::Odometry _odom[MAX_SWRAM_NUM+1];
ros::Timer local_sensing_timer[MAX_SWRAM_NUM+1];

ros::Publisher pub_cloud_ugv[MAX_SWRAM_NUM+1];
ros::Subscriber _odom_ugv_sub[MAX_SWRAM_NUM+1];
nav_msgs::Odometry _odom_ugv[MAX_SWRAM_NUM+1];
ros::Timer local_sensing_ugv_timer[MAX_SWRAM_NUM+1];

pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

random_device rd;
default_random_engine eng(rd()); 
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;

vector<double> _state;
int map_flag, swarm_num, swarm_num_ugv, uav_id[MAX_SWRAM_NUM+1], ugv_id[MAX_SWRAM_NUM+1];
std::string uav_name[MAX_SWRAM_NUM+1],ugv_name[MAX_SWRAM_NUM+1];
double _x_size, _y_size, _z_size;
double _x_l, _x_h, _y_l, _y_h;
double _sensing_range, sensing_horizon, _resolution, _sense_rate, _init_x, _init_y;
double _min_dist;

bool _map_ok = false;
bool _has_odom[MAX_SWRAM_NUM+1] = {false};
bool _has_odom_ugv[MAX_SWRAM_NUM+1] = {false};

// 障碍物参数 - 圆柱体、正方体
int cylinder_num;
double cylinder_radius, cylinder_height;
int sqaure_num;
double sqaure_size;

sensor_msgs::PointCloud2 globalMap_pcd;
sensor_msgs::PointCloud2 localMap_pcd;

pcl::PointCloud<pcl::PointXYZ> cloudMap,local_map;


void MapGenerateCXY_1() 
{
  // 待存入全局点云的点
  pcl::PointXYZ pt_random;

  double x, y,z;

  // 生成圆柱体障碍物
  for (int i = 1; i <= 21; i++)
  {
    for (int j = 1; j <= 21; j++)
    {

      x = -18 + 2*i;
      y = -18 + 2*j;

      if(j%2 == 1)
      {
        // y轴单数生成圆柱体
        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;

        int widNum = ceil((cylinder_radius) / _resolution);
        int heiNum = ceil(cylinder_height / _resolution);

        // 生成圆柱体的点云
        for (int r = -widNum; r < widNum; r++)
          for (int s = -widNum; s < widNum; s++) {
            for (int t = -1.0; t < heiNum; t++) {
              double temp_x = x + (r + 0.5) * _resolution + 1e-2;
              double temp_y = y + (s + 0.5) * _resolution + 1e-2;
              double temp_z = (t + 0.5) * _resolution + 1e-2;
              if ( (Eigen::Vector2d(temp_x,temp_y) - Eigen::Vector2d(x,y)).norm() <= cylinder_radius )
              {
                pt_random.x = temp_x;
                pt_random.y = temp_y;
                pt_random.z = temp_z;
                cloudMap.points.push_back(pt_random);
              }
            }
          }

      }else
      {
        // y轴单数生成正方体
        z = sqaure_size / 2;
        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;
        z = floor(z / _resolution) * _resolution + _resolution / 2.0;

        int widNum = ceil((sqaure_size) / _resolution);

        // 生成正方体的点云
        for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
          for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
            for (int t = -widNum / 2.0; t < widNum / 2.0; t++) {
              double temp_x = x + (r + 0.5) * _resolution + 1e-2;
              double temp_y = y + (s + 0.5) * _resolution + 1e-2;
              double temp_z = z + (t + 0.5) * _resolution + 1e-2;
              pt_random.x = temp_x;
              pt_random.y = temp_y;
              pt_random.z = temp_z;
              cloudMap.points.push_back(pt_random);
            }
          }
      }
      
    }
  }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  ROS_INFO("\033[1;33;41m----> Finished generate CXY map 1.\033[0m");

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}

void MapGenerateCXY_2() 
{
  // 待存入全局点云的点
  pcl::PointXYZ pt_random;

  double x, y,z;

  // 生成圆柱体障碍物
  for (int i = 1; i <= 3; i++)
  {
    for (int j = 1; j <= 3; j++)
    {

      x = -4 + 2*i;
      y = -4 + 2*j;

      if(j%2 == 1)
      {
        // y轴单数生成圆柱体
        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;

        int widNum = ceil((cylinder_radius) / _resolution);
        int heiNum = ceil(cylinder_height / _resolution);

        // 生成圆柱体的点云
        for (int r = -widNum; r < widNum; r++)
          for (int s = -widNum; s < widNum; s++) {
            for (int t = -1.0; t < heiNum; t++) {
              double temp_x = x + (r + 0.5) * _resolution + 1e-2;
              double temp_y = y + (s + 0.5) * _resolution + 1e-2;
              double temp_z = (t + 0.5) * _resolution + 1e-2;
              if ( (Eigen::Vector2d(temp_x,temp_y) - Eigen::Vector2d(x,y)).norm() <= cylinder_radius )
              {
                pt_random.x = temp_x;
                pt_random.y = temp_y;
                pt_random.z = temp_z;
                cloudMap.points.push_back(pt_random);
              }
            }
          }

      }else
      {
        // y轴单数生成正方体
        z = sqaure_size / 2;
        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;
        z = floor(z / _resolution) * _resolution + _resolution / 2.0;

        int widNum = ceil((sqaure_size) / _resolution);

        // 生成正方体的点云
        for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
          for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
            for (int t = -widNum / 2.0; t < widNum / 2.0; t++) {
              double temp_x = x + (r + 0.5) * _resolution + 1e-2;
              double temp_y = y + (s + 0.5) * _resolution + 1e-2;
              double temp_z = z + (t + 0.5) * _resolution + 1e-2;
              pt_random.x = temp_x;
              pt_random.y = temp_y;
              pt_random.z = temp_z;
              cloudMap.points.push_back(pt_random);
            }
          }
      }
      
    }
  }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  ROS_INFO("\033[1;33;41m----> Finished generate CXY map 2.\033[0m");

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}

void generate_cylinder(double x,double y)
{
  // 必须在地图范围内
  if(x<_x_l || x>_x_h || y<_y_l ||  y>_y_h)
  {
    return;
  }

  pcl::PointXYZ pt_random;
  x = floor(x/_resolution)*_resolution + _resolution/2.0;
  y = floor(y/_resolution)*_resolution + _resolution/2.0;
  int widNum = ceil((cylinder_radius)/_resolution);
  int heiNum = ceil(cylinder_height/_resolution);
  for(int r = -widNum; r < widNum; r++)
    for(int s = -widNum; s < widNum; s++) 
      for(int t = -1.0; t < heiNum; t++) {
        double temp_x = x + (r + 0.5)*_resolution + 1e-2;
        double temp_y = y + (s + 0.5)*_resolution + 1e-2;
        double temp_z = (t + 0.5)*_resolution + 1e-2;
        if((Eigen::Vector2d(temp_x,temp_y) - Eigen::Vector2d(x,y)).norm() <= cylinder_radius)
        {
          pt_random.x = temp_x;
          pt_random.y = temp_y;
          pt_random.z = temp_z;
          cloudMap.points.push_back(pt_random);
        }
      }
}

void generate_cube(double x,double y)
{
  // 必须在地图范围内
  if(x<_x_l || x>_x_h || y<_y_l ||  y>_y_h)
  {
    return;
  }
  
  pcl::PointXYZ pt_random;
  double z = sqaure_size/2.0;
  x = floor(x/_resolution)*_resolution + _resolution/2.0;
  y = floor(y/_resolution)*_resolution + _resolution/2.0;
  z = floor(z/_resolution)*_resolution + _resolution/2.0;
  int widNum = ceil((sqaure_size)/_resolution);
  for(int r = -widNum / 2.0; r < widNum / 2.0; r++)
    for(int s = -widNum / 2.0; s < widNum / 2.0; s++) 
      for(int t = -widNum / 2.0; t < widNum / 2.0; t++) {
        double temp_x = x + (r + 0.5)*_resolution + 1e-2;
        double temp_y = y + (s + 0.5)*_resolution + 1e-2;
        double temp_z = z + (t + 0.5)*_resolution + 1e-2;
        pt_random.x = temp_x;
        pt_random.y = temp_y;
        pt_random.z = temp_z;
        cloudMap.points.push_back(pt_random);
      }
}

int sign(int num, double mid,bool zero_flag)
{
  if(num>mid) return -1;
  if(zero_flag && num==mid) return 0;
  else return 1;
}

void MapGenerateCXY_3() 
{
  double x,y;
  for(int num = 0; num < 10; num++)
  {
    // format1
    for(int ci = 0; ci < 5; ci++)
    {
      x = sign(num,4,0) * (ci*2 + 3);
      y = (num*4*2)%40 - 13;
      generate_cylinder(x,y);
      // printf("<model name='cylinder_%d'>\n\t<include>\n\t\t<uri>model://cylinder</uri>\n\t\t<pose>",num*5+ci);
      // printf("%.1f %.1f 1.5 0 0 0</pose>\n\t</include>\n</model>\n",x,y);
    }
    for(int qi = 0; qi < 6; qi++)
    {
      x = sign(num,4,0) * (qi*2 + 2);
      y = (num*4*2)%40 - 13 + sign(qi%2,0,0)*0.5;
      generate_cube(x,y);
      // printf("<model name='square_%d'>\n\t<include>\n\t\t<uri>model://square</uri>\n\t\t<pose>",num*6+qi);
      // printf("%.1f %.1f 0.2 0 0 0</pose>\n\t</include>\n</model>\n",x,y);
    }
    // format2
    for(int ci = 0; ci < 5; ci++)
    {
      x = sign(num,4,0) * (ci*2 + 3);
      y = (num*4*2)%40 - 14.5 + sign(ci,2,1)*(ci-2)*0.5;
      generate_cylinder(x,y);
      // printf("<model name='cylinder_%d'>\n\t<include>\n\t\t<uri>model://cylinder</uri>\n\t\t<pose>",num*5+ci+50);
      // printf("%.1f %.1f 1.5 0 0 0</pose>\n\t</include>\n</model>\n",x,y);
    }
    for(int qi = 0; qi < 6; qi++)
    {
      x = sign(num,4,0) * (qi*2 + 2);
      y = (num*4*2)%40 - 15 - sign(qi%2,0,0)*0.5;
      generate_cube(x,y);
      // printf("<model name='square_%d'>\n\t<include>\n\t\t<uri>model://square</uri>\n\t\t<pose>",num*6+qi+60);
      // printf("%.1f %.1f 0.2 0 0 0</pose>\n\t</include>\n</model>\n",x,y);
    }
    // format3
    for(int ci = 0; ci < 5; ci++)
    {
      x = sign(num,4,0) * (ci*2 + 3);
      y = (num*4*2)%40 - 17 + sign(ci%2,0,0)*0.5;
      generate_cylinder(x,y);
      // printf("<model name='cylinder_%d'>\n\t<include>\n\t\t<uri>model://cylinder</uri>\n\t\t<pose>",num*5+ci+100);
      // printf("%.1f %.1f 1.5 0 0 0</pose>\n\t</include>\n</model>\n",x,y);
    }
    for(int qi = 0; qi < 6; qi++)
    {
      x = sign(num,4,0) * (qi*2 + 2);
      y = (num*4*2)%40 - 17;
      generate_cube(x,y);
      // printf("<model name='square_%d'>\n\t<include>\n\t\t<uri>model://square</uri>\n\t\t<pose>",num*6+qi+120);
      // printf("%.1f %.1f 0.2 0 0 0</pose>\n\t</include>\n</model>\n",x,y);
    }
    // format4
    for(int ci = 0; ci < 5; ci++)
    {
      x = sign(num,4,0) * (ci*2 + 3);
      y = (num*4*2)%40 - 19 + (ci-2)/2*0.5;
      generate_cylinder(x,y);
      // printf("<model name='cylinder_%d'>\n\t<include>\n\t\t<uri>model://cylinder</uri>\n\t\t<pose>",num*5+ci+150);
      // printf("%.1f %.1f 1.5 0 0 0</pose>\n\t</include>\n</model>\n",x,y);
    }
    for(int qi = 0; qi < 6; qi++)
    {
      x = sign(num,4,0) * (qi*2 + 2);
      y = (num*4*2)%40 - 19 + sign(qi%2,0,0)*0.5;
      generate_cube(x,y);
      // printf("<model name='square_%d'>\n\t<include>\n\t\t<uri>model://square</uri>\n\t\t<pose>",num*6+qi+180);
      // printf("%.1f %.1f 0.2 0 0 0</pose>\n\t</include>\n</model>\n",x,y);
    }
  }
  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  ROS_INFO("\033[1;33;41m----> Finished generate CXY map 3.\033[0m");

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}

void MapGenerateCXY_4() 
{
  double x,y;
  for(int num = 0; num < 10; num++)
  {
    // format1
    for(int ci = 0; ci < 3; ci++)
    {
      x = sign(num,4,0) * (ci*4 + 4);
      y = (num*4*2)%40 - 13;
      generate_cylinder(x,y);
      // printf("<model name='cylinder_%d'>\n\t<include>\n\t\t<uri>model://cylinder</uri>\n\t\t<pose>",num*5+ci);
      // printf("%.1f %.1f 1.5 0 0 0</pose>\n\t</include>\n</model>\n",x,y);
    }
    for(int qi = 0; qi < 3; qi++)
    {
      x = sign(num,4,0) * (qi*4 + 2);
      y = (num*4*2)%40 - 13 + sign(qi%2,0,0)*0.5;
      generate_cube(x,y);
      // printf("<model name='square_%d'>\n\t<include>\n\t\t<uri>model://square</uri>\n\t\t<pose>",num*6+qi);
      // printf("%.1f %.1f 0.2 0 0 0</pose>\n\t</include>\n</model>\n",x,y);
    }
    // format2
    for(int ci = 0; ci < 3; ci++)
    {
      x = sign(num,4,0) * (ci*4 + 4);
      y = (num*4*2)%40 - 14.5 + sign(ci,2,1)*(ci-2)*0.5;
      generate_cylinder(x,y);
      // printf("<model name='cylinder_%d'>\n\t<include>\n\t\t<uri>model://cylinder</uri>\n\t\t<pose>",num*5+ci+50);
      // printf("%.1f %.1f 1.5 0 0 0</pose>\n\t</include>\n</model>\n",x,y);
    }
    for(int qi = 0; qi < 3; qi++)
    {
      x = sign(num,4,0) * (qi*4 + 2);
      y = (num*4*2)%40 - 15 - sign(qi%2,0,0)*0.5;
      generate_cube(x,y);
      // printf("<model name='square_%d'>\n\t<include>\n\t\t<uri>model://square</uri>\n\t\t<pose>",num*6+qi+60);
      // printf("%.1f %.1f 0.2 0 0 0</pose>\n\t</include>\n</model>\n",x,y);
    }
    // format3
    for(int ci = 0; ci < 3; ci++)
    {
      x = sign(num,4,0) * (ci*4 + 4);
      y = (num*4*2)%40 - 17 + sign(ci%2,0,0)*0.5;
      generate_cylinder(x,y);
      // printf("<model name='cylinder_%d'>\n\t<include>\n\t\t<uri>model://cylinder</uri>\n\t\t<pose>",num*5+ci+100);
      // printf("%.1f %.1f 1.5 0 0 0</pose>\n\t</include>\n</model>\n",x,y);
    }
    for(int qi = 0; qi < 3; qi++)
    {
      x = sign(num,4,0) * (qi*4 + 2);
      y = (num*4*2)%40 - 17;
      generate_cube(x,y);
      // printf("<model name='square_%d'>\n\t<include>\n\t\t<uri>model://square</uri>\n\t\t<pose>",num*6+qi+120);
      // printf("%.1f %.1f 0.2 0 0 0</pose>\n\t</include>\n</model>\n",x,y);
    }
    // format4
    for(int ci = 0; ci < 3; ci++)
    {
      x = sign(num,4,0) * (ci*4 + 4);
      y = (num*4*2)%40 - 19 + (ci-2)/2*0.5;
      generate_cylinder(x,y);
      // printf("<model name='cylinder_%d'>\n\t<include>\n\t\t<uri>model://cylinder</uri>\n\t\t<pose>",num*5+ci+150);
      // printf("%.1f %.1f 1.5 0 0 0</pose>\n\t</include>\n</model>\n",x,y);
    }
    for(int qi = 0; qi < 3; qi++)
    {
      x = sign(num,4,0) * (qi*4 + 2);
      y = (num*4*2)%40 - 19 + sign(qi%2,0,0)*0.5;
      generate_cube(x,y);
      // printf("<model name='square_%d'>\n\t<include>\n\t\t<uri>model://square</uri>\n\t\t<pose>",num*6+qi+180);
      // printf("%.1f %.1f 0.2 0 0 0</pose>\n\t</include>\n</model>\n",x,y);
    }
  }
  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  ROS_INFO("\033[1;33;41m----> Finished generate CXY map 4.\033[0m");

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}

void RandomMapGenerateCXY() 
{
  // 待存入全局点云的点
  pcl::PointXYZ pt_random;

  // 障碍物位置容器
  vector<Eigen::Vector2d> obs_position;

  // 将无人机初始位置存入容器（防止在无人机初始位置周围生成障碍物）
  obs_position.push_back( Eigen::Vector2d(_init_x,_init_y) );

  // 障碍物位置随机生成器
  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);

  // 生成圆柱体障碍物
  for (int i = 0; i < cylinder_num && ros::ok(); i++) 
  {
    double x, y;
    // 随机生成障碍物位置：[x,y]
    x = rand_x(eng);
    y = rand_y(eng);
    
    // 检查是否与已经生成的障碍物重叠：两个障碍物之间距离大于 _min_dist，否则生成失败
    bool flag_continue = false;
    for ( auto p : obs_position )
      if ( (Eigen::Vector2d(x,y) - p).norm() < _min_dist /*metres*/ )
      {
        i--; // i--代表此次生成失败，重新生成障碍物
        flag_continue = true;
        break;
      }
    if ( flag_continue ) continue;

    // 将本次生成的障碍物位置推入容器
    obs_position.push_back( Eigen::Vector2d(x,y) );
    
    // floor 是向下取整函数
    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    int widNum = ceil((cylinder_radius) / _resolution);
    int heiNum = ceil(cylinder_height / _resolution);

    // 生成圆柱体的点云
    for (int r = -widNum; r < widNum; r++)
      for (int s = -widNum; s < widNum; s++) {
        for (int t = -1.0; t < heiNum; t++) {
          double temp_x = x + (r + 0.5) * _resolution + 1e-2;
          double temp_y = y + (s + 0.5) * _resolution + 1e-2;
          double temp_z = (t + 0.5) * _resolution + 1e-2;
          if ( (Eigen::Vector2d(temp_x,temp_y) - Eigen::Vector2d(x,y)).norm() <= cylinder_radius )
          {
            pt_random.x = temp_x;
            pt_random.y = temp_y;
            pt_random.z = temp_z;
            cloudMap.points.push_back(pt_random);
          }
        }
      }
  }

  // 生成正方形障碍物
  for (int i = 0; i < sqaure_num; ++i) 
  {
    double x, y, z;
    // 随机生成障碍物位置：[x,y]
    x = rand_x(eng);
    y = rand_y(eng);
    z = sqaure_size / 2;

    // 检查是否与已经生成的障碍物重叠：两个障碍物之间距离大于 _min_dist，否则生成失败
    bool flag_continue = false;
    for ( auto p : obs_position )
      if ( (Eigen::Vector2d(x,y) - p).norm() < _min_dist /*metres*/ )
      {
        i--; // i--代表此次生成失败，重新生成障碍物
        flag_continue = true;
        break;
      }
    if ( flag_continue ) continue;

    // 将本次生成的障碍物位置推入容器
    obs_position.push_back( Eigen::Vector2d(x,y) );

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    int widNum = ceil((sqaure_size) / _resolution);

    // 生成正方体的点云
    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
        for (int t = -widNum / 2.0; t < widNum / 2.0; t++) {
          double temp_x = x + (r + 0.5) * _resolution + 1e-2;
          double temp_y = y + (s + 0.5) * _resolution + 1e-2;
          double temp_z = z + (t + 0.5) * _resolution + 1e-2;
          pt_random.x = temp_x;
          pt_random.y = temp_y;
          pt_random.z = temp_z;
          cloudMap.points.push_back(pt_random);
        }
      }
  }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  ROS_INFO("\033[1;33;41m----> Finished generate random map.\033[0m");

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}

void rcvOdometryCallbck(const nav_msgs::Odometry::ConstPtr& odom, int uav_id) 
{
  _has_odom[uav_id] = true;

  _odom[uav_id] = *odom;

  _state = {odom->pose.pose.position.x,
            odom->pose.pose.position.y,
            odom->pose.pose.position.z,
            odom->twist.twist.linear.x,
            odom->twist.twist.linear.y,
            odom->twist.twist.linear.z,
            0.0,
            0.0,
            0.0};
}

void rcvUGVOdometryCallbck(const nav_msgs::Odometry::ConstPtr& odom, int ugv_id) 
{
  _has_odom_ugv[ugv_id] = true;

  _odom_ugv[ugv_id] = *odom;

  _state = {odom->pose.pose.position.x,
            odom->pose.pose.position.y,
            odom->pose.pose.position.z,
            odom->twist.twist.linear.x,
            odom->twist.twist.linear.y,
            odom->twist.twist.linear.z,
            0.0,
            0.0,
            0.0};
}

void pubGlobalPoints(const ros::TimerEvent& event) 
{
  if(_map_ok)
  {
    // 发布全局点云
    pcl::toROSMsg(cloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = "world";
    _all_map_pub.publish(globalMap_pcd);
  }else
  {
    return;
  }
}

void renderSensedPoints(const ros::TimerEvent& event, int uav_id) 
{
  if (!_map_ok || !_has_odom[uav_id]) return;

  Eigen::Quaterniond q;
  q.x() = _odom[uav_id].pose.pose.orientation.x;
  q.y() = _odom[uav_id].pose.pose.orientation.y;
  q.z() = _odom[uav_id].pose.pose.orientation.z;
  q.w() = _odom[uav_id].pose.pose.orientation.w;

  Eigen::Matrix3d rot;
  rot = q;
  Eigen::Vector3d yaw_vec = rot.col(0);

  local_map.points.clear();
  pcl::PointXYZ searchPoint(_odom[uav_id].pose.pose.position.x,
                            _odom[uav_id].pose.pose.position.y,
                            _odom[uav_id].pose.pose.position.z);
  pointIdxRadiusSearch.clear();
  pointRadiusSquaredDistance.clear();

  pcl::PointXYZ pt;
  if (kdtreeLocalMap.radiusSearch(searchPoint, sensing_horizon,
                                   pointIdxRadiusSearch,
                                   pointRadiusSquaredDistance) > 0) {
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
      pt = cloudMap.points[pointIdxRadiusSearch[i]];

      // if ((fabs(pt.z - _odom.pose.pose.position.z) / (pt.x - _odom.pose.pose.position.x)) >
      //     tan(M_PI / 12.0))
      //   continue;
      if ((fabs(pt.z - _odom[uav_id].pose.pose.position.z) / sensing_horizon) >
          tan(M_PI / 6.0))
        continue; 

      Vector3d pt_vec(pt.x - _odom[uav_id].pose.pose.position.x,
                      pt.y - _odom[uav_id].pose.pose.position.y,
                      pt.z - _odom[uav_id].pose.pose.position.z);

      if (pt_vec.normalized().dot(yaw_vec) < 0.5) continue; 
      
      local_map.points.push_back(pt);
    }
  } else {
    return;
  }

  local_map.width = local_map.points.size();
  local_map.height = 1;
  local_map.is_dense = true;

  pcl::toROSMsg(local_map, localMap_pcd);
  localMap_pcd.header.frame_id = "/world";

  pub_cloud[uav_id].publish(localMap_pcd);
}

void renderSensedPointsUGV(const ros::TimerEvent& event, int ugv_id) 
{
  if (!_map_ok || !_has_odom_ugv[ugv_id]) return;

  Eigen::Quaterniond q;
  q.x() = _odom_ugv[ugv_id].pose.pose.orientation.x;
  q.y() = _odom_ugv[ugv_id].pose.pose.orientation.y;
  q.z() = _odom_ugv[ugv_id].pose.pose.orientation.z;
  q.w() = _odom_ugv[ugv_id].pose.pose.orientation.w;

  Eigen::Matrix3d rot;
  rot = q;
  Eigen::Vector3d yaw_vec = rot.col(0);

  local_map.points.clear();
  pcl::PointXYZ searchPoint(_odom_ugv[ugv_id].pose.pose.position.x,
                            _odom_ugv[ugv_id].pose.pose.position.y,
                            _odom_ugv[ugv_id].pose.pose.position.z);
  pointIdxRadiusSearch.clear();
  pointRadiusSquaredDistance.clear();

  pcl::PointXYZ pt;
  if (kdtreeLocalMap.radiusSearch(searchPoint, sensing_horizon,
                                   pointIdxRadiusSearch,
                                   pointRadiusSquaredDistance) > 0) {
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
      if(abs(pt.z - _odom_ugv[ugv_id].pose.pose.position.z)>5*_resolution+1e-2) continue; //qi comment
      pt = cloudMap.points[pointIdxRadiusSearch[i]];
      // pt.z = _odom_ugv[ugv_id].pose.pose.position.z; //qi comment

      // if ((fabs(pt.z - _odom_ugv.pose.pose.position.z) / (pt.x - _odom_ugv.pose.pose.position.x)) >
      //     tan(M_PI / 12.0))
      //   continue;
      if ((fabs(pt.z - _odom_ugv[ugv_id].pose.pose.position.z) / sensing_horizon) >
          tan(M_PI / 6.0))
        continue; 

      Vector3d pt_vec(pt.x - _odom_ugv[ugv_id].pose.pose.position.x,
                      pt.y - _odom_ugv[ugv_id].pose.pose.position.y,
                      pt.z - _odom_ugv[ugv_id].pose.pose.position.z);

      if (pt_vec.normalized().dot(yaw_vec) < 0.5) continue; 
      
      local_map.points.push_back(pt);
    }
  } else {
    return;
  }

  local_map.width = local_map.points.size();
  local_map.height = 1;
  local_map.is_dense = true;

  pcl::toROSMsg(local_map, localMap_pcd);
  localMap_pcd.header.frame_id =  "/world";
 
  pub_cloud_ugv[ugv_id].publish(localMap_pcd);
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "random_map_sensing_cxy");
  ros::NodeHandle n("~");

  n.param("swarm_num", swarm_num, 0);
  n.param("swarm_num_ugv", swarm_num_ugv, 0);
  // 0代表随机生成地图，1使用1号地图，2使用2号地图
  n.param("map_generator/map_flag", map_flag, 0);
  // 地图尺寸
  n.param("map_generator/x_size", _x_size, 30.0);
  n.param("map_generator/y_size", _y_size, 30.0);
  n.param("map_generator/z_size", _z_size, 2.0);
  // 点云分辨率
  n.param("map_generator/resolution", _resolution, 0.1);
  // 障碍物间最小距离
  n.param("map_generator/min_distance", _min_dist, 1.0);
  // 障碍物参数
  n.param("map_generator/cylinder_num", cylinder_num, 30);
  n.param("map_generator/sqaure_num", sqaure_num, 30);
  n.param("map_generator/cylinder_radius", cylinder_radius, 0.2);
  n.param("map_generator/cylinder_height", cylinder_height, 3.0);
  n.param("map_generator/sqaure_size", sqaure_size, 0.25);
  // 无人机初始位置
  n.param("init_state_x", _init_x, 0.0);
  n.param("init_state_y", _init_y, 0.0);
  // 感知范围与感知频率
  n.param("map_generator/sensing_range", _sensing_range, 10.0);
  n.param("map_generator/sensing_horizon", sensing_horizon, 5.0);
  n.param("map_generator/sense_rate", _sense_rate, 10.0);

  _x_l = -_x_size / 2.0;
  _x_h = +_x_size / 2.0;

  _y_l = -_y_size / 2.0;
  _y_h = +_y_size / 2.0;

  if( (cylinder_num + sqaure_num) > _x_size*8)
  {
    ROS_ERROR("The map can't put all the obstacles, remove some.");
    cylinder_num = _x_size*4;
    sqaure_num = _x_size*4;
  }

  // 发布 全局地图
  _all_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);
  global_pcl_timer = n.createTimer(ros::Duration(10.0/_sense_rate), pubGlobalPoints);

  // 无人机
  for(int i = 1 ; i <= swarm_num; i++)
  {
    uav_id[i] = i;
    uav_name[i] = "/uav"+std::to_string(i);
    pub_cloud[i] = n.advertise<sensor_msgs::PointCloud2>("/uav"+std::to_string(i)+"/map_generator/local_cloud", 1);
    _odom_sub[i] = n.subscribe<nav_msgs::Odometry>("/uav"+std::to_string(i)+"/prometheus/drone_odom", 10, boost::bind(&rcvOdometryCallbck,_1,uav_id[i]));
    local_sensing_timer[i] = n.createTimer(ros::Duration(1.0/_sense_rate), boost::bind(&renderSensedPoints,_1,uav_id[i]));
  }

  // 无人车
  for(int i = 1 ; i <= swarm_num_ugv; i++)
  {
    ugv_id[i] = i;
    ugv_name[i] = "/ugv"+std::to_string(i);
    pub_cloud_ugv[i] = n.advertise<sensor_msgs::PointCloud2>("/ugv"+std::to_string(i)+"/map_generator/local_cloud", 1);
    _odom_ugv_sub[i] = n.subscribe<nav_msgs::Odometry>("/ugv"+std::to_string(i)+"/odom", 10, boost::bind(&rcvUGVOdometryCallbck,_1,ugv_id[i]));
    local_sensing_ugv_timer[i] = n.createTimer(ros::Duration(1.0/_sense_rate), boost::bind(&renderSensedPointsUGV,_1,ugv_id[i]));
  }

  unsigned int seed = rd();
  // unsigned int seed = 2433201515;
  cout << "seed=" << seed << endl;
  eng.seed(seed);

  if(map_flag == 0) RandomMapGenerateCXY();
  else if(map_flag == 1) MapGenerateCXY_1();
  else if(map_flag == 2) MapGenerateCXY_2();
  else if(map_flag == 3) MapGenerateCXY_3();
  else if(map_flag == 4) MapGenerateCXY_4();
  else ROS_ERROR("[Map server] wrong map_flag.");
  
  ros::Rate loop_rate(100.0);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}