#include "map_generator.h"

//  主函数
void Map_Generator::init(ros::NodeHandle &nh)
{
  // 【参数】集群数量
  nh.param("swarm_num", swarm_num, 0);
  // 【参数】地图尺寸、分辨率
  nh.param("map_generator/x_size", map_size_x, 30.0);
  nh.param("map_generator/y_size", map_size_y, 30.0);
  nh.param("map_generator/z_size", map_size_z, 5.0);
  nh.param("map_generator/resolution", map_resolution, 0.1);
  // 【参数】传感器感知半径与感知频率
  nh.param("map_generator/sensing_range", sensing_range, 10.0);
  nh.param("map_generator/sensing_horizon", sensing_horizon, 5.0);
  nh.param("map_generator/sensing_rate", sensing_rate, 10.0);
  // 【参数】障碍物最小距离
  nh.param("map_generator/obs_min_dist", obs_min_dist, 2.0);
  // 【参数】障碍物数量 - 圆柱
  nh.param("map_generator/cylinder_num", cylinder_num, 20);
  // 【参数】障碍物参数 - 圆柱
  nh.param("map_generator/cylinder_radius", cylinder_radius, 0.25);
  nh.param("map_generator/cylinder_height", cylinder_height, 2.0);
  // 【参数】障碍物数量 - 立方体
  nh.param("map_generator/square_num", square_num, 10);
  // 【参数】障碍物参数 - 立方体
  nh.param("map_generator/square_size", square_size, 0.5);
  nh.param("map_generator/square_height", square_height, 2.0);
  // 【参数】障碍物参数 - 墙
  nh.param("map_generator/wall_length", wall_length, 10.0);
  nh.param("map_generator/wall_height", wall_height, 2.0);
  nh.param("map_generator/line_height", line_height, 2.0);
  // 【参数】初始位置
  nh.param("uav_init_x", uav_init_x, -100.0);
  nh.param("uav_init_y", uav_init_y, -100.0);

  // 地图坐标范围 [map_x_min,  map_x_max] - [map_y_min, map_y_max]
  map_x_min = -map_size_x / 2.0;
  map_x_max = +map_size_x / 2.0;
  map_y_min = -map_size_y / 2.0;
  map_y_max = +map_size_y / 2.0;
  map_z_limit = map_size_z;

  cout << GREEN << "Map INFO:" << TAIL << endl;
  cout << GREEN << "X:  [" << map_x_min << "," << map_x_max << "]" << TAIL << endl;
  cout << GREEN << "Y:  [" << map_y_min << "," << map_y_max << "]" << TAIL << endl;
  cout << GREEN << "Z:  [ 0,"<< map_z_limit  << "]" << TAIL << endl;

  if ((cylinder_num + square_num) > map_size_x * 8)
  {
    cout << RED << "[map_generator] The map can't put all the obstacles, remove some." << TAIL << endl;
    cylinder_num = map_size_x * 4;
    square_num = map_size_x * 4;
  }

  for (int i = 1; i <= swarm_num; i++)
  {
    uav_id[i] = i;
    // 【订阅】里程计数据
    uav_odom_sub[i] = nh.subscribe<nav_msgs::Odometry>("/uav" + std::to_string(i) + "/fake_odom", 1, boost::bind(&Map_Generator::uav_odom_cb,this, _1, uav_id[i]));
    // 【发布】模拟的局部点云信息
    local_map_pub[i] = nh.advertise<sensor_msgs::PointCloud2>("/uav" + std::to_string(i) + "/map_generator/local_cloud", 1);
    // 【定时器】发布局部点云定时器
    pub_local_map_timer[i] = nh.createTimer(ros::Duration(1.0 / sensing_rate), boost::bind(&Map_Generator::pub_local_map_cb,this, _1, uav_id[i]));
  }

  // 【发布】全局点云地图
  global_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);
  // 【定时器】发布全局点云定时器
  pub_global_map_timer = nh.createTimer(ros::Duration(10.0 / sensing_rate), &Map_Generator::pub_global_map_cb,this);

  ros::Duration(1.0).sleep();
  
  //  随机函数初始化
  unsigned int seed = rd();
  // unsigned int seed = 2433201515;
  cout << "seed=" << seed << endl;
  eng.seed(seed);
}


// 生成圆柱体
// 圆柱体半径：cylinder_radius
// 圆柱体高度：cylinder_height
void Map_Generator::generate_cylinder(double x, double y)
{
  // 必须在地图范围内
  if (x < map_x_min || x > map_x_max || y < map_y_min || y > map_y_max)
  {
    cout << RED << "generate_cylinder wrong." << TAIL << endl;
    return;
  }

  pcl::PointXYZ pt_random;
  x = floor(x / map_resolution) * map_resolution;
  y = floor(y / map_resolution) * map_resolution;
  int widNum = ceil((cylinder_radius) / map_resolution);
  int heiNum = ceil(cylinder_height / map_resolution);
  for (int r = -widNum; r <= widNum; r++)
    for (int s = -widNum; s <= widNum; s++)
      for (int t = -1; t <= heiNum; t++)
      {
        double temp_x = x + r * map_resolution;
        double temp_y = y + s * map_resolution;
        double temp_z = t * map_resolution;
        if ((Eigen::Vector2d(temp_x, temp_y) - Eigen::Vector2d(x, y)).norm() < cylinder_radius + 1e-2)
        {
          pt_random.x = temp_x;
          pt_random.y = temp_y;
          pt_random.z = temp_z;
          global_map_pcl.points.push_back(pt_random);
        }
      }
}

// 生成长方体
// 长方体长宽：square_height
// 长方体高度：square_height
void Map_Generator::generate_square(double x, double y)
{
  // 必须在地图范围内
  if (x < map_x_min || x > map_x_max || y < map_y_min || y > map_y_max)
  {
    cout << RED << "generate_square wrong." << TAIL << endl;
    return;
  }

  pcl::PointXYZ pt_random;
  x = floor(x / map_resolution) * map_resolution;
  y = floor(y / map_resolution) * map_resolution;
  // z = floor(z / map_resolution) * map_resolution + map_resolution / 2.0;
  int widNum = ceil((square_size / 2.0) / map_resolution);
  int heiNum = ceil(square_height / map_resolution);
  int temp_wid = square_size / 2.0 / map_resolution;
  int flag = 0;
  if (widNum != temp_wid)
    flag = 1;
  for (int r = -widNum; r <= widNum; r++)
    for (int s = -widNum; s <= widNum; s++)
      for (int t = -1; t <= heiNum; t++)
      {
        double temp_x = x + r * map_resolution + flag * 0.5 * map_resolution;
        double temp_y = y + s * map_resolution + flag * 0.5 * map_resolution;
        double temp_z = t * map_resolution;
        if ((Eigen::Vector2d(temp_x, temp_y) - Eigen::Vector2d(x, y)).norm() <= square_size / 2.0 * sqrt(2) + 1e-1)
        {
          pt_random.x = temp_x;
          pt_random.y = temp_y;
          pt_random.z = temp_z;
          global_map_pcl.points.push_back(pt_random);
        }
      }
}

// 生成横墙，即与X轴平行的墙
// 横墙长度：wall_length
// 横墙高度：wall_height
void Map_Generator::generate_row_wall(double x, double y)
{
  // 必须在地图范围内
  if (x < map_x_min || x > map_x_max || y < map_y_min || y > map_y_max)
  {
    cout << RED << "generate_row_wall wrong." << TAIL << endl;
    return;
  }

  pcl::PointXYZ pt_random;
  int widNum = ceil((wall_length / 2.0) / map_resolution);
  int heiNum = ceil(wall_height / map_resolution);

  for (int r = -widNum; r <= widNum; r++)
    for (int t = -1; t <= heiNum; t++)
    {
      double temp_x = x + r * map_resolution;
      double temp_y = y;
      double temp_z = t * map_resolution;
      pt_random.x = temp_x;
      pt_random.y = temp_y;
      pt_random.z = temp_z;
      global_map_pcl.points.push_back(pt_random);
    }
}

// 生成竖墙，即与Y轴平行的墙
// 横墙长度：wall_length
// 横墙高度：wall_height
void Map_Generator::generate_column_wall(double x, double y)
{
  // 必须在地图范围内
  if (x < map_x_min || x > map_x_max || y < map_y_min || y > map_y_max)
  {
    cout << RED << "generate_column_wall wrong." << TAIL << endl;
    return;
  }

  pcl::PointXYZ pt_random;
  int widNum = ceil((wall_length / 2.0) / map_resolution);
  int heiNum = ceil(wall_height / map_resolution);


  for (int r = -widNum; r <= widNum; r++)
    for (int t = -1; t < heiNum; t++)
    {
      double temp_x = x;
      double temp_y = y + r * map_resolution;
      double temp_z = t * map_resolution;
      pt_random.x = temp_x;
      pt_random.y = temp_y;
      pt_random.z = temp_z;
      global_map_pcl.points.push_back(pt_random);
    }
}

// 生成一根竖直的线
// 长度：line_height
void Map_Generator::generate_line(double x, double y)
{
  // 必须在地图范围内
  if (x < map_x_min || x > map_x_max || y < map_y_min || y > map_y_max)
  {
    cout << RED << "generate_line wrong." << TAIL << endl;
    return;
  }

  pcl::PointXYZ pt_random;
  int heiNum = ceil(line_height / map_resolution);

  for (int t = -1; t < heiNum; t++)
  {
    double temp_x = x;
    double temp_y = y;
    double temp_z = t * map_resolution;
    pt_random.x = temp_x;
    pt_random.y = temp_y;
    pt_random.z = temp_z;
    global_map_pcl.points.push_back(pt_random);
  }
}

void Map_Generator::GenerateBorder()
{
  wall_height = 0.1;

  wall_length = map_size_x;
  generate_row_wall(0.0, map_y_min);
  generate_row_wall(0.0, map_y_max);
  wall_length = map_size_y;
  generate_column_wall(map_x_min, 0.0);
  generate_column_wall(map_x_max, 0.0);

  cout << GREEN << "[map_generator] Finished generate border." << TAIL << endl;
}

void Map_Generator::GenerateMap1()
{
  generate_cylinder(1.0, 1.0);

  generate_square(2.0, 2.0);

  generate_row_wall(10.0, 10.0);

  generate_column_wall(-10.0, -10.0);

  generate_line(0.0, 0.0);

  global_map_pcl.width = global_map_pcl.points.size();

  
  global_map_pcl.height = 1;
  global_map_pcl.is_dense = true;

  kdtreeLocalMap.setInputCloud(global_map_pcl.makeShared());

  global_map_ok = true;

  cout << GREEN << "[map_generator] Finished generate map 1. Map points:" << global_map_pcl.width<< TAIL << endl;
}

void Map_Generator::GenerateRandomMap()
{
  // 待存入全局点云的点
  pcl::PointXYZ pt_random;
  // 障碍物位置容器
  vector<Eigen::Vector2d> obs_position;
  // 将无人机初始位置存入容器（防止在无人机初始位置周围生成障碍物）
  obs_position.push_back(Eigen::Vector2d(uav_init_x, uav_init_y));

  // 障碍物位置随机生成器，参数为地图尺寸
  rand_x = uniform_real_distribution<double>(map_x_min, map_x_max);
  rand_y = uniform_real_distribution<double>(map_y_min, map_y_max);

  // 生成圆柱体障碍物
  for (int i = 0; i < cylinder_num; i++)
  {
    double x, y;
    // 随机生成障碍物位置：[x,y]
    x = rand_x(eng);
    y = rand_y(eng);

    // 检查是否与已经生成的障碍物重叠：两个障碍物之间距离大于 _min_dist，否则生成失败
    bool flag_continue = false;
    for (auto p : obs_position)
      if ((Eigen::Vector2d(x, y) - p).norm() < obs_min_dist)
      {
        i--;
        flag_continue = true;
        break;
      }
    if (flag_continue)
      continue;

    // 将本次生成的障碍物位置推入容器
    obs_position.push_back(Eigen::Vector2d(x, y));

    generate_cylinder(x, y);
  }

  // 生成长方体障碍物 
  for (int i = 0; i < square_num; ++i)
  {
    double x, y, z;
    // 随机生成障碍物位置：[x,y]
    x = rand_x(eng);
    y = rand_y(eng);

    // 检查是否与已经生成的障碍物重叠：两个障碍物之间距离大于 _min_dist，否则生成失败
    bool flag_continue = false;
    for (auto p : obs_position)
      if ((Eigen::Vector2d(x, y) - p).norm() < obs_min_dist /*metres*/)
      {
        i--; // i--代表此次生成失败，重新生成障碍物
        flag_continue = true;
        break;
      }
    if (flag_continue)
      continue;

    // 将本次生成的障碍物位置推入容器
    obs_position.push_back(Eigen::Vector2d(x, y));

    generate_square(x, y);
  }

  global_map_pcl.width = global_map_pcl.points.size();
  global_map_pcl.height = 1;
  global_map_pcl.is_dense = true;
  kdtreeLocalMap.setInputCloud(global_map_pcl.makeShared());
  global_map_ok = true;

  cout << GREEN << "[map_generator] Finished generate random map. Map points:" << global_map_pcl.width<< TAIL << endl;
}

void Map_Generator::uav_odom_cb(const nav_msgs::Odometry::ConstPtr &odom, int uav_id)
{
  uav_odom_ok[uav_id] = true;

  uav_odom[uav_id] = *odom;
}

void Map_Generator::pub_global_map_cb(const ros::TimerEvent &event)
{
  if (global_map_ok)
  {
    pcl::toROSMsg(global_map_pcl, global_map_ros);
    global_map_ros.header.frame_id = "world";
    global_map_pub.publish(global_map_ros);
  }
  else
  {
    return;
  }
}

void Map_Generator::pub_local_map_cb(const ros::TimerEvent &event, int uav_id)
{
  if (!global_map_ok || !uav_odom_ok[uav_id])
  {
    // cout << RED << "[map_generator] fail to pub local map." << TAIL << endl;
    return;
  }

  // 读取无人机姿态
  Eigen::Quaterniond q;
  q.x() = uav_odom[uav_id].pose.pose.orientation.x;
  q.y() = uav_odom[uav_id].pose.pose.orientation.y;
  q.z() = uav_odom[uav_id].pose.pose.orientation.z;
  q.w() = uav_odom[uav_id].pose.pose.orientation.w;

  // 读取偏航角
  Eigen::Matrix3d rot;
  rot = q;
  Eigen::Vector3d yaw_vec = rot.col(0);

  local_map_pcl.points.clear();
  pcl::PointXYZ searchPoint(uav_odom[uav_id].pose.pose.position.x,
                            uav_odom[uav_id].pose.pose.position.y,
                            uav_odom[uav_id].pose.pose.position.z);
  pointIdxRadiusSearch.clear();
  pointRadiusSquaredDistance.clear();

  pcl::PointXYZ pt;
  if (kdtreeLocalMap.radiusSearch(searchPoint, sensing_horizon,
                                  pointIdxRadiusSearch,
                                  pointRadiusSquaredDistance) > 0)
  {
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
    {
      pt = global_map_pcl.points[pointIdxRadiusSearch[i]];

      if ((fabs(pt.z - uav_odom[uav_id].pose.pose.position.z) / sensing_horizon) >
          tan(M_PI / 6.0))
        continue;

      Vector3d pt_vec(pt.x - uav_odom[uav_id].pose.pose.position.x,
                      pt.y - uav_odom[uav_id].pose.pose.position.y,
                      pt.z - uav_odom[uav_id].pose.pose.position.z);

      // if (pt_vec.normalized().dot(yaw_vec) < 0.5) continue;

      local_map_pcl.points.push_back(pt);
    }
  }

  local_map_pcl.width = local_map_pcl.points.size();
  local_map_pcl.height = 1;
  local_map_pcl.is_dense = true;

  pcl::toROSMsg(local_map_pcl, local_map_ros);
  local_map_ros.header.frame_id = "/world";

  local_map_pub[uav_id].publish(local_map_ros);
}


