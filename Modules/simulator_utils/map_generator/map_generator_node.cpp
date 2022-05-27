#include <ros/ros.h>
#include "map_generator.h"

//  主函数
int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_generator_node");
  ros::NodeHandle nh("~");

  string map_name;
  // 【参数】地图名称
  nh.param<string>("map_name", map_name, "random");
  cout << GREEN << "map_name:  [ " << map_name << " ]" << TAIL << endl;

  // 初始化地图生成器
  Map_Generator Obs_Map;
  Obs_Map.init(nh);

  // 生成边界
  Obs_Map.GenerateBorder();

  if (map_name == "random")
  {
    // 生成随机地图
    Obs_Map.GenerateRandomMap();
  }
  else if (map_name == "test")
  {
    // 生成示例地图，地图元素包括cylinder、square、row_wall、column_wall、line
    // cylinder对应obs_cylinder.sdf
    // square对应obs_square.sdf
    // 墙体暂时没有对应，todo
    Obs_Map.generate_cylinder(1.0, 1.0);
    Obs_Map.generate_square(2.0, 2.0);
    Obs_Map.generate_row_wall(10.0, 10.0);
    Obs_Map.generate_column_wall(-10.0, -10.0);
    Obs_Map.generate_line(0.0, 0.0);
    Obs_Map.global_map_pcl.width = Obs_Map.global_map_pcl.points.size();
    Obs_Map.global_map_pcl.height = 1;
    Obs_Map.global_map_pcl.is_dense = true;
    Obs_Map.kdtreeLocalMap.setInputCloud(Obs_Map.global_map_pcl.makeShared());
    Obs_Map.global_map_ok = true;
    cout << GREEN << "[map_generator] Finished generate map [ " << map_name << " ]. Map points:" << Obs_Map.global_map_pcl.width << TAIL << endl;
  }
  else if (map_name == "motion_planning")
  {
    // 对应 motion_planning.world
    // 第一排
    for (int t = -8; t <= 8; t = t + 2)
    {
      Obs_Map.generate_cylinder((double)8, (double)t);
    }

    // 第二排
    for (int t = -9; t <= 7; t = t + 2)
    {
      Obs_Map.generate_cylinder((double)6, (double)t);
    }

    for (int t = -8; t <= 8; t = t + 2)
    {
      Obs_Map.generate_cylinder((double)4, (double)t);
    }

    for (int t = -9; t <= 7; t = t + 2)
    {
      Obs_Map.generate_cylinder((double)2, (double)t);
    }

    for (int t = -8; t <= 8; t = t + 2)
    {
      Obs_Map.generate_cylinder((double)0, (double)t);
    }

    for (int t = -9; t <= 7; t = t + 2)
    {
      Obs_Map.generate_cylinder((double)-2, (double)t);
    }

    for (int t = -8; t <= 8; t = t + 2)
    {
      Obs_Map.generate_cylinder((double)-4, (double)t);
    }

    for (int t = -9; t <= 7; t = t + 2)
    {
      Obs_Map.generate_cylinder((double)-6, (double)t);
    }

    for (int t = -8; t <= 8; t = t + 2)
    {
      Obs_Map.generate_cylinder((double)-8, (double)t);
    }

    Obs_Map.global_map_pcl.width = Obs_Map.global_map_pcl.points.size();
    Obs_Map.global_map_pcl.height = 1;
    Obs_Map.global_map_pcl.is_dense = true;
    Obs_Map.kdtreeLocalMap.setInputCloud(Obs_Map.global_map_pcl.makeShared());
    Obs_Map.global_map_ok = true;
    cout << GREEN << "[map_generator] Finished generate map [ " << map_name << " ]. Map points:" << Obs_Map.global_map_pcl.width << TAIL << endl;
  }
  else
  {
    cout << RED << "[map_generator] wrong map_name:   [ " << map_name << " ]"<< TAIL << endl;
  }

  ros::spin();

  return 0;
}