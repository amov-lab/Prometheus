#include <ros/ros.h>
#include "map_generator.h"

//  主函数
int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_generator_node");
  ros::NodeHandle nh("~");

  int map_flag;
  // 【参数】0代表随机生成地图，1使用1号地图，2使用2号地图
  nh.param("map_flag", map_flag, 1);

  // 初始化地图生成器
  Map_Generator Obs_Map;
  Obs_Map.init(nh);

  // 生成边界
  Obs_Map.GenerateBorder();

  if (map_flag == 0)
  {
    // 生成随机地图
    Obs_Map.GenerateRandomMap();
  }
  else if (map_flag == 1)
  {
    // 生成固定地图
    Obs_Map.GenerateMap1();
  }
  else if (map_flag == 2)
  {
    // 生成自定义地图
    Obs_Map.generate_cylinder(1.0, 1.0);

    Obs_Map.global_map_pcl.width = Obs_Map.global_map_pcl.points.size();
    Obs_Map.global_map_pcl.height = 1;
    Obs_Map.global_map_pcl.is_dense = true;
    Obs_Map.kdtreeLocalMap.setInputCloud(Obs_Map.global_map_pcl.makeShared());
    Obs_Map.global_map_ok = true;
    cout << GREEN << "[map_generator] Finished generate map 2. Map points:" << Obs_Map.global_map_pcl.width << TAIL << endl;
  }
  else
  {
    cout << RED << "[map_generator] wrong map_flag." << TAIL << endl;
  }

  ros::spin();

  return 0;
}