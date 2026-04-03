#include <ros/ros.h>
#include "map_generator.h"

//  主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_generator_node");
    ros::NodeHandle nh("~");

    string map_name;
    // 【参数】地图名称
    nh.param<string>("map_name", map_name, "planning_test");
    cout << GREEN << "map_name:  [ " << map_name << " ]" << TAIL << endl;

    // 初始化地图生成器
    Map_Generator Obs_Map;
    Obs_Map.init(nh);

    // 生成边界
    Obs_Map.GenerateBorder();
    // Obs_Map.generate_square(3.0, 3.0);

    if (map_name == "planning_test")
    {
        // 对应 planning_test.world
        Obs_Map.GeneratePlanningTestMap();
    }
    else if (map_name == "planning_test2")
    {
        // 对应 planning_test2.world
        Obs_Map.GeneratePlanningTestMap2();
    }else if (map_name == "planning_test3")
    {
        // 对应 planning_test3.world
        Obs_Map.GeneratePlanningTestMap3();
    }
    else if (map_name == "random")
    {
        // 生成随机地图
        Obs_Map.GenerateRandomMap();
    }
    else if (map_name == "test")
    {
        // 生成示例地图，地图元素包括cylinder、square、row_wall、column_wall、line
        // small_cylinder对应obs_cylinder_small.sdf
        // large_cylinder对应obs_cylinder_large.sdf
        // square对应obs_square.sdf
        // 墙体暂时没有对应的Gazebo模型，todo
        Obs_Map.generate_square(3.0, 3.0);
        Obs_Map.generate_small_cylinder(1.0, 1.0);
        Obs_Map.generate_large_cylinder(2.0, 2.0);
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
    else
    {
        cout << RED << "[map_generator] wrong map_name:   [ " << map_name << " ]" << TAIL << endl;
    }

    ros::spin();

    return 0;
}