#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include "printf_utils.h"

#define MAX_SWRAM_NUM 41
ros::Publisher planner_goal_pub[MAX_SWRAM_NUM];

double target[10][3];

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ego_goal_pub");
    ros::NodeHandle nh("~");

    int swarm_num;
    // 集群数量
    nh.param("swarm_num", swarm_num, 1);
 
    string uav_name;
    for (int i = 1; i <= swarm_num; i++)
    {
      nh.param("uav" + to_string(i) + "/target_x", target[i][0], -1.0);
      nh.param("uav" + to_string(i) + "/target_y", target[i][1], -1.0);
      nh.param("uav" + to_string(i) + "/target_z", target[i][2], -1.0);

      cout << GREEN << "uav_"<< i <<"/target:  [" << target[i][0] << "," << target[i][1] << "," << target[i][2] << "]" << TAIL << endl;

        uav_name = "/uav" + std::to_string(i);
        // 【发布】目标点至EGO-planner-swarm
        planner_goal_pub[i] = nh.advertise<geometry_msgs::PoseStamped>(uav_name + "/prometheus/motion_planning/goal", 1);
    }  
 
    bool flag;
    cout << GREEN << "input 1 to pub preset target:" << TAIL << endl;
    cin >> flag;

    geometry_msgs::PoseStamped target_point;
    for (int i = 1; i <= swarm_num; i++)
    {
        target_point.pose.position.x = target[i][0];
        target_point.pose.position.y = target[i][1];
        target_point.pose.position.z = target[i][2];

        planner_goal_pub[i].publish(target_point);
    }  
    
    return 0;
}