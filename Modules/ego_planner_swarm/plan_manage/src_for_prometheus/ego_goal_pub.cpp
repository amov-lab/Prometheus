#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include "printf_utils.h"

#define MAX_SWRAM_NUM 41
ros::Publisher planner_goal_pub[MAX_SWRAM_NUM];
ros::Publisher planner_start_pub[MAX_SWRAM_NUM];

double target[10][3];

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ego_goal_pub");
    ros::NodeHandle nh("~");

    int swarm_num;
    bool sim_mode;
    // 无人机编号 1号无人机则为1
    nh.param("swarm_num", swarm_num, 1);
    nh.param("sim_mode", sim_mode, true);
 
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
        // 【发布】start_trigger
        planner_start_pub[i] = nh.advertise<geometry_msgs::PoseStamped>(uav_name + "/traj_start_trigger", 1);
    }  

    // 【订阅】EGO的轨迹输出(traj_server的输出)
    // ego_ouput_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(uav_name + "/prometheus/ego/traj_cmd", 1, &Case3FSM::ego_ouput_cb, this);
 
    bool flag;
    cout << GREEN << "input 1 to pub target:" << TAIL << endl;
    cin >> flag;

    geometry_msgs::PoseStamped target_point;
    geometry_msgs::PoseStamped start_trigger;
    for (int i = 1; i <= swarm_num; i++)
    {
        target_point.pose.position.x = target[i][0];
        target_point.pose.position.y = target[i][1];
        target_point.pose.position.z = target[i][2];

        planner_goal_pub[i].publish(target_point);
        // planner_start_pub[i].publish(start_trigger);
    }  
    
    return 0;
}