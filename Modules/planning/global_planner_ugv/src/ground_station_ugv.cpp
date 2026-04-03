#include <ros/ros.h>

#include <Eigen/Eigen>
#include <string>
#include <std_msgs/Float32.h>
#include <prometheus_msgs/StationCommand.h>

#include <random>
#include "printf_utils.h"
using namespace std;


int swarm_num_ugv;
int ugv_id;
bool manual_goal;
float yaw_ref;
Eigen::Vector3d ugv_goal;
double ugv_height{0.0};
double goal_pos_x;
double goal_pos_y;
double goal_yaw;

bool rviz_recive_flag;
prometheus_msgs::StationCommand ugv_cmd;
ros::Publisher ugv_cmd_pub;
ros::Subscriber goal_sub;

void goal_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    // 2D定高飞行
    ugv_goal << msg->pose.position.x, msg->pose.position.y, ugv_height;
    yaw_ref = msg->pose.orientation.w;    
    rviz_recive_flag = true;
    cout << GREEN << "/ugv" + std::to_string(ugv_id) + " Global_Planner_UGV: Get a new manual goal: ["<< ugv_goal[0] << ", "  << ugv_goal[1]  << ", "  << yaw_ref << " ]"  << TAIL <<endl;
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "ground_station_ugv");
    ros::NodeHandle nh("~");

    nh.param("ugv_id", ugv_id, 0);
    nh.param("manual_goal", manual_goal, false);
    // 无人车高度
    nh.param("ugv_height", ugv_height, 0.0);
    nh.param("goal_pos_x", goal_pos_x, 0.0);
    nh.param("goal_pos_y", goal_pos_y, 0.0);
    nh.param("goal_yaw", goal_yaw, 1.0);

    rviz_recive_flag = false;

    ugv_cmd_pub = nh.advertise<prometheus_msgs::StationCommand>("/ugv"+std::to_string(ugv_id) +"/ground_station/ugv_cmd", 1);


    while(ros::ok())
    {     
        int control_command;
        std::cout << "Please input control command: 1 for Start, 2 for Return, 3 for Stop" << std::endl;
        std::cin >> control_command;
        if(control_command == 1)
        {
            // 读取预设的无人车，无人机目标点(手动输入或者程序内部读取)
            if(manual_goal)
            {
                std::cout << "Please select the target point in Rviz !" << std::endl;
                goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/ugv" + std::to_string(ugv_id) + "/prometheus/global_planner_ugv/goal", 1, &goal_cb);
                sleep(5);
                if(rviz_recive_flag)
                {
                    std::cout << "After selecting the target point, enter 1 to continue !" << std::endl;
                    std::cin.get();
                }
                else
                {
                    std::cout << "Select the target point timeout!" << std::endl;
                    break;
                }
            }else
            {   
                ugv_goal[0] =  goal_pos_x;
                ugv_goal[1] =  goal_pos_y;
                yaw_ref =  goal_yaw;
                cout << GREEN << "/ugv" + std::to_string(ugv_id) + " Global_Planner_UGV: Get yaml goal: ["<< ugv_goal[0] << ", "  << ugv_goal[1]  << ", "  << yaw_ref << " ]"  << TAIL <<endl;
            }

            cout << GREEN << "[ground_station_ugv] ---> Start CMD."  << TAIL <<endl;

            ugv_cmd.Command = prometheus_msgs::StationCommand::Start;
            ugv_cmd.goal.pose.position.x = ugv_goal[0];
            ugv_cmd.goal.pose.position.y = ugv_goal[1];
            ugv_cmd.goal.pose.position.z = ugv_goal[2];
            ugv_cmd_pub.publish(ugv_cmd);  
        }
        else if(control_command == 2)
        {
            cout << GREEN << "[ground_station_ugv] ---> Return CMD."  << TAIL <<endl;

            ugv_cmd.Command = prometheus_msgs::StationCommand::Return;
            ugv_cmd_pub.publish(ugv_cmd);
        }
        else if(control_command == 3)
        {
            cout << GREEN << "[ground_station_ugv] ---> Stop CMD."  << TAIL <<endl;

            ugv_cmd.Command = prometheus_msgs::StationCommand::Stop;
            ugv_cmd_pub.publish(ugv_cmd);
        }
        else
        {   
            cout << RED << "[ground_station_ugv] ---> Wrong external CMD."  << TAIL <<endl;
        } 
       
    }

    ros::spin();

    return 0;
}