#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <random>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <prometheus_drl/move_cmd.h>
#include <prometheus_drl/agent_reset.h>

using namespace std;

random_device rd;                               // 随机函数
default_random_engine eng(rd()); 
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
bool goal_pub;
geometry_msgs::PoseStamped goal;
geometry_msgs::PoseStamped collect_cmd;
ros::Publisher collect_pub;
nav_msgs::Odometry odom;
void fake_odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom = *msg;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　主函数　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "collect");
    ros::NodeHandle nh("~");

    string agent_prefix;
    // 模型前缀 - 默认为 ugv，无人机则设置为 uav
    nh.param<string>("agent_prefix", agent_prefix, "/ugv");
    int action_mode = 0;
    // 动作模式 - 0 代表离散控制，1代表连续控制
    nh.param("action_mode", action_mode, 0);

    string agent_name = agent_prefix + std::to_string(1);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(agent_name + "/fake_odom", 1, fake_odom_cb);

    //　【发布】　reset指令
    collect_pub = nh.advertise<geometry_msgs::PoseStamped>("/collect", 10);
    ros::Publisher ego_goal_pub = nh.advertise<geometry_msgs::PoseStamped>(agent_name + "/prometheus/ego/goal", 10);

    prometheus_drl::move_cmd discreated_action;
    geometry_msgs::Twist continued_action;
    prometheus_drl::agent_reset reset;

    discreated_action.ID = 0;
    discreated_action.CMD = prometheus_drl::move_cmd::HOLD;



    int num;
    unsigned int seed = rd();
    eng.seed(seed);

    while(ros::ok())
    {
 
        cout << ">>>>>>>>>>>>>>>> Welcome to use Prometheus Terminal Control <<<<<<<<<<<<<<<<"<< endl;
        cout << "Please input num.."<<endl;
        cin >> num;  

        for (int t = 1; t <= num; t++)
        {
            cout << "--------->>>>>>>>  num:   " <<  t <<endl;

            if( t%2 == 1)
            {
                rand_x = uniform_real_distribution<double>(9.0 , 10.0);
                rand_y = uniform_real_distribution<double>(-10 , 10);
            }else
            {
                rand_x = uniform_real_distribution<double>(-9.0 , -10.0);
                rand_y = uniform_real_distribution<double>(-10 , 10); 
            }


            goal.pose.position.x = rand_x(eng);  
            goal.pose.position.y = rand_y(eng); 
            ego_goal_pub.publish(goal);
            goal_pub = true;
            cout << "--------->>>>>>>> goal:   " <<  goal.pose.position.x << " [m] " <<  goal.pose.position.y << " [m] "<<endl;

            sleep(4.0);
            ROS_INFO("start collect");
            collect_cmd.pose.position.x = goal.pose.position.x;  
            collect_cmd.pose.position.y = goal.pose.position.y; 
            collect_cmd.pose.position.z = 1;   // 开始采集
            collect_pub.publish(collect_cmd);

            bool arrive_goal=false;
            while(!arrive_goal)
            {
                ros::spinOnce();
                sleep(0.2);

                if( abs(odom.pose.pose.position.x - goal.pose.position.x)<0.8 &&
                    abs(odom.pose.pose.position.y - goal.pose.position.y)<0.8       )
                {
                    ROS_INFO("end collect");
                    collect_cmd.pose.position.x = 0;  
                    collect_cmd.pose.position.y = 0; 
                    collect_cmd.pose.position.z = 2;   // 停止采集
                    collect_pub.publish(collect_cmd);
                    arrive_goal = true;
                    
                }
            }

            sleep(1.0);
        }


    }

    return 0;
}
