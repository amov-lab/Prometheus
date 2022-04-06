#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>

#include <geometry_msgs/PoseStamped.h>

#include <prometheus_drl/move_cmd.h>
#include <prometheus_drl/agent_reset.h>

using namespace std;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　主函数　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "terminal_control");
    ros::NodeHandle nh("~");

    string agent_prefix;
    // 模型前缀 - 默认为 ugv，无人机则设置为 uav
    nh.param<string>("agent_prefix", agent_prefix, "/ugv");
    int action_mode = 0;
    // 动作模式 - 0 代表离散控制，1代表连续控制
    nh.param("action_mode", action_mode, 0);

    string agent_name = agent_prefix + std::to_string(1);

    //　【发布】　离散控制指令
    ros::Publisher discreated_action_pub = nh.advertise<prometheus_drl::move_cmd>(agent_name + "/move_cmd", 10);
    //　【发布】　连续控制指令
    ros::Publisher continued_action_pub = nh.advertise<geometry_msgs::Twist>(agent_name + "/cmd_vel", 10);    
    
    //　【发布】　reset指令
    ros::Publisher collect_pub = nh.advertise<geometry_msgs::PoseStamped>("/collect", 10);
    ros::Publisher reset_pub = nh.advertise<prometheus_drl::agent_reset>("/reset", 10);
    ros::Publisher ego_goal_pub = nh.advertise<geometry_msgs::PoseStamped>(agent_name + "/prometheus/ego/goal", 10);

    prometheus_drl::move_cmd discreated_action;
    geometry_msgs::Twist continued_action;
    prometheus_drl::agent_reset reset;

    discreated_action.ID = 0;
    discreated_action.CMD = prometheus_drl::move_cmd::HOLD;

    geometry_msgs::PoseStamped collect_cmd;
    geometry_msgs::PoseStamped goal;

    int cmd_mode;

    while(ros::ok())
    {
        if(action_mode == 0)
        {
            cout << ">>>>>>>>>>>>>>>> Welcome to use Prometheus Terminal Control <<<<<<<<<<<<<<<<"<< endl;
            cout << "Please select cmd: 0 for hold, 1 for forward, 2 for back, 3 for left, 4 for right, 99 for reset.."<<endl;
            cin >> cmd_mode;

            if(cmd_mode == 0)
            {
                discreated_action.ID += 1;
                discreated_action.CMD = prometheus_drl::move_cmd::HOLD;
                discreated_action_pub.publish(discreated_action);
            }else if(cmd_mode == 1)
            {
                discreated_action.ID += 1;
                discreated_action.CMD = prometheus_drl::move_cmd::FORWARD;
                discreated_action_pub.publish(discreated_action);
            }else if(cmd_mode == 2)
            {
                discreated_action.ID += 1;
                discreated_action.CMD = prometheus_drl::move_cmd::BACK;
                discreated_action_pub.publish(discreated_action);
            }else if(cmd_mode == 3)
            {
                discreated_action.ID += 1;
                discreated_action.CMD = prometheus_drl::move_cmd::LEFT;
                discreated_action_pub.publish(discreated_action);
            }else if(cmd_mode == 4)
            {
                discreated_action.ID += 1;
                discreated_action.CMD = prometheus_drl::move_cmd::RIGHT;
                discreated_action_pub.publish(discreated_action);
            }else if(cmd_mode == 99)
            {
                discreated_action.ID = 0;
                reset.reset = 1;
                reset_pub.publish(reset);
            }else
            {
                ROS_ERROR("wrong input");
            }
        }else if(action_mode == 1)
        {
            cout << ">>>>>>>>>>>>>>>> Welcome to use Prometheus Terminal Control <<<<<<<<<<<<<<<<"<< endl;
            cout << "Please select cmd: 0 for hold, 1 for forward, 2 for back, 3 for left, 4 for right, 99 for reset.."<<endl;
            cin >> cmd_mode;  

            // todo
            if(cmd_mode == 0)
            {
                continued_action.linear.x = 0.0;
                continued_action.linear.y = 0.0;
                continued_action.linear.z = 0.0;
                continued_action.angular.x = 0.0;
                continued_action.angular.y = 0.0;
                continued_action.angular.z = 0.0;
                for(int i=0;i<10;i++)
                {
                    continued_action_pub.publish(continued_action);
                    sleep(0.1);
                }
            }else if(cmd_mode == 1)
            {
                continued_action.linear.x = 1.0;
                continued_action.linear.y = 0.0;
                continued_action.linear.z = 0.0;
                continued_action.angular.x = 0.0;
                continued_action.angular.y = 0.0;
                continued_action.angular.z = 0.0;
                for(int i=0;i<10;i++)
                {
                    continued_action_pub.publish(continued_action);
                    sleep(0.1);
                }
            }else if(cmd_mode == 2)
            {
                continued_action.linear.x = -1.0;
                continued_action.linear.y = 0.0;
                continued_action.linear.z = 0.0;
                continued_action.angular.x = 0.0;
                continued_action.angular.y = 0.0;
                continued_action.angular.z = 0.0;
                for(int i=0;i<10;i++)
                {
                    continued_action_pub.publish(continued_action);
                    sleep(0.1);
                }
            }else if(cmd_mode == 3)
            {
                continued_action.linear.x = 0.0;
                continued_action.linear.y = 1.0;
                continued_action.linear.z = 0.0;
                continued_action.angular.x = 0.0;
                continued_action.angular.y = 0.0;
                continued_action.angular.z = 0.0;
                for(int i=0;i<10;i++)
                {
                    continued_action_pub.publish(continued_action);
                    sleep(0.1);
                }
            }else if(cmd_mode == 4)
            {
                continued_action.linear.x = 0.0;
                continued_action.linear.y = -1.0;
                continued_action.linear.z = 0.0;
                continued_action.angular.x = 0.0;
                continued_action.angular.y = 0.0;
                continued_action.angular.z = 0.0;
                for(int i=0;i<10;i++)
                {
                    continued_action_pub.publish(continued_action);
                    sleep(0.1);
                }
            }else if(cmd_mode == 99)
            {
                reset.reset = 1;
                reset_pub.publish(reset);
            }else
            {
                ROS_ERROR("wrong input");
            }
        }
        else if(action_mode == 2)
        {
            cout << ">>>>>>>>>>>>>>>> Welcome to use Prometheus Terminal Control <<<<<<<<<<<<<<<<"<< endl;
            cout << "Please select cmd: 0 for Goal, 1 for stop collect.."<<endl;
            cin >> cmd_mode;  

            // todo
            if(cmd_mode == 0)
            {
                cout << "X:"<<endl;
                cin >> goal.pose.position.x;  
                cout << "Y:"<<endl;
                cin >> goal.pose.position.y;  
                ego_goal_pub.publish(goal);

                sleep(4.0);
                ROS_INFO("start collect");
                collect_cmd.pose.position.x = goal.pose.position.x;  
                collect_cmd.pose.position.y = goal.pose.position.y; 
                collect_cmd.pose.position.z = 1;   // 开始采集
                collect_pub.publish(collect_cmd);

            }else if(cmd_mode == 1)
            {
                ROS_INFO("end collect");
                collect_cmd.pose.position.x = 0;  
                collect_cmd.pose.position.y = 0; 
                collect_cmd.pose.position.z = 2;   // 停止采集
                collect_pub.publish(collect_cmd);

                // reset.reset = 1;
                // reset_pub.publish(reset);
            }else
            {
                ROS_ERROR("wrong input");
            }
        }
        

        sleep(0.5);
    }

    return 0;
}
