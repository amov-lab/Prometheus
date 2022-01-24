#include <ros/ros.h>
#include <random>
#include <prometheus_drl/agent_reset.h>

#include "drl_actuator.h"
#include "drl_sensing.h"

using namespace drl_ns;

#define MAX_NUM 100                              // 最大智能体数量
int swarm_num;                                  // 数量
drl_actuator agent_actuator[MAX_NUM];           // 智能体执行类 - 负责模拟智能体的运动
drl_sensing agent_sensor[MAX_NUM];              // 智能体感知类 - 负责收集传感器数据，发布learning算法需要的状态信息
float agent_height;                             // 智能体高度
random_device rd;                               // 随机函数
default_random_engine eng(rd()); 
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
gazebo_msgs::ModelState model_state;            // Gazebo模型位置
ros::Publisher gazebo_model_state_pub;          // Gazebo模型位置发布

Eigen::MatrixXd get_random_init_pos();                     // 获取随机初始位置函数
Eigen::MatrixXd get_random_goal();              // 获取随机目标函数
void reset_cb(const prometheus_drl::agent_reset::ConstPtr& msg);
void gazebo_pub_cb(const ros::TimerEvent &e);
void debug_cb(const ros::TimerEvent &e);
int main(int argc, char **argv)
{
    ros::init(argc, argv, "drl_main_node");
    ros::NodeHandle nh("~");
    string agent_prefix;
    nh.param("swarm_num", swarm_num, 1);
    nh.param<string>("agent_prefix", agent_prefix, "/ugv");

    if(swarm_num > MAX_NUM)
    {
        cout << RED  << "Error: swarm_num  [" << swarm_num << "] >" << "MAX_NUM  [" << MAX_NUM << "] " << TAIL <<endl;
        return 0;
    }

    if(agent_prefix == "/uav")
    {
        agent_height = 1.0;
    }else
    {
        agent_height = 0.1;
    }

    // 初始化 - 获取随机初始位置
    Eigen::MatrixXd init_position_matrix = get_random_init_pos();
    // 初始化 - 获取随机目标
    Eigen::MatrixXd goal = get_random_goal();
    // 初始化 - 获取随机初始位置
    for(int i=1; i<=swarm_num;i++)
    {
        Eigen::Vector3d init_pos = init_position_matrix.row(i-1); 
        agent_actuator[i].init(nh,i,init_pos,0.0);
        agent_sensor[i].init(nh,i,goal);
    }

    // 【订阅】 智能体重置位置指令（每次训练结束后智能体需要重置位置）
    ros::Subscriber reset_sub = nh.subscribe<prometheus_drl::agent_reset>("/reset", 1, reset_cb);
    // 【发布】 智能体位置(写在这里发布是因为set_model_state这个函数不能反复调用，会出现卡顿)
    gazebo_model_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
    // 【定时器】更新Gazebo中所有智能体模型的位置
    ros::Timer gazebo_pub_timer = nh.createTimer(ros::Duration(0.05), gazebo_pub_cb);
    // 【定时器】打印debug
    ros::Timer debug_timer = nh.createTimer(ros::Duration(5.0), debug_cb);

    ros::spin();

    return 0;
}


void reset_cb(const prometheus_drl::agent_reset::ConstPtr& msg)
{
    cout << GREEN  << "Get reset CMD!"<< TAIL << endl;
    // 重置 - 获取随机初始位置
    Eigen::MatrixXd init_position_matrix = get_random_init_pos();
    // 重置 - 获取随机目标
    Eigen::MatrixXd goal = get_random_goal();
    // 重置 - 获取随机初始位置
    for(int i=1; i<=swarm_num;i++)
    {
        Eigen::Vector3d init_pos = init_position_matrix.row(i-1);  
        // 重置初始点
        agent_actuator[i].reset(init_pos, 0.0);
        // 重置目标点（每一个智能体都知道全部智能体的目标点）
        agent_sensor[i].reset(goal);
    }
}

Eigen::MatrixXd get_random_init_pos()
{
    Eigen::MatrixXd pos(swarm_num, 3);
    double min_dist = 1.0;

    unsigned int seed = rd();
    eng.seed(seed);

    for(int i = 0; i<swarm_num; i++)
    {
        rand_y = uniform_real_distribution<double>(-9.5 , 9.5);
        pos(i,0) = -10.0;
        pos(i,1) = rand_y(eng);
        pos(i,2) = agent_height;

        // 检查是否与已经生成的智能体位置重叠：两个智能体之间距离小于min_dist，否则生成失败
        for(int j=0;j<i;j++)
        {
            if((Eigen::Vector2d(pos(i,0),pos(i,1)) - Eigen::Vector2d(pos(j,0),pos(j,1))).norm() < min_dist)
            {
                i--; // i--代表此次生成失败，重新生成
                break;  // 退出j的for循环
            }
        }
    }

    cout << GREEN  << "New init pos:"  << TAIL <<endl;
    cout << GREEN  << pos  << TAIL <<endl;
    return pos;
}

Eigen::MatrixXd get_random_goal()
{
    Eigen::MatrixXd goal(swarm_num, 2);
    double min_dist = 1.0;

    unsigned int seed = rd();
    eng.seed(seed);

    rand_x = uniform_real_distribution<double>(-2.5 , 2.5);
    rand_y = uniform_real_distribution<double>(-9.5 , 9.5);

    for(int i = 0; i<swarm_num; i++)
    {
        goal(i,0) = 10.0;
        goal(i,1) = rand_y(eng);

        // 检查是否与已经生成的智能体目标重叠：两个智能体目标之间距离小于min_dist，否则生成失败
        for(int j=0;j<i;j++)
        {
            if((Eigen::Vector2d(goal(i,0),goal(i,1)) - Eigen::Vector2d(goal(j,0),goal(j,1))).norm() < min_dist)
            {
                i--; // i--代表此次生成失败，重新生成
                break;  // 退出j的for循环
            }
        }
    }

    cout << GREEN  << "New goal:"  << TAIL <<endl;
    cout << GREEN  << goal  << TAIL <<endl;
    return goal;
}

void gazebo_pub_cb(const ros::TimerEvent &e)
{
    for(int i=1; i<=swarm_num;i++)
    {
        model_state = agent_actuator[i].get_model_state();
        gazebo_model_state_pub.publish(model_state);
        sleep(0.001);
    }
}

void debug_cb(const ros::TimerEvent &e)
{
    cout << BLUE << "-------> Debug INFO : " << TAIL <<endl;
    for(int i=1; i<=swarm_num;i++)
    {
        cout << GREEN  << "-------> Agent  [" << i << "] " << TAIL <<endl;
        agent_sensor[i].printf_cb();
        agent_actuator[i].printf_cb();
    }
}
