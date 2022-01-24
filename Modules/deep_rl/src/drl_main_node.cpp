#include <ros/ros.h>
#include <random>
#include <prometheus_drl/agent_reset.h>

#include "drl_actuator.h"
#include "drl_sensing.h"

using namespace drl_ns;

#define MAX_NUM 10                              // 最大数量

int swarm_num;                                  // 数量

drl_actuator agent_actuator[MAX_NUM];           // 智能体执行类
drl_sensing agent_sensor[MAX_NUM];              // 智能体感知类
float agent_height;
Eigen::Vector3d init_pos[MAX_NUM];              // 初始位置
double init_yaw[MAX_NUM];                       // 初始偏航角

random_device rd;                               // 随机函数
default_random_engine eng(rd()); 
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;

gazebo_msgs::ModelState model_state;            // Gazebo模型位置
ros::Publisher gazebo_model_state_pub;          // Gazebo模型位置发布

void get_preset_pos(int i);

Eigen::MatrixXd get_random_goal()
{
    Eigen::MatrixXd goal(swarm_num, 2);

    // 如何选择到不重复而且不在障碍物里的点？
    unsigned int seed = rd();
    eng.seed(seed);

    rand_x = uniform_real_distribution<double>(-2.5 , 2.5);
    rand_y = uniform_real_distribution<double>(-9.5 , 9.5);

    for(int i = 0; i<swarm_num; i++)
    {
        goal(i,0) = 9.5;
        goal(i,1) = rand_y(eng);
    }
    return goal;
}

void reset_cb(const prometheus_drl::agent_reset::ConstPtr& msg)
{
    cout << GREEN  << "Reset!"<< TAIL << endl;

    // 重置 - 获取随机目标
    Eigen::MatrixXd goal = get_random_goal();
    // 重置 - 获取随机初始位置
    for(int i=1; i<=swarm_num;i++)
    {
        get_preset_pos(i);
        agent_actuator[i].reset(init_pos[i], init_yaw[i]);
        agent_sensor[i].reset(goal);
    }

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

    // 【订阅】 智能体重置位置指令（每次训练结束后智能体需要重置位置）
    ros::Subscriber reset_sub = nh.subscribe<prometheus_drl::agent_reset>("/reset", 1, reset_cb);

    // 初始化 - 获取随机目标
    Eigen::MatrixXd goal = get_random_goal();
    // 初始化 - 获取随机初始位置
    for(int i=1; i<=swarm_num;i++)
    {
        get_preset_pos(i);
        agent_actuator[i].init(nh,i,init_pos[i],init_yaw[i]);
        agent_sensor[i].init(nh,i,goal);
    }

    sleep(0.5);
    
    // 【发布】 智能体位置(写在这里发布是因为set_model_state这个函数不能反复调用，会出现卡顿)
    gazebo_model_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
    // 【定时器】
    ros::Timer gazebo_pub_timer = nh.createTimer(ros::Duration(0.05), gazebo_pub_cb);
    // 【定时器】
    ros::Timer debug_timer = nh.createTimer(ros::Duration(5.0), debug_cb);

    ros::spin();

    return 0;
}


void get_preset_pos(int i)
{
    // 如何指定初始位置？
    unsigned int seed = rd();
    eng.seed(seed);

    rand_y = uniform_real_distribution<double>(-9.5 , 9.5);
    init_pos[i][0] = -9.5;
    init_pos[i][1] = rand_y(eng);

    // 
    init_pos[i][2] = agent_height;
    init_yaw[i] = 0.0;
}