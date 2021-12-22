#include <ros/ros.h>
#include <random>
#include <prometheus_drl/ugv_reset.h>

#include "drl_actuator.h"
#include "drl_sensing.h"

using namespace drl_ns;

#define MAX_NUM 10

int swarm_num_ugv;

drl_actuator ugv_actuator[MAX_NUM];
drl_sensing ugv_sensor[MAX_NUM];

Eigen::Vector3d init_pos_ugv[MAX_NUM];
double init_yaw_ugv[MAX_NUM];

random_device rd;
default_random_engine eng(rd()); 
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;

ros::Publisher gazebo_model_state_pub;
gazebo_msgs::ModelState model_state;

void get_preset_pos_ugv(int i);

Eigen::MatrixXd get_random_goal()
{
    Eigen::MatrixXd goal(swarm_num_ugv, 2);

    // 如何选择到不重复而且不在障碍物里的点？
    unsigned int seed = rd();
    eng.seed(seed);

    rand_x = uniform_real_distribution<double>(-2.5 , 2.5);
    rand_y = uniform_real_distribution<double>(-9.5 , 9.5);

    for(int i = 0; i<swarm_num_ugv; i++)
    {
        goal(i,0) = 9.5;
        goal(i,1) = rand_y(eng);
    }
    return goal;
}

void reset_cb(const prometheus_drl::ugv_reset::ConstPtr& msg)
{
    cout << GREEN  << "Reset!"<< TAIL << endl;

    Eigen::MatrixXd goal = get_random_goal();
    for(int i=1; i<=swarm_num_ugv;i++)
    {
        get_preset_pos_ugv(i);

        ugv_actuator[i].reset(init_pos_ugv[i], init_yaw_ugv[i]);

        ugv_sensor[i].reset(goal);
    }

}
void gazebo_pub_cb(const ros::TimerEvent &e)
{
    for(int i=1; i<=swarm_num_ugv;i++)
    {
        model_state = ugv_actuator[i].get_model_state();
        
        gazebo_model_state_pub.publish(model_state);

        sleep(0.001);
    }
}

void debug_cb(const ros::TimerEvent &e)
{

    cout << BLUE << "-------> Debug INFO : " << TAIL <<endl;
    for(int i=1; i<=swarm_num_ugv;i++)
    {
        cout << GREEN  << "-------> UGV  [" << i << "] " << TAIL <<endl;
        ugv_sensor[i].printf_cb();
        ugv_actuator[i].printf_cb();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drl_main_node");
    ros::NodeHandle nh("~");
    nh.param("swarm_num_ugv", swarm_num_ugv, 1);

    ros::Subscriber reset_sub = nh.subscribe<prometheus_drl::ugv_reset>("/reset", 1, reset_cb);

    // 初始化
    Eigen::MatrixXd goal = get_random_goal();
    for(int i=1; i<=swarm_num_ugv;i++)
    {
        get_preset_pos_ugv(i);
        ugv_actuator[i].init(nh,i,init_pos_ugv[i],init_yaw_ugv[i]);
        ugv_sensor[i].init(nh,i,goal);
    }

    sleep(0.5);
    
    // 【发布】 无人车位置(写在这里发布是因为set_model_state这个函数不能反复调用，会出现卡顿)
    gazebo_model_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
    // 【定时器】
    ros::Timer gazebo_pub_timer = nh.createTimer(ros::Duration(0.1), gazebo_pub_cb);
    // 【定时器】
    ros::Timer debug_timer = nh.createTimer(ros::Duration(5.0), debug_cb);

    ros::spin();

    return 0;
}


void get_preset_pos_ugv(int i)
{
    // 如何指定初始位置？
    unsigned int seed = rd();
    eng.seed(seed);

    rand_y = uniform_real_distribution<double>(-9.5 , 9.5);
    init_pos_ugv[i][0] = -9.5;
    init_pos_ugv[i][1] = rand_y(eng);
    init_pos_ugv[i][2] = 0.0;
    init_yaw_ugv[i] = 0.0;
}