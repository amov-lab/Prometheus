#include "fake_uav.h"
#include "fake_ugv.h"
#include <random>

#define MAX_NUM 40
int swarm_num_uav,swarm_num_ugv;
bool manual_init_pos;
bool pub_gazebo_model_state;
int preset_init_pos_flag;
string node_name;
gazebo_msgs::ModelState model_state;

Fake_UAV uav_agent[MAX_NUM];
Eigen::Vector3d init_pos_uav[MAX_NUM];
double init_yaw_uav[MAX_NUM];

Fake_UGV ugv_agent[MAX_NUM];
Eigen::Vector3d init_pos_ugv[MAX_NUM];
double init_yaw_ugv[MAX_NUM];

random_device rd;
default_random_engine eng(rd()); 
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;

ros::Publisher gazebo_model_state_pub;
void get_preset_pos_uav(int i);
void get_preset_pos_ugv(int i);
void gazebo_pub_cb(const ros::TimerEvent &e);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_odom_node");
    ros::NodeHandle nh("~");
    // fake_odom 主要是取代PX4在环仿真+Gazebo，
    // 订阅来自uav_control的底层控制指令，根据建立的无人机数学模型，计算并发布无人机状态数据，直接将动力学结果输出至RVIZ界面，完成快速简易仿真
    // fake_odom还可以发布model_state 直接将结果显示在Gazebo中，实现实景仿真
    // 仿真器中包含了地图生成器，可以模拟传感器发布全局地图和局部地图信息，用于规划模块的输入
    // 用于规划算法仿真：地图模拟 ---> 规划模块(如A*) --- (规划指令,/prometheus/command) ---> 控制模块(即uav_cotrol) --- (底层控制指令,即mavros指令) ---> fake_odom 
    // 同时控制算法仿真：控制模块(即uav_cotrol) --- (底层控制指令,即mavros指令) ---> fake_odom 

    nh.param("fake_odom/swarm_num_uav", swarm_num_uav, 8);
    nh.param("fake_odom/swarm_num_ugv", swarm_num_ugv, 8);
    nh.param("fake_odom/manual_init_pos", manual_init_pos, false);
    nh.param("fake_odom/preset_init_pos_flag", preset_init_pos_flag, 1);
    nh.param("fake_odom/pub_gazebo_model_state", pub_gazebo_model_state, false);

    unsigned int seed = rd();
    eng.seed(seed);

    for(int i = 0; i<swarm_num_uav;i++)
    {
        // 设置无人机初始位置
        if(manual_init_pos)
        {
            nh.param("fake_odom/uav" + to_string(i+1) + "_init_x", init_pos_uav[i][0], 0.0);
            nh.param("fake_odom/uav" + to_string(i+1) + "_init_y", init_pos_uav[i][1], 0.0);
            nh.param("fake_odom/uav" + to_string(i+1) + "_init_z", init_pos_uav[i][2], 0.0);
            nh.param("fake_odom/uav" + to_string(i+1) + "_init_yaw", init_yaw_uav[i], 0.0);    
        }else
        {
            // 得到预设的初始位置
            get_preset_pos_uav(i);
        }
        uav_agent[i].init(nh, i+1, init_pos_uav[i], init_yaw_uav[i]);
    }

    for(int i = 0; i<swarm_num_ugv;i++)
    {
        // 设置无人机初始位置
        if(manual_init_pos)
        {
            nh.param("fake_odom/ugv" + to_string(i+1) + "_init_x", init_pos_ugv[i][0], 0.0);
            nh.param("fake_odom/ugv" + to_string(i+1) + "_init_y", init_pos_ugv[i][1], 0.0);
            nh.param("fake_odom/ugv" + to_string(i+1) + "_init_z", init_pos_ugv[i][2], 0.08);
            nh.param("fake_odom/ugv" + to_string(i+1) + "_init_yaw", init_yaw_ugv[i], 0.0);    
        }else
        {
            // 得到预设的初始位置
            get_preset_pos_ugv(i);
        }
        ugv_agent[i].init(nh, i+1, init_pos_ugv[i], init_yaw_ugv[i]);
    }

    sleep(0.5);

    if(pub_gazebo_model_state)
    {
        gazebo_model_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
        ros::Timer gazebo_pub_timer = nh.createTimer(ros::Duration(0.1), gazebo_pub_cb);
    }

    node_name = "[fake_odom_node]"; 
    cout << GREEN << node_name << " init! "<< TAIL << endl;

    ros::spin();

    return 0;
}

void gazebo_pub_cb(const ros::TimerEvent &e)
{
    for(int i = 0; i<swarm_num_uav; i++)
    {
        model_state = uav_agent[i].get_model_state();
        
        gazebo_model_state_pub.publish(model_state);

        sleep(0.001);
    }

    for(int i = 0; i<swarm_num_ugv; i++)
    {
        model_state = ugv_agent[i].get_model_state();
        
        gazebo_model_state_pub.publish(model_state);

        sleep(0.001);
    }
}

void get_preset_pos_ugv(int i)
{
    int ugv_id = i+1;
    if(preset_init_pos_flag == 1)
    {
        if(ugv_id%2==1)
        {
            init_pos_ugv[i][0] = 0.5 * ugv_id;
            init_pos_ugv[i][1] = -1.0;
            init_pos_ugv[i][2] = 0.0;
            init_yaw_ugv[i] = 0.0;
        }else
        {
            init_pos_ugv[i][0] = -0.5 * (ugv_id - 1);
            init_pos_ugv[i][1] = -1.0;
            init_pos_ugv[i][2] = 0.0;
            init_yaw_ugv[i] = 0.0;
        }
    }else
    {
        rand_y = uniform_real_distribution<double>(-10 , 10);
        init_pos_ugv[i][0] = -1.0;
        init_pos_ugv[i][1] = rand_y(eng);
        init_pos_ugv[i][2] = 0.0;
        init_yaw_ugv[i] = 0.0;
        cout << RED  << "Wrong preset_init_pos_flag (ugv)."<< TAIL << endl;
    }
}

void get_preset_pos_uav(int i)
{
    int uav_id = i+1;
    if(preset_init_pos_flag == 1)
    {
        if(uav_id%2==1)
        {
            init_pos_uav[i][0] = 0.5 * uav_id;
            init_pos_uav[i][1] = 1.0;
            init_pos_uav[i][2] = 0.0;
            init_yaw_uav[i] = 0.0;
        }else
        {
            init_pos_uav[i][0] = -0.5 * (uav_id - 1);
            init_pos_uav[i][1] = 1.0;
            init_pos_uav[i][2] = 0.0;
            init_yaw_uav[i] = 0.0;
        }
    }else
    {
        rand_y = uniform_real_distribution<double>(-10 , 10);
        init_pos_uav[i][0] = 1.0;
        init_pos_uav[i][1] = rand_y(eng);
        init_pos_uav[i][2] = 0.0;
        init_yaw_uav[i] = 0.0;
        cout << RED  << "Wrong preset_init_pos_flag(uav)."<< TAIL << endl;
    }
}