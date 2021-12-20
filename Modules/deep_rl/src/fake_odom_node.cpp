#include "fake_ugv.h"
#include <random>

#define MAX_NUM 40
int swarm_num_ugv;
int preset_init_pos_flag;
string node_name;
gazebo_msgs::ModelState model_state;

Fake_UGV ugv_agent[MAX_NUM];
Eigen::Vector3d init_pos_ugv[MAX_NUM];
double init_yaw_ugv[MAX_NUM];

random_device rd;
default_random_engine eng(rd()); 
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;

ros::Publisher gazebo_model_state_pub;
void get_preset_pos_ugv(int i);
void gazebo_pub_cb(const ros::TimerEvent &e);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_odom_node");
    ros::NodeHandle nh("~");

    nh.param("fake_odom/swarm_num_ugv", swarm_num_ugv, 8);
    nh.param("fake_odom/preset_init_pos_flag", preset_init_pos_flag, 1);

    unsigned int seed = rd();
    // unsigned int seed = 2433201515;
    // cout << GREEN<< "random seed=" << seed << TAIL<< endl;
    eng.seed(seed);


    for(int i = 0; i<swarm_num_ugv;i++)
    {
        get_preset_pos_ugv(i);

        ugv_agent[i].init(nh, i+1, init_pos_ugv[i], init_yaw_ugv[i]);
    }

    sleep(0.5);

    gazebo_model_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
    
    ros::Timer gazebo_pub_timer = nh.createTimer(ros::Duration(0.1), gazebo_pub_cb);

    node_name = "[fake_odom_node]"; 
    cout << GREEN << node_name << " init! "<< TAIL << endl;

    ros::spin();

    return 0;
}

void gazebo_pub_cb(const ros::TimerEvent &e)
{
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