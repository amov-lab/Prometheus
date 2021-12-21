#include "drl_sensing.h"

namespace drl_ns
{
// 初始化函数
void drl_sensing::init(ros::NodeHandle& nh, int id, Eigen::MatrixXd goal)
{
    // 集群数量
    nh.param("swarm_num_ugv", swarm_num_ugv, 1);
    // 是否为仿真模式
    nh.param("sim_mode", sim_mode, false); 
    // 矩阵方格每一格的实际长度
    nh.param("block_size", block_size, 0.5f);

    inv_block_size = 1.0 / block_size;
    // 矩阵的维数
    matrix_size = 10; 

    ugv_id = id;
    ugv_name = "/ugv" + std::to_string(ugv_id);

    ugv_goal = goal;

    // 【订阅】 无人车真实位置 - 来自gazebo
    odom_sub = nh.subscribe<nav_msgs::Odometry>(ugv_name + "/odom", 1, &drl_sensing::odom_cb, this);
    // 【订阅】 激光雷达原始数据 - 来自gazebo
    scan_sub = nh.subscribe<sensor_msgs::LaserScan>(ugv_name + "/scan", 1, &drl_sensing::scan_cb, this);

    // 【订阅】其他无人车位置
    for(int i = 1; i <= swarm_num_ugv; i++)
    {
        if(i == ugv_id)
        {
            continue;
        }
        get_nei_odom[i] = false;
        odom_nei[i] << 99.9,99.9,99.9;
        nei_odom_sub[i] = nh.subscribe<nav_msgs::Odometry>("/ugv"+std::to_string(i)+"/fake_odom", 10, boost::bind(&drl_sensing::nei_odom_cb,this,_1,i));
    }
    // 【发布】无人车状态 - 发布到learning节点
    ugv_state_pub = nh.advertise<prometheus_drl::ugv_state>(ugv_name + "/ugv_state", 1);
    // 【定时器】发布无人车状态定时器
    pub_ugv_state_timer = nh.createTimer(ros::Duration(0.05), &drl_sensing::pub_ugv_state_cb, this);        
    // 【定时器】在全局地图中更新其他无人车位置
    update_other_ugv_pos_timer = nh.createTimer(ros::Duration(0.1), &drl_sensing::update_other_ugv_pos_cb, this);        
    
    // 初始化占据地图
    Occupy_map_ptr.reset(new Occupy_map);
    Occupy_map_ptr->init(nh, id);
    odom_ready = false;
    sensor_ready = false;
    ugv_height = 0.1;
}

void drl_sensing::reset(Eigen::MatrixXd goal)
{
    ugv_goal = goal;

    // 【订阅】其他无人车位置
    for(int i = 1; i <= swarm_num_ugv; i++)
    {
        if(i == ugv_id)
        {
            continue;
        }
        get_nei_odom[i] = false;
        odom_nei[i] << 99.9,99.9,99.9;
    }
    odom_ready = false;
    sensor_ready = false;
    Occupy_map_ptr->reset();
}

void drl_sensing::pub_ugv_state_cb(const ros::TimerEvent& e)
{
    static int exec_num=0;
    exec_num++;

    // 检查当前状态，不满足规划条件则直接退出主循环
    if(!odom_ready || !sensor_ready)
    {
        // 此处改为根据循环时间计算的数值
        if(exec_num == 50)
        {
            if(!odom_ready)
            {
                cout << YELLOW << ugv_name + " drl_sensing: [ Need Odom ]."  << TAIL <<endl;
            }else if(!sensor_ready)
            {
                cout << YELLOW << ugv_name + " drl_sensing: [ Need sensor info ]."  << TAIL <<endl;
            }
            exec_num=0;
        }  

        return;
    }

    //记录开始时间
    ros::Time time_start = ros::Time::now();

    // 根据方格点的位置确定
    ugv_state.matrix_size = matrix_size;
    ugv_state.block_size = block_size;

    // 确定左上角位置
    Eigen::Vector3d left_up_corner_pos;
    left_up_corner_pos[0] = ugv_odom.pose.pose.position.x + block_size*matrix_size/2;
    left_up_corner_pos[1] = ugv_odom.pose.pose.position.y + block_size*matrix_size/2;
    left_up_corner_pos[2] = ugv_height; 
    Eigen::Vector3d right_down_corner_pos;
    right_down_corner_pos[0] = ugv_odom.pose.pose.position.x - block_size*matrix_size/2;
    right_down_corner_pos[1] = ugv_odom.pose.pose.position.y - block_size*matrix_size/2;
    right_down_corner_pos[2] = ugv_height; 


    // 障碍物矩阵
    for(int i = 0; i < matrix_size; i++)
    {
        for(int j = 0; j < matrix_size; j++)
        {
            Eigen::Vector3d new_corner_pos = left_up_corner_pos;
            new_corner_pos[0] -= i*block_size;
            new_corner_pos[1] -= j*block_size;
            ugv_state.obstacle_matrix[i*matrix_size+ j] = Occupy_map_ptr->getOccupancy(new_corner_pos, block_size); 
        }
    }

    // 邻居位置矩阵
    // 首先全部置0
    for(int i = 0; i < matrix_size*matrix_size; i++)
    {
        ugv_state.agent_matrix[i] = 0; 
    }
    // 然后查找邻居位置是否填充
    for(int i = 1; i <= swarm_num_ugv; i++)
    {
        if(i == ugv_id)
        {
            continue;
        }
        if(!get_nei_odom[i])
        {
            continue;
        }
        if( odom_nei[i][0]>left_up_corner_pos[0] || odom_nei[i][0]<right_down_corner_pos[0]||
            odom_nei[i][1]>left_up_corner_pos[1] || odom_nei[i][1]<right_down_corner_pos[1])
        {
            continue;
        }
        Eigen::Vector3i id;
        for (int j = 0; j < 2; ++j)
        {
            id(j) = floor( (left_up_corner_pos(j) - odom_nei[i][j]) * inv_block_size );
        }

        ugv_state.agent_matrix[id(0)*matrix_size+id(1)] = i; 
    }

    // 目标矩阵
    // 首先全部置0
    // ugv_goal(ugv_id-1,0)
    for(int i = 0; i < matrix_size*matrix_size; i++)
    {
        ugv_state.goal_matrix[i] = 0; 
    }
    if( ugv_goal(ugv_id-1,0)<left_up_corner_pos[0] && ugv_goal(ugv_id-1,0)>right_down_corner_pos[0]&&
        ugv_goal(ugv_id-1,1)<left_up_corner_pos[1] && ugv_goal(ugv_id-1,1)>right_down_corner_pos[1])
    {
        Eigen::Vector3i id;
        for (int j = 0; j < 2; ++j)
        {
            id(j) = floor( (left_up_corner_pos(j) - ugv_goal(ugv_id-1,j)) * inv_block_size );
        }

        ugv_state.goal_matrix[id(0)*matrix_size+id(1)] = 1; 
    }

    // 邻居目标矩阵
    for(int i = 0; i < matrix_size*matrix_size; i++)
    {
        ugv_state.nei_goal_matrix[i] = 0; 
    }
    // 类似填充自己的goal，填充邻居的goal
    for(int i = 1; i <= swarm_num_ugv; i++)
    {
        if(i == ugv_id)
        {
            continue;
        }
        if( ugv_goal(i-1,0)<left_up_corner_pos[0] && ugv_goal(i-1,0)>right_down_corner_pos[0]&&
            ugv_goal(i-1,1)<left_up_corner_pos[1] && ugv_goal(i-1,1)>right_down_corner_pos[1])
        {
            Eigen::Vector3i id;
            for (int j = 0; j < 2; ++j)
            {
                id(j) = floor( (left_up_corner_pos(j) - ugv_goal(i-1,j)) * inv_block_size );
            }
            // 如何赋值呢？
            ugv_state.nei_goal_matrix[id(0)*matrix_size+id(1)] = i; 
        }
    }

    ugv_state_pub.publish(ugv_state);

    // 此处改为根据循环时间计算的数值
    if(exec_num == 1000)
    {
        // 膨胀地图效率与地图大小有关
        cout << YELLOW << "drl_sensing: Matrix update take " << (ros::Time::now()-time_start).toSec() <<" [s]. " << TAIL <<endl;
        
        exec_num=0;
    }

}

void drl_sensing::update_other_ugv_pos_cb(const ros::TimerEvent& e)
{
    // 暂时不使用其他无人机位置来更新地图
    // Occupy_map_ptr->map_update_other_ugv(odom_nei, get_nei_odom, swarm_num_ugv);
}

void drl_sensing::odom_cb(const nav_msgs::OdometryConstPtr& msg)
{
    ugv_odom = *msg;

    ugv_odom.pose.pose.position.z = ugv_height;

    odom_ready = true;
}

void drl_sensing::scan_cb(const sensor_msgs::LaserScanConstPtr &msg)
{
    if (!odom_ready) 
    {
        return;
    }

    Occupy_map_ptr->map_update_laser(msg, ugv_odom);
    sensor_ready = true;
}

void drl_sensing::nei_odom_cb(const nav_msgs::Odometry::ConstPtr& odom, int id) 
{
    odom_nei[id] << odom->pose.pose.position.x, odom->pose.pose.position.y, ugv_height; 

    get_nei_odom[id] = true;
}


void drl_sensing::printf_cb()
{
    cout << GREEN  << "State  : " << TAIL <<endl;

    cout << GREEN  << "pos    : " << ugv_odom.pose.pose.position.x << " [m] "<< ugv_odom.pose.pose.position.y <<" [m] "<< TAIL <<endl;
    cout << GREEN  << "goal   : " << ugv_goal(ugv_id-1,0) << " [m] "<< ugv_goal(ugv_id-1,1) <<" [m] "<< TAIL <<endl;

}

}