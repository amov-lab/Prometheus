#include "apf.h"

namespace Local_Planning
{

void APF::init(ros::NodeHandle& nh)
{
    has_local_map_ = false;

    nh.param("apf/inflate_distance", inflate_distance, 0.20);  // 感知障碍物距离
    nh.param("apf/sensor_max_range", sensor_max_range, 2.5);  // 感知障碍物距离
    nh.param("apf/k_push", k_push, 0.8);                         // 推力增益
    nh.param("apf/k_att", k_att, 0.4);                                  // 引力增益
    nh.param("apf/min_dist", min_dist, 0.2);                            // 最小壁障距离
    nh.param("apf/max_att_dist", max_att_dist, 5.0);             // 最大吸引距离
    nh.param("apf/ground_height", ground_height, 0.1);  // 地面高度
    nh.param("apf/ground_safe_height", ground_safe_height, 0.2);  // 地面安全距离
    nh.param("apf/safe_distance", safe_distance, 0.15); // 安全停止距离

    // TRUE代表2D平面规划及搜索,FALSE代表3D 
    nh.param("local_planner/is_2D", is_2D, true); 
}

void APF::set_local_map(sensor_msgs::PointCloud2ConstPtr &local_map_ptr)
{
    local_map_ptr_ = local_map_ptr;
    ros::Time begin_load_point_cloud = ros::Time::now();

    pcl::fromROSMsg(*local_map_ptr, latest_local_pcl_);

    has_local_map_=true;
}

void APF::set_local_map_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_ptr)
{
    latest_local_pcl_ = *pcl_ptr;
    has_local_map_=true;
}

void APF::set_odom(nav_msgs::Odometry cur_odom)
{
    cur_odom_ = cur_odom;
    has_odom_=true;
}

int APF::compute_force(Eigen::Vector3d &goal, Eigen::Vector3d &desired_vel)
{
    // 0 for not init; 1for safe; 2 for dangerous
    int local_planner_state=0;  
    int safe_cnt=0;

    if(!has_local_map_|| !has_odom_)
        return 0;

    if ((int)latest_local_pcl_.points.size() == 0) 
        return 0;

    if (isnan(goal(0)) || isnan(goal(1)) || isnan(goal(2)))
        return 0;

    //　当前位置
    Eigen::Vector3d current_pos;
    current_pos[0] = cur_odom_.pose.pose.position.x;
    current_pos[1] = cur_odom_.pose.pose.position.y;
    current_pos[2] = cur_odom_.pose.pose.position.z;

    ros::Time begin_collision = ros::Time::now();

    // 引力
    Eigen::Vector3d uav2goal = goal - current_pos;
    // 不考虑高度影响
    uav2goal(2) = 0.0;
    double dist_att = uav2goal.norm();
    if(dist_att > max_att_dist)
    {
        uav2goal = max_att_dist * uav2goal/dist_att ;
    }
    //　计算吸引力
    attractive_force = k_att * uav2goal;

    // 排斥力
    double uav_height = cur_odom_.pose.pose.position.z;
    push_force = Eigen::Vector3d(0.0, 0.0, 0.0);

    Eigen::Vector3d p3d;
    vector<Eigen::Vector3d> obstacles;
    
    //　根据局部点云计算排斥力（是否可以考虑对点云进行降采样？）
    for (size_t i = 0; i < latest_local_pcl_.points.size(); ++i) 
    {
        p3d(0) = latest_local_pcl_.points[i].x;
        p3d(1) = latest_local_pcl_.points[i].y;
        p3d(2) = latest_local_pcl_.points[i].z;

        Eigen::Vector3d current_pos_local(0.0, 0.0, 0.0);
        
        //　低于地面高度，则不考虑该点的排斥力
        double point_height_global = uav_height+p3d(2);
        if(fabs(point_height_global)<ground_height)
            continue;

        //　超出最大感知距离，则不考虑该点的排斥力
        double dist_push = (current_pos_local - p3d).norm();
        if(dist_push > sensor_max_range || isnan(dist_push))
            continue;

        //　考虑膨胀距离
        dist_push = dist_push - inflate_distance;

        // 如果当前的观测点中，包含小于安全停止距离的点，进行计数
        if(dist_push < safe_distance)
        {
            safe_cnt++;
        }
        
        //　小于最小距离时，则增大该距离，从而增大排斥力
        if(dist_push < min_dist)
        {
            dist_push = min_dist /1.5;
        }

        obstacles.push_back(p3d);
        double push_gain = k_push * (1/dist_push - 1/sensor_max_range)* 1.0/(dist_push * dist_push);

        if(dist_att<1.0)
        {
            push_gain *= dist_att;  // to gaurantee to reach the goal.
        }

        push_force += push_gain * (current_pos_local - p3d)/dist_push;
    }

    //　平均排斥力
    if(obstacles.size() != 0)
    {
        push_force=push_force/obstacles.size();
    }

    Eigen::Quaterniond cur_rotation_local_to_global(cur_odom_.pose.pose.orientation.w, 
                                                    cur_odom_.pose.pose.orientation.x,  
                                                    cur_odom_.pose.pose.orientation.y,  
                                                    cur_odom_.pose.pose.orientation.z); 


    Eigen::Matrix<double,3,3> rotation_mat_local_to_global = cur_rotation_local_to_global.toRotationMatrix();

    push_force = rotation_mat_local_to_global * push_force; 

    // 合力
    desired_vel = push_force + attractive_force;

    if(is_2D)
    {
        desired_vel[2] = 0.0;
    }

    // 如果不安全的点超出，
    if(safe_cnt>10)
    {
        local_planner_state = 2;  //成功规划，但是飞机不安全
    }else
    {
        local_planner_state =1;  //成功规划， 安全
    }

    static int exec_num=0;
    exec_num++;

    // 此处改为根据循环时间计算的数值
    if(exec_num == 50)
    {
        printf("APF calculate take %f [s].\n",   (ros::Time::now()-begin_collision).toSec());
        exec_num=0;
    }  

    return local_planner_state;
}



}
