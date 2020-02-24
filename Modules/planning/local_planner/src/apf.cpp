#include "apf.h"

namespace local_planner
{

void APF::set_local_map(sensor_msgs::PointCloud2ConstPtr &local_map_ptr){
    local_map_ptr_ = local_map_ptr;
    ros::Time begin_load_point_cloud = ros::Time::now();
    ROS_INFO("--- SDFMAP_GLOBAL: have pcl, begin ---");
    pcl::fromROSMsg(*local_map_ptr, latest_local_pcl_);

    begin_update_map = ros::Time::now();
    has_local_map_=true;
}


bool APF::compute_force(Eigen::Matrix<double, 3, 1> &goal, Eigen::Matrix<double, 3, 1> current_odom, Eigen::Vector3d &desired_vel){
    if(!has_local_map_)
        return false;

    if ((int)latest_local_pcl_.points.size() == 0) 
        return false;

    if (isnan(current_odom(0)) || isnan(current_odom(1)) || isnan(current_odom(2)))
        return false;

    if (isnan(goal(0)) || isnan(goal(1)) || isnan(goal(2)))
        return false;

    vector<Eigen::Vector3d> obstacles;
    pcl::PointXYZ pt;
    Eigen::Vector3d p3d;
    ros::Time begin_collision = ros::Time::now();
    ROS_INFO("--- SDFMAP_GLABAL: begin collision_map, time: %f", begin_collision.toSec()-begin_update_map.toSec());
    ROS_INFO("point size: %d", latest_local_pcl_.points.size());

    // 引力
    double dist_att = (goal - current_odom).norm();
    if(dist_att > max_att_dist){
        attractive_force = k_att * max_att_dist * (goal - current_odom)/dist_att;
    }else
    {
        attractive_force = k_att * (goal - current_odom);
    }
    // 排斥力
    push_force = Eigen::Vector3d(0.0, 0, 0);
    for (size_t i = 0; i < latest_local_pcl_.points.size(); ++i) {
        pt = latest_local_pcl_.points[i];
        p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

        double dist_push = (current_odom - p3d).norm();
        if(dist_push > obs_distance)
            continue;
        if(dist_push < min_dist){
            // should be addressed carefully.
            // printf("the distance is very dangerous, dist: %f\n", dist_push);
        }
        obstacles.push_back(p3d);
        double push_gain = k_push * (1/dist_push - 1/obs_distance)* 1.0/(dist_push * dist_push);

        if(dist_att<1.0){
            push_gain *= dist_att;  // to gaurantee to reach the goal.
        }
        
        push_force += push_gain * (current_odom - p3d)/dist_push;
        
    }
    if(obstacles.size() != 0){
        push_force = push_force/double(obstacles.size());
    }
    ROS_INFO("push force: [%f, %f, %f], attractive force: [%f, %f, %f], obs size: %d, obs_dis: %f, k_push: %f", push_force(0), push_force(1), 
    push_force(2), attractive_force(0), attractive_force(1), attractive_force(2), obstacles.size(), obs_distance, k_push);
    
    // 合力
    desired_vel = push_force + attractive_force;

    return true;
}

void APF::init(ros::NodeHandle& nh){
    has_local_map_ = false;

    nh.param("apf/obs_distance", obs_distance, 1.2);
    nh.param("apf/k_push", k_push, 1.5);
    // obs_distance = 2.0;
    min_dist = 0.1;
    // k_push = 1.0;

    max_att_dist = 4;
    k_att = 1.0;
}


}