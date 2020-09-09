#include "apf.h"

namespace local_planner
{

void APF::set_local_map(sensor_msgs::PointCloud2ConstPtr &local_map_ptr){
    local_map_ptr_ = local_map_ptr;
    ros::Time begin_load_point_cloud = ros::Time::now();

    pcl::fromROSMsg(*local_map_ptr, latest_local_pcl_);

    begin_update_map = ros::Time::now();
    has_local_map_=true;
}

void APF::set_local_map_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_ptr)
{
    latest_local_pcl_ = *pcl_ptr;
    has_local_map_=true;
}

void APF::set_odom(nav_msgs::Odometry cur_odom){
    cur_odom_ = cur_odom;
}


int APF::compute_force(Eigen::Matrix<double, 3, 1> &goal, Eigen::Matrix<double, 3, 1> current_odom, Eigen::Vector3d &desired_vel){
    int local_planner_state=0;  // 0 for not init; 1for safe; 2 for dangerous
    int safe_cnt=0;

    if(!has_local_map_)
        return 0;

    if ((int)latest_local_pcl_.points.size() == 0) 
        return 0;

    if (isnan(current_odom(0)) || isnan(current_odom(1)) || isnan(current_odom(2)))
        return 0;

    if (isnan(goal(0)) || isnan(goal(1)) || isnan(goal(2)))
        return 0;

    vector<Eigen::Vector3d> obstacles;
    pcl::PointXYZ pt;
    Eigen::Vector3d p3d;
    ros::Time begin_collision = ros::Time::now();
    // ROS_INFO("point size: %d", latest_local_pcl_.points.size());

    // 引力
    Eigen::Vector3d odom2goal = goal - current_odom;
    // 不考虑高度影响
    odom2goal(2) = 0.0;
    double dist_att = odom2goal.norm();
    if(dist_att > max_att_dist){
        attractive_force = k_att * max_att_dist * odom2goal/dist_att;
    }else
    {
        attractive_force = k_att * odom2goal;
    }

    // 排斥力
    double uav_height = cur_odom_.pose.pose.position.z;
    push_force = Eigen::Vector3d(0.0, 0, 0);
    for (size_t i = 0; i < latest_local_pcl_.points.size(); ++i) {
        pt = latest_local_pcl_.points[i];
        p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;
        // 不考虑高度的影响
        p3d(2) = 0.0;
        

        Eigen::Matrix<double, 3,1> current_odom_local(0.0, 0,0);
        // Eigen::Matrix<double, 3,1> current_odom_local = current_odom;
        // remove the ground point 
        double point_height_global = uav_height+p3d(2);
        if(fabs(point_height_global)<ground_height)
            continue;

        double dist_push = (current_odom_local - p3d).norm();
        if(dist_push > obs_distance || isnan(dist_push))
            continue;

        dist_push = dist_push - inflate_distance;
        // 如果当前的观测点中，包含小于安全停止距离的点，进行计数
        if(dist_push < safe_distance){
            safe_cnt++;
        }
        
        if(dist_push < min_dist){
            // should be addressed carefully.
            // printf("the distance is very dangerous, dist: %f\n", dist_push);
            dist_push = min_dist /1.5;
        }

        obstacles.push_back(p3d);
        double push_gain = k_push * (1/dist_push - 1/obs_distance)* 1.0/(dist_push * dist_push);

        if(dist_att<1.0){
            push_gain *= dist_att;  // to gaurantee to reach the goal.
        }
        // std::cout<<"dist_push" << dist_push << std::endl;
        // std::cout << "push_gain: " << push_gain << std::endl;
        // std::cout << "p3d: " << p3d << std::endl;
        push_force += push_gain * (current_odom_local - p3d)/dist_push;
    }

    if(obstacles.size() != 0){
        // printf("obstacle size: %d\n", obstacles.size());
        push_force=push_force/obstacles.size();
    }

    Eigen::Quaterniond cur_rotation_local_to_global(cur_odom_.pose.pose.orientation.w, 
                                                                                                            cur_odom_.pose.pose.orientation.x,  
                                                                                                            cur_odom_.pose.pose.orientation.y,  
                                                                                                            cur_odom_.pose.pose.orientation.z); 

    // printf("odom q:[%f, %f, %f, %f]\n", cur_odom_.pose.pose.orientation.w, 
    //                                                                                                         cur_odom_.pose.pose.orientation.x,  
    //                                                                                                         cur_odom_.pose.pose.orientation.y,  
    //                                                                                                         cur_odom_.pose.pose.orientation.z);

    Eigen::Matrix<double,3,3> rotation_mat_local_to_global = cur_rotation_local_to_global.toRotationMatrix();


    push_force = rotation_mat_local_to_global * push_force; 

    if(uav_height<ground_safe_height){
            // printf("[compute_force]: near the ground, the height%f \n", uav_height);
            push_force = push_force + Eigen::Matrix<double, 3, 1>(0, 0, (ground_safe_height-uav_height)*3.0);
    }

    push_force = push_force;

    // ROS_INFO("push force: [%f, %f, %f], attractive force: [%f, %f, %f], obs size: %d, obs_dis: %f, k_push: %f", push_force(0), push_force(1), 
    // push_force(2), attractive_force(0), attractive_force(1), attractive_force(2), obstacles.size(), obs_distance, k_push);
    
    // 合力
    desired_vel = push_force + attractive_force;
    // 处理高度
    desired_vel(2) = 0.0;
    // 如果不安全的点超出，
    if(safe_cnt>5){
        local_planner_state = 2;  //成功规划，但是飞机不安全
    }else{
        local_planner_state =1;  //成功规划， 安全
    }

    return local_planner_state;
}

void APF::init(ros::NodeHandle& nh){
    has_local_map_ = false;

    nh.param("apf/inflate_distance", inflate_distance, 0.20);  // 感知障碍物距离
    nh.param("apf/obs_distance", obs_distance, 2.5);  // 感知障碍物距离
    nh.param("apf/k_push", k_push, 0.8);                         // 推力增益
    nh.param("apf/k_att", k_att, 0.4);                                  // 引力增益
    nh.param("apf/min_dist", min_dist, 0.2);                            // 最小壁障距离
    nh.param("apf/max_att_dist", max_att_dist, 5.0);             // 最大吸引距离
    nh.param("apf/ground_height", ground_height, 0.1);  // 地面高度
    nh.param("apf/ground_safe_height", ground_safe_height, 0.2);  // 地面安全距离
    nh.param("apf/safe_distance", safe_distance, 0.15); // 安全停止距离

}


}
