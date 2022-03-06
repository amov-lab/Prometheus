#include "apf.h"

namespace LocalPlannerNS
{

    void APF::init(ros::NodeHandle &nh)
    {
        
        // 【参数】障碍物膨胀距离
        nh.param("apf/inflate_distance", inflate_distance, 0.2);    
        // 【参数】感知障碍物距离
        nh.param("apf/sensor_max_range", sensor_max_range, 5.0);    
        // 【参数】障碍物排斥力增益
        nh.param("apf/k_repulsion", k_repulsion, 0.8);                        
        // 【参数】目标吸引力增益
        nh.param("apf/k_attraction", k_attraction, 0.4);                          
        // 【参数】最小避障距离
        nh.param("apf/min_dist", min_dist, 0.2);                     
        // 【参数】最大吸引距离（相对于目标）
        nh.param("apf/max_att_dist", max_att_dist, 5.0);             
        // 【参数】地面高度
        nh.param("apf/ground_height", ground_height, 0.1);           
        // 【参数】地面安全距离，低于地面高度，则不考虑该点的排斥力
        nh.param("apf/ground_safe_height", ground_safe_height, 0.2); 
        // 【参数】安全停止距离
        nh.param("apf/safe_distance", safe_distance, 0.15);          
        has_local_map_ = false;
    }

    // 设置局部地图
    void APF::set_local_map(sensor_msgs::PointCloud2ConstPtr &local_map_ptr)
    {
        local_map_ptr_ = local_map_ptr;
        pcl::fromROSMsg(*local_map_ptr, latest_local_pcl_);
        has_local_map_ = true;
    }

    // 设置局部点云
    void APF::set_local_map_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_ptr)
    {
        latest_local_pcl_ = *pcl_ptr;
        has_local_map_ = true;
    }

    // 设置本地位置
    void APF::set_odom(nav_msgs::Odometry cur_odom)
    {
        current_odom = cur_odom;
        has_odom_ = true;
    }

    // 计算输出
    int APF::compute_force(Eigen::Vector3d &goal, Eigen::Vector3d &desired_vel)
    {
        // 规划器返回的状态值：0 for not init; 1 for safe; 2 for dangerous
        int local_planner_state = 0;
        int safe_cnt = 0;

        if (!has_local_map_ || !has_odom_)
            return 0;

        if ((int)latest_local_pcl_.points.size() == 0)
            return 0;

        if (isnan(goal(0)) || isnan(goal(1)) || isnan(goal(2)))
            return 0;

        //　当前位置
        Eigen::Vector3d current_pos;
        current_pos[0] = current_odom.pose.pose.position.x;
        current_pos[1] = current_odom.pose.pose.position.y;
        current_pos[2] = current_odom.pose.pose.position.z;

        ros::Time begin_collision = ros::Time::now();

        // 引力
        Eigen::Vector3d uav2goal = goal - current_pos;
        // 不考虑高度影响
        uav2goal(2) = 0.0;
        double dist_att = uav2goal.norm();
        if (dist_att > max_att_dist)
        {
            uav2goal = max_att_dist * uav2goal / dist_att;
        }
        //　计算吸引力
        attractive_force = k_attraction * uav2goal;

        // 排斥力
        double uav_height = current_odom.pose.pose.position.z;
        repulsive_force = Eigen::Vector3d(0.0, 0.0, 0.0);

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
            double point_height_global = uav_height + p3d(2);
            if (fabs(point_height_global) < ground_height)
                continue;

            //　超出最大感知距离，则不考虑该点的排斥力
            double dist_push = (current_pos_local - p3d).norm();
            if (dist_push > sensor_max_range || isnan(dist_push))
                continue;

            //　考虑膨胀距离
            dist_push = dist_push - inflate_distance;

            // 如果当前的观测点中，包含小于安全停止距离的点，进行计数
            if (dist_push < safe_distance)
            {
                safe_cnt++;
            }

            //　小于最小距离时，则增大该距离，从而增大排斥力
            if (dist_push < min_dist)
            {
                dist_push = min_dist / 1.5;
            }

            obstacles.push_back(p3d);
            double push_gain = k_repulsion * (1 / dist_push - 1 / sensor_max_range) * 1.0 / (dist_push * dist_push);

            if (dist_att < 1.0)
            {
                push_gain *= dist_att; // to gaurantee to reach the goal.
            }

            repulsive_force += push_gain * (current_pos_local - p3d) / dist_push;
        }

        //　平均排斥力
        if (obstacles.size() != 0)
        {
            repulsive_force = repulsive_force / obstacles.size();
        }

        Eigen::Quaterniond cur_rotation_local_to_global(current_odom.pose.pose.orientation.w,
                                                        current_odom.pose.pose.orientation.x,
                                                        current_odom.pose.pose.orientation.y,
                                                        current_odom.pose.pose.orientation.z);

        Eigen::Matrix<double, 3, 3> rotation_mat_local_to_global = cur_rotation_local_to_global.toRotationMatrix();

        repulsive_force = rotation_mat_local_to_global * repulsive_force;

        // 合力
        desired_vel = repulsive_force + attractive_force;

        // 由于定高飞行，设置期望Z轴速度为0
        desired_vel[2] = 0.0;

        // 如果不安全的点超出，
        if (safe_cnt > 10)
        {
            local_planner_state = 2; //成功规划，但是飞机不安全
        }
        else
        {
            local_planner_state = 1; //成功规划， 安全
        }

        static int exec_num = 0;
        exec_num++;

        // 此处改为根据循环时间计算的数值
        if (exec_num == 50)
        {
            cout << GREEN << NODE_NAME << "APF calculate take: " << (ros::Time::now() - begin_collision).toSec() << " [s] " << TAIL << endl;
            exec_num = 0;
        }

        return local_planner_state;
    }

} // namespace LocalPlannerNS
