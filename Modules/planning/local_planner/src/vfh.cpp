#include "vfh.h"
#include "math.h"

namespace local_planner
{
// get the map
void VFH::set_local_map(sensor_msgs::PointCloud2ConstPtr &local_map_ptr){
    local_map_ptr_ = local_map_ptr;
    ros::Time begin_load_point_cloud = ros::Time::now();

    pcl::fromROSMsg(*local_map_ptr, latest_local_pcl_);

    begin_update_map = ros::Time::now();
    has_local_map_=true;
}


void VFH::set_odom(nav_msgs::Odometry cur_odom){
    cur_odom_ = cur_odom;
}

int VFH::compute_force(Eigen::Matrix<double, 3, 1> &goal, Eigen::Matrix<double, 3, 1> current_odom, Eigen::Vector3d &desired_vel){
    int local_planner_state=0;  // 0 for not init; 1 for safe; 2 for dangerous
    int safe_cnt=0;
    // clear the Hdata
    for(int i =0; i<Hcnt; i++){
        Hdata[i]=0;
    }

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
    Eigen::Vector3d p3d_gloabl_rot;
    ros::Time begin_collision = ros::Time::now();
    // ROS_INFO("point size: %d", latest_local_pcl_.points.size());

    // 吸引
    Eigen::Vector3d odom2goal = goal - current_odom;
    // 不考虑高度影响
    odom2goal(2) = 0.0;
    double dist_att = odom2goal.norm();
    double goal_heading = atan2(odom2goal(1), odom2goal(0));
    
    if(dist_att > max_att_dist){
        dist_att = max_att_dist;
        // odom2goal = odom2goal/dist_att * max_att_dist;
    }else
    {    }


    // 排斥力
    Eigen::Quaterniond cur_rotation_local_to_global(cur_odom_.pose.pose.orientation.w, 
                                                                                                            cur_odom_.pose.pose.orientation.x,  
                                                                                                            cur_odom_.pose.pose.orientation.y,  
                                                                                                            cur_odom_.pose.pose.orientation.z); 

    // printf("odom q:[%f, %f, %f, %f]\n", cur_odom_.pose.pose.orientation.w, 
    //                                                                                                         cur_odom_.pose.pose.orientation.x,  
    //                                                                                                         cur_odom_.pose.pose.orientation.y,  
    //                                                                                                         cur_odom_.pose.pose.orientation.z);
    // Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle(2),Vector3d::UnitX()));
    // Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle(1),Vector3d::UnitY()));
    // Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle(0),Vector3d::UnitZ()));
 
    // Eigen::Matrix3d rotation_matrix;
    // rotation_matrix=yawAngle*pitchAngle*rollAngle;


    Eigen::Matrix<double,3,3> rotation_mat_local_to_global = cur_rotation_local_to_global.toRotationMatrix();
    Eigen::Vector3d eulerAngle_yrp = rotation_mat_local_to_global.eulerAngles(2, 1, 0);
    rotation_mat_local_to_global = Eigen::AngleAxisd(eulerAngle_yrp(0), Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // double uav_height = cur_odom_.pose.pose.position.z;
    // push_force = Eigen::Vector3d(0.0, 0, 0);

    Eigen::Matrix<double, 3,1> current_odom_local(0.0, 0,0);  // in local frame
    // Eigen::Matrix<double, 3,1> current_odom_local = current_odom;

    for (size_t i = 0; i < latest_local_pcl_.points.size(); ++i) {
        pt = latest_local_pcl_.points[i];
        p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;
        // 3D lidar
         if(isnan(p3d(2)) || p3d(2)<-0.1){
             continue;
         }
        // 不考虑高度的影响
        p3d(2) = 0.0;
        // rotation the local point (heading)
        p3d_gloabl_rot = rotation_mat_local_to_global * p3d; 

        if(isIgnored(p3d_gloabl_rot(0), p3d_gloabl_rot(1), p3d_gloabl_rot(2),obs_distance)){
            continue;
        }

        double obs_dist = p3d_gloabl_rot.norm();

        // // remove the ground point 
        // double point_height_global = uav_height+p3d(2);
        // if(fabs(point_height_global)<ground_height)
        //     continue;

        double obs_angle = atan2(p3d_gloabl_rot(1), p3d_gloabl_rot(0));
        double angle_range;
        if(obs_dist>inflate_and_safe_distance){
            angle_range = asin(inflate_and_safe_distance/obs_dist);
        }else if (obs_dist<=inflate_and_safe_distance)
        {
            safe_cnt++;  // 非常危险
            continue;
        }

        double obstacle_cost = obstacle_weight * (1/obs_dist - 1/obs_distance)* 1.0/(obs_dist * obs_dist) /* * (1/obs_dist)*/;
        // printf("vfh: obs_dist: %f, obs_distance: %f, obstacle_cost: %f, angle_range: %f, obs_angle: %f\n", obs_dist, obs_distance, obstacle_cost, angle_range, obs_angle);
        generate_voxel_data(obs_angle, angle_range, obstacle_cost);

        // dist_push = dist_push - inflate_distance;

        obstacles.push_back(p3d);
        // double push_gain = k_push * (1/dist_push - 1/obs_distance)* 1.0/(dist_push * dist_push);

        // std::cout<<"dist_push" << dist_push << std::endl;
        // std::cout << "push_gain: " << push_gain << std::endl;
        // std::cout << "p3d: " << p3d << std::endl;
        // push_force += push_gain * (current_odom_local - p3d)/dist_push;
    }

    if(obstacles.size() != 0){
        // printf("obstacle size: %d\n", obstacles.size());
        // push_force=push_force/obstacles.size();
    }

    // if(uav_height<ground_safe_height){
    //         // printf("[compute_force]: near the ground, the height%f \n", uav_height);
    //         push_force = push_force + Eigen::Matrix<double, 3, 1>(0, 0, (ground_safe_height-uav_height)*3.0);
    // }

    // push_force = push_force;

    // ROS_INFO("push force: [%f, %f, %f], attractive force: [%f, %f, %f], obs size: %d, obs_dis: %f, k_push: %f", push_force(0), push_force(1), 
    // push_force(2), attractive_force(0), attractive_force(1), attractive_force(2), obstacles.size(), obs_distance, k_push);
    
    for(int i=0; i<Hcnt; i++){
        // Hdata;
        double angle_i = find_angle(i);
        double prev_cost = 0;
        if(is_prev){
            double angle_er  = angle_error(angle_i, prev_heading);
            prev_cost = prevWeight * angle_er;
        }

        double goal_cost = 0;
        {
            double angle_er = angle_error(angle_i, goal_heading);
            float goal_gain;
            if(dist_att>3.0) {goal_gain = 3.0;}
            else if(dist_att<0.5) {goal_gain = 0.5;} 
            else{goal_gain = dist_att;}

            goal_cost = goalWeight * angle_er * goal_gain;
        }
        Hdata[i] += (prev_cost + goal_cost);
    }

    int best_ind = find_optimization_path();   // direction 
    double best_heading  = find_angle(best_ind);

    double vel_norm = dist_att/2; //与距离终点距离有关, 0.2m， 32到达
    if(vel_norm>limit_v_norm){
        vel_norm = limit_v_norm;
    }
    desired_vel(0) = cos(best_heading)*vel_norm;
    desired_vel(1) = sin(best_heading)*vel_norm;
    // 处理高度
    desired_vel(2) = 0.0;
    // printf("vfh: angle: %f,  vnorm: %f , obstacle_weight: %f\n", best_heading, vel_norm, obstacle_weight);

    // 如果不安全的点超出，
    if(safe_cnt>5){
        local_planner_state = 2;  //成功规划，但是飞机不安全
    }else{
        is_prev = true;
        prev_heading = best_heading;
        local_planner_state =1;  //成功规划， 安全
    }

    return local_planner_state;
}
// 寻找最小
int VFH::find_optimization_path(void){
    int bset_ind = 10000;
    double best_cost = 100000;
    for(int i=0; i<Hcnt; i++){
        if(Hdata[i]<best_cost){
            best_cost = Hdata[i];
            bset_ind = i;
        }
    }
    return bset_ind;
}

void VFH::init(ros::NodeHandle& nh){
    has_local_map_ = false;
    nh.param("vfh/inflate_distance", inflate_distance, 0.20);  // 感知障碍物距离
    nh.param("vfh/obs_distance", obs_distance, 3.0);  // 感知障碍物距离

    // nh.param("apf/min_dist", min_dist, 0.2);                            // 最小壁障距离
    nh.param("vfh/max_att_dist", max_att_dist, 5.0);             // 最大吸引距离
    // nh.param("apf/ground_height", ground_height, 0.1);  // 地面高度
    // nh.param("apf/ground_safe_height", ground_safe_height, 0.2);  // 地面安全距离
    nh.param("vfh/safe_distance", safe_distance, 0.2); // 安全停止距离

    nh.param("vfh/goalWeight", goalWeight, 0.2); // 目标权重
    nh.param("vfh/prevWeight", prevWeight, 0.0); // 光滑权重
    nh.param("vfh/obstacle_weight", obstacle_weight, 0.0); // 障碍物权重
    
    nh.param("vfh/limit_v_norm", limit_v_norm, 0.4); // 极限速度

    inflate_and_safe_distance = safe_distance + inflate_distance;
    is_prev = false;

    // Hres
    nh.param("vfh/h_res", Hcnt, 150); // 直方图 个数
    Hres = 2*M_PI/Hcnt;
    Hdata = new double[Hcnt]();
    for(int i(0); i<Hcnt; i++){
        Hdata[i] =0.0;       
    }
}

bool VFH::isIgnored(float x, float y, float z, float ws){
    z = 0;
    if(isnan(x)||isnan(y)||isnan(z))
        return true;

    if(x*x+y*y+z*z>ws)
        return true;
    
    return false;
}

void VFH::generate_voxel_data(double angle_cen, double angle_range, double val)  // set the map obstacle into the H data
{
    double angle_max = angle_cen + angle_range;
    double angle_min = angle_cen - angle_range;
    int cnt_min = find_Hcnt(angle_min);
    int cnt_max = find_Hcnt(angle_max);
    if(cnt_min>cnt_max){
        for(int i=cnt_min; i<Hcnt; i++){
            Hdata[i] =+ val;
        }
        for(int i=0;i<cnt_max; i++){
            Hdata[i] +=val;
        }
    }else if(cnt_max>=cnt_min){
    for(int i=cnt_min; i<=cnt_max; i++){
        Hdata[i] += val;
    }
    }
     
}

// angle: deg
int VFH::find_Hcnt(double angle){
    if(angle<0){
        angle += 2 * M_PI;
    }
    if(angle>2*M_PI){
        angle -=2*M_PI;
    }
    int cnt = floor(angle/Hres);
    return cnt;
}

double VFH::find_angle(int cnt){
    double angle = (cnt +0.5)* Hres;
    return angle;
}

double VFH::angle_error(double angle1, double angle2){
    double angle_er = angle1 - angle2;
    while(angle_er>M_PI){
        angle_er = angle_er - 2*M_PI;
    }

    while (angle_er<-M_PI)
    {
        angle_er = angle_er + 2*M_PI;
    }
    angle_er = fabs(angle_er);
    return angle_er;

}

}