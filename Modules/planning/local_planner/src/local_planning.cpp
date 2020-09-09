//
// Created by taojiang on 2019/12/9.
//

#include "local_planning.h"

namespace local_planner
{

// 局部规划算法 初始化函数
void LocalPlanningClass::init(ros::NodeHandle& nh)
{
    // set mode
    flight_type_ = FLIGHT_TYPE::MANUAL_GOAL;


    // 发布本节点提示消息
    message_pub = node_.advertise<prometheus_msgs::Message>("/prometheus/message/local_planner", 10);

    // 参数读取
    // 最大速度
    nh.param("planning/max_planning_vel", max_planning_vel, 0.4);
    // 激光雷达模型,0代表3d雷达,1代表2d雷达
    // 3d雷达输入类型为 <sensor_msgs::PointCloud2> 2d雷达输入类型为 <sensor_msgs::LaserScan>
    nh.param("planning/lidar_model", lidar_model, 0);
    // 根据参数 planning/algorithm_mode 选择局部避障算法: 0为APF,1为VFH
    nh.param("planning/algorithm_mode", algorithm_mode, 0);

    // 选择避障算法
    if(algorithm_mode==0){
        local_alg_ptr.reset(new APF);
        local_alg_ptr->init(nh);
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "APF init.");
    }
    else if(algorithm_mode==1)
    {
        local_alg_ptr.reset(new VFH);
        local_alg_ptr->init(nh);
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "VFH init.");
    }
    
        
    // 订阅目标点
    waypoint_sub_ = node_.subscribe("/prometheus/planning/goal", 1, &LocalPlanningClass::waypointCallback, this);

    // 订阅无人机当前位置
    odom_sub_ = node_.subscribe<nav_msgs::Odometry>("/prometheus/planning/odom_world", 10, &LocalPlanningClass::odomCallback, this);

    // 订阅传感器点云信息,该话题名字可在launch文件中任意指定
    if (lidar_model == 0)
    {
        local_point_clound_sub_ = node_.subscribe<sensor_msgs::PointCloud2>("/prometheus/planning/local_pcl", 1, &LocalPlanningClass::localcloudCallback, this);
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Subscribe to <sensor_msgs::PointCloud2>.");
    }else if (lidar_model == 1)
    {
        local_point_clound_sub_ = node_.subscribe<sensor_msgs::LaserScan>("/prometheus/planning/local_pcl", 1, &LocalPlanningClass::laserscanCallback, this);
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Subscribe to <sensor_msgs::LaserScan>.");
    }
    

    // 订阅节点开始运行指令
    swith_sub = node_.subscribe<std_msgs::Bool>("/prometheus/switch/local_planner", 10, &LocalPlanningClass::switchCallback, this);  

    // 发布规划结果 : 期望速度
    px4_pos_cmd_pub = node_.advertise<geometry_msgs::Point>("/prometheus/local_planner/desired_vel", 10);

    // 发布停止紧急指令(无人机离障碍物太近)
    replan_cmd_Pub = node_.advertise<std_msgs::Int8>("/prometheus/planning/stop_cmd", 1);  

    // 定时函数,执行周期为1Hz
    exec_timer_ = node_.createTimer(ros::Duration(1.0), &LocalPlanningClass::execFSMCallback, this, false);

    // 地图初始化
    sensor_msgs::PointCloud2ConstPtr init_local_map(new sensor_msgs::PointCloud2());
    local_map_ptr_ = init_local_map;
    // 状态参数初始化
    trigger_=false;
    have_goal_=false;
    has_point_map_=false;
    have_odom_=false;

    // 规划结果可视化
    visualization_.reset(new PlanningVisualization(nh));
    local_map_marker_Pub   = node_.advertise<visualization_msgs::Marker>("/planning/local_map_marker",  10);  

    // loop
    ros::spin();
}


void LocalPlanningClass::execFSMCallback(const ros::TimerEvent& e){
    static int exect_num=0;
    exect_num++;

    string print_info;
    if(exect_num==5){
        if (!trigger_)
        {   
            print_info = "don't trigger!";
        }

        if(!have_odom_){
            print_info = "don't have odometry!";
        }
            
        if(!has_point_map_)
        {
            print_info = "don't have point cloud! ";
        }
        if(!have_goal_){
            print_info = " wait goal! ";
        }
        exect_num=0;
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, print_info);
    }


    if (!trigger_)
    {   
        return;
    }
    if(!have_odom_){
        return;
    }
        
    if(!has_point_map_)
    {
        return;
    }

    if(!have_goal_){
        return;
    }


    if (lidar_model == 0)
    {
        local_alg_ptr->set_local_map(local_map_ptr_);
    }else if (lidar_model == 1)
    {
        local_alg_ptr->set_local_map_pcl(pcl_ptr);
    }

    local_alg_ptr->set_odom(odom_);
    // 返回值为2时,飞机不安全(距离障碍物太近)
    int planner_state = local_alg_ptr->compute_force(end_pt_, start_pt_, desired_vel);

    // 
    static int fix_pub = 0;
    if (fix_pub==int(2.0/0.05))
    {
        if(planner_state==2){
            // dangerous
            replan.data = 1;
            replan_cmd_Pub.publish(replan);
        } else if(planner_state==1){
            replan.data = 0;
            replan_cmd_Pub.publish(replan);
        }
        fix_pub = 0;
    }else{
        fix_pub++;
    }
    

    if(desired_vel.norm() > max_planning_vel)
    {
        desired_vel = desired_vel / desired_vel.norm() * max_planning_vel;  // the max velocity is max_planning_vel
    }

    if(exect_num==20)
    {
        char sp[100];
        sprintf(sp, "local planning desired vel: [%f, %f, %f]", desired_vel(0), desired_vel(1), desired_vel(2));
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME,sp);
        exect_num=0;
    }
    
    // 发布控制指令
    generate_cmd(desired_vel);
    control_time = ros::Time::now();
    // pos_cmd_pub.publish(cmd);
    if((end_pt_-start_pt_).norm() < 0.05){
        // printf("reach the goal!\n");
    }

    // visualization_->drawPath(kino_path, 0.1, Eigen::Vector4d(1, 0, 0, 1));  // red

}

void LocalPlanningClass::generate_cmd(Eigen::Vector3d desired_vel)
{
    // 发布控制指令
    px4_cmd.x = desired_vel(0);
    px4_cmd.y = desired_vel(1);
    px4_cmd.z = desired_vel(2);
    // px4_cmd.z = 0.0; //高度通道不做控制
    px4_pos_cmd_pub.publish(px4_cmd);

    // 可视化
    visualization_->drawVel(start_pt_, desired_vel * 0.5, Eigen::Vector4d(1, 0.3, 0.3, 1.0), 0);
}

//  the goal is in the world frame. 
void LocalPlanningClass::waypointCallback(const geometry_msgs::PoseStampedConstPtr& msg){

    if (msg->pose.position.z < 0.1)  // the minimal goal height 
        return;

    double /*goal_x, goal_y,*/ goal_z;

        // two mode: 1. manual setting goal from rviz; 2. preset goal in launch file.
    auto conf=[](double v, double min_v, double max_v)->double{
        return v<min_v? min_v:(v>max_v?max_v:v);
    };

    if (flight_type_ == FLIGHT_TYPE::MANUAL_GOAL)
    {
        goal_z = msg->pose.position.z;
        goal_z = conf(msg->pose.position.z, 0.3, 3.5);
        end_pt_ << msg->pose.position.x, msg->pose.position.y, goal_z;
    }
    else if (flight_type_ == FLIGHT_TYPE::PRESET_GOAL)
    {}
    

    char sp[100];
    sprintf(sp, "get waypoint: [ %f, %f, %f]!---", end_pt_(0),
                                                            end_pt_(1), 
                                                            end_pt_(2));

    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME,sp);


    visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));

    end_vel_.setZero();
    have_goal_ = true;

}



void LocalPlanningClass::odomCallback(const nav_msgs::OdometryConstPtr &msg){
    odom_ = *msg;
    odom_.header.frame_id = "map";
    have_odom_ = true;
    start_pt_ << odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z; 
}

void LocalPlanningClass::laserscanCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
    /* need odom_ for center radius sensing */
    if (!have_odom_) {
        // ROS_INFO("local point cloud: --- no odom!---");
        return;
    }

    sensor_msgs::LaserScan::ConstPtr _laser_scan;

    _laser_scan = msg;

    pcl::PointCloud<pcl::PointXYZ> _pointcloud;

    _pointcloud.clear();
    pcl::PointXYZ newPoint;
    double newPointAngle;

    int beamNum = _laser_scan->ranges.size();
    for (int i = 0; i < beamNum; i++)
    {
        newPointAngle = _laser_scan->angle_min + _laser_scan->angle_increment * i;
        newPoint.x = _laser_scan->ranges[i] * cos(newPointAngle);
        newPoint.y = _laser_scan->ranges[i] * sin(newPointAngle);
        newPoint.z = odom_.pose.pose.position.z;
        _pointcloud.push_back(newPoint);
    }

    pcl_ptr = _pointcloud.makeShared();

    latest_local_pcl_ = *pcl_ptr;
    has_point_map_ = true;


    visualization_msgs::Marker m;
    getOccupancyMarker(m, 0, Eigen::Vector4d(0, 0.5, 0.5, 1.0));
    local_map_marker_Pub.publish(m);
}

//  the local cloud is in the local frame. 
void LocalPlanningClass::localcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    /* need odom_ for center radius sensing */
    if (!have_odom_) {
        // ROS_INFO("local point cloud: --- no odom!---");
        return;
    }
    // printf("receive the local point cloud\n");
    local_map_ptr_ = msg;

    // ros::Time begin_load_point_cloud = ros::Time::now();
    pcl::fromROSMsg(*msg, latest_local_pcl_);
    has_point_map_ = true;

    visualization_msgs::Marker m;
    getOccupancyMarker(m, 0, Eigen::Vector4d(0, 0.5, 0.5, 1.0));
    local_map_marker_Pub.publish(m);

}



void LocalPlanningClass::getOccupancyMarker(visualization_msgs::Marker &m, int id, Eigen::Vector4d color) {
    // 坐标系可能有问题
    m.header.frame_id = "world";
    m.id = id;
    m.type = visualization_msgs::Marker::CUBE_LIST;
    m.action = visualization_msgs::Marker::MODIFY;
    m.scale.x = 0.1; // resolustion
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.color.a = color(3);
    m.color.r = color(0);
    m.color.g = color(1);
    m.color.b = color(2);

    // iterate the map
    pcl::PointXYZ pt;
    Eigen::Matrix<double, 3, 3> rotation_mat_local_to_global = Eigen::Quaterniond(odom_.pose.pose.orientation.w,
                                                                                                                                                                            odom_.pose.pose.orientation.x,
                                                                                                                                                                            odom_.pose.pose.orientation.y,
                                                                                                                                                                            odom_.pose.pose.orientation.z).toRotationMatrix();
    Eigen::Matrix<double, 3, 1> position_world_to_local (odom_.pose.pose.position.x,
                                                                                                                    odom_.pose.pose.position.y,
                                                                                                                    odom_.pose.pose.position.z);
    Eigen::Matrix<double, 3, 1> pointd;
    for (size_t i = 0; i < latest_local_pcl_.points.size(); ++i) {
        pt = latest_local_pcl_.points[i];
        geometry_msgs::Point p;
        pointd(0) = pt.x;
        pointd(1) = pt.y;
        pointd(2) = pt.z;
        // transform to world frame
        pointd =  rotation_mat_local_to_global * pointd + position_world_to_local;

        p.x = pointd(0);
        p.y = pointd(1);
        p.z=pointd(2);
        // p.x = pt.x;
        // p.y = pt.y;
        // p.z = pt.z;
        m.points.push_back(p);

    }
}

void LocalPlanningClass::switchCallback(const std_msgs::Bool::ConstPtr &msg){
    trigger_= msg->data;

    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "local planner trigger.");
}


}


