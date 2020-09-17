#include "global_planning.h"

namespace global_planner
{

void GlobalPlanner::init(ros::NodeHandle& nh)
{
    // 读取参数
    nh.param("planning/safe_distance", safe_distance, 0.25);
    nh.param("planning/is_2D", is_2D, 0);  // 1代表2D平面规划及搜索,0代表3D
    nh.param("planning/2D_fly_height", fly_height, 1.5);  // 2D规划时,定高高度
    
    //　订阅开始标志
    swith_sub                   = node_.subscribe<std_msgs::Bool>("/prometheus/switch/global_planner", 10, &GlobalPlanner::switchCallback, this);  
    // 订阅目标点
    waypoint_sub_               = node_.subscribe("/prometheus/planning/goal", 1, &GlobalPlanner::waypointCallback, this);
    //　订阅无人机当前位置
    odom_sub_                   = node_.subscribe<nav_msgs::Odometry>("/prometheus/planning/odom_world", 10, &GlobalPlanner::odomCallback, this);
    //　订阅全局点云
    global_point_clound_sub_    = node_.subscribe<sensor_msgs::PointCloud2>("/prometheus/planning/global_pcl", 1, &GlobalPlanner::globalcloudCallback, this);
    //　发布规划路径
    path_cmd_Pub                = node_.advertise<nav_msgs::Path>("/prometheus/global_planner/path_cmd",  10);  
    //　发布重规划指令，即无人机停止指令
    replan_cmd_Pub              = node_.advertise<std_msgs::Int8>("/prometheus/planning/stop_cmd", 1); 
    // 发布标记
    global_map_marker_Pub       = node_.advertise<visualization_msgs::Marker>("/prometheus/planning/global_map_marker",  10); 
    //　定时安全检查 
    safety_timer_               = node_.createTimer(ros::Duration(2.0), &GlobalPlanner::safetyCallback, this);
    //　定时执行规划算法
    exec_timer_                 = node_.createTimer(ros::Duration(1.5), &GlobalPlanner::execCallback, this); 
    // 发布debug消息
    message_pub                 = node_.advertise<prometheus_msgs::Message>("/prometheus/message/global_planner", 10);

    // kinodynamic_Astar
    KinodynamicAstar_ptr.reset(new KinodynamicAstar);
    KinodynamicAstar_ptr->setParam(nh);
    KinodynamicAstar_ptr->init(nh);

    //　初始化全局点云
    sensor_msgs::PointCloud2ConstPtr init_global_map(new sensor_msgs::PointCloud2());
    global_map_ptr_ = init_global_map;

    // init visualization
    visualization_.reset(new PlanningVisualization(nh));
     
    //　初始化各类变量
    flight_type_  = FLIGHT_TYPE::MANUAL_GOAL;
    trigger_ = false;

    ROS_INFO("---KinodynamicAstar　init!---");
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "kinodynamic_Astar init.");
    ros::spin(); 
}

void GlobalPlanner::waypointCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME,"Get a new goal point");

    // two mode: 1. manual setting goal from rviz; 2. preset goal in launch file.
    if (flight_type_ == FLIGHT_TYPE::MANUAL_GOAL)
    {
        end_pt_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        
        if (is_2D == 1)
        {
            end_pt_(2) = fly_height;
        }

        //　终点的速度设置为０
        end_vel_.setZero();
    }
    else if (flight_type_ == FLIGHT_TYPE::PRESET_GOAL)
    {
        end_pt_ << 0,0,1;
        end_vel_.setZero();
    }

    have_goal_ = true;
    visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
}

void GlobalPlanner::execCallback(const ros::TimerEvent& e)
{
    static int exec_num=0;
    exec_num++;
    
    if(exec_num >= 2)
    {
        if(!trigger_)
        {
            exect_msg = "don't trigger!";
        }

        if(!have_odom_)
        {
            exect_msg = "don't have odometry!";
        }
            
        if(!has_point_map_)
        {
            exect_msg = "don't have point cloud!";
        }
        if(!have_goal_)
        {
            exect_msg = "wait goal!";
        }

        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME,exect_msg);
        exec_num=0;
    }

    if(!trigger_ || !have_odom_ || !has_point_map_ || !have_goal_)
    {
        return;
    }

    KinodynamicAstar_ptr->reset();
    int astar_state;
    //　init 和　dynamic 怎么赋值　有什么意义
    bool init = false;
    bool dynamic = false;
    double time_start = 0;

    astar_state = KinodynamicAstar_ptr->search(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_, init, dynamic, time_start);

    if(astar_state==KinodynamicAstar::NO_PATH)
    {
        pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "a star find no path, please reset the goal!");
    }
    else
    {
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "astart find path success!");
        //std::vector<Eigen::Vector3d> KinodynamicAstar::getKinoTraj(double delta_t)
        std::vector<Eigen::Vector3d> KinoTraj = KinodynamicAstar_ptr->getKinoTraj(0.0);
        //visualization_->drawPath(A_star_path, 0.1,Eigen::Matrix<double, 4, 1>(1.0, 0, 0, 1), 1);
        //generate_CMD(A_star_path);
    }

}

void GlobalPlanner::generate_CMD(std::vector<Eigen::Vector3d> path)
{
    A_star_path_cmd.header.frame_id = "world";
    A_star_path_cmd.header.stamp = ros::Time::now();
    // printf("Path:  \n");
    A_star_path_cmd.poses.clear();
    for (int i=0; i<path.size(); ++i){
        geometry_msgs::PoseStamped path_i_pose;
        path_i_pose .header.frame_id = "world";
        path_i_pose.pose.position.x = path[i](0);
        path_i_pose.pose.position.y = path[i](1);
        path_i_pose.pose.position.z = path[i](2);
        A_star_path_cmd.poses.push_back(path_i_pose);
    }
    control_time = ros::Time::now();
    replan.data = 0;
    path_cmd_Pub.publish(A_star_path_cmd);
}

void GlobalPlanner::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    odom_ = *msg;
    have_odom_ = true;
    start_pt_ << odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z; 
    start_vel_ << odom_.twist.twist.linear.x, odom_.twist.twist.linear.y, odom_.twist.twist.linear.z; 
    start_acc_ << 0.0, 0.0, 0.0;

    if (is_2D == 1)
    {
        start_pt_(2) = fly_height;
    }
}

void GlobalPlanner::globalcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    /* need odom_ for center radius sensing */
    if (!have_odom_) {
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "global point cloud: --- no odom!---");
        return;
    }

    has_point_map_ = true;
    pcl::fromROSMsg(*msg, latest_global_pcl_);
    global_map_ptr_ = msg;

    // 二维平面规划
    if (is_2D == 1)
    {
        //global_map_ptr_
    }

    KinodynamicAstar_ptr->setEnvironment(global_map_ptr_);

    visualization_msgs::Marker m;
    getOccupancyMarker(m, 0, Eigen::Vector4d(0, 0.5, 0.5, 1.0));
    global_map_marker_Pub.publish(m);
}

void GlobalPlanner::getOccupancyMarker(visualization_msgs::Marker &m, int id, Eigen::Vector4d color) 
{
    m.header.frame_id = "map";
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
    for (size_t i = 0; i < latest_global_pcl_.points.size(); ++i) {
        pt = latest_global_pcl_.points[i];

        geometry_msgs::Point p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        m.points.push_back(p);
    }
}

void GlobalPlanner::safetyCallback(const ros::TimerEvent& e)
{

    Eigen::Vector3d cur_pos(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);
    bool is_safety = KinodynamicAstar_ptr->check_safety(cur_pos, safe_distance);

    if(!is_safety )
    {
        pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "replan cmd.");
        replan.data = 1;
    }
    else
    {
        replan.data = 0;
    }
    replan_cmd_Pub.publish(replan);
}

void GlobalPlanner::switchCallback(const std_msgs::Bool::ConstPtr &msg)
{
    trigger_= msg->data;
}


}