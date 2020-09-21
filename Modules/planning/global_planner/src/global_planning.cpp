#include "global_planning.h"

namespace global_planner
{


void GlobalPlanner::init(ros::NodeHandle& nh){
    // global variable
    message_pub = node_.advertise<prometheus_msgs::Message>("/prometheus/message/global_planner", 10);
    // safe_distance
    nh.param("planning/safe_distance", safe_distance, 0.25);
    nh.param("astar/is_2D", is_2D, 0);  // 1代表2D平面规划及搜索,0代表3D
    nh.param("astar/2D_fly_height", fly_height, 1.5);  // 2D规划时,定高高度
    // set algorithm
    sensor_msgs::PointCloud2ConstPtr init_global_map(new sensor_msgs::PointCloud2());
    global_map_ptr_ = init_global_map;

    // init visualization
    ROS_INFO("---init visualization!---");
    visualization_.reset(new PlanningVisualization(nh));
    string point_map_name;

    /* ---------- callback ---------- */
    ROS_INFO("---init sub and pub!---");
    waypoint_sub_ = node_.subscribe("/prometheus/planning/goal", 1, &GlobalPlanner::waypointCallback, this);

    odom_sub_ = node_.subscribe<nav_msgs::Odometry>("/prometheus/planning/odom_world", 10, &GlobalPlanner::odomCallback, this);

    global_point_clound_sub_ = node_.subscribe<sensor_msgs::PointCloud2>("/prometheus/planning/global_pcl", 1, &GlobalPlanner::globalcloudCallback, this);

    // publish 
    global_map_marker_Pub   = node_.advertise<visualization_msgs::Marker>("/prometheus/planning/global_map_marker",  10);  
    safety_timer_ = node_.createTimer(ros::Duration(2.0), &GlobalPlanner::safetyCallback, this);
    exec_timer_ = node_.createTimer(ros::Duration(1.5), &GlobalPlanner::execCallback, this);        // 每3秒执行一次A*

    path_cmd_Pub   = node_.advertise<nav_msgs::Path>("/prometheus/global_planner/path_cmd",  10);  
    replan_cmd_Pub = node_.advertise<std_msgs::Int8>("/prometheus/planning/stop_cmd", 1);  

    swith_sub = node_.subscribe<std_msgs::Bool>("/prometheus/switch/global_planner", 10, &GlobalPlanner::switchCallback, this);  
    
    // a* algorithm
    Astar_ptr.reset(new Astar);
    Astar_ptr->setParam(nh);
    Astar_ptr->init(nh);

    flight_type_  = FLIGHT_TYPE::MANUAL_GOAL;
    trigger_ = false;

    ros::spin(); 
}


void GlobalPlanner::waypointCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME,"Get a new goal point");

    double /*goal_x, goal_y,*/ goal_z;

    if (is_2D == 1)
    {
        goal_z = fly_height;
    }

    // two mode: 1. manual setting goal from rviz; 2. preset goal in launch file.
    if (flight_type_ == FLIGHT_TYPE::MANUAL_GOAL)
    {
        end_pt_ << msg->pose.position.x, msg->pose.position.y, goal_z;
    }
    else if (flight_type_ == FLIGHT_TYPE::PRESET_GOAL)
    {
        end_pt_ << 0,0,1;
    }

    ROS_INFO("---global planning_: get waypoint: [ %f, %f, %f]!---", end_pt_(0),
                                                            end_pt_(1), 
                                                            end_pt_(2));

    visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));

    end_vel_.setZero();
    have_goal_ = true;

}

void GlobalPlanner::execCallback(const ros::TimerEvent& e){
        // execute A star
    static int exec_num=0;
    exec_num++;
    string exect_msg;
    if(exec_num==2){
        if(!trigger_){
            exect_msg = "don't trigger!";
            //printf("don't trigger!\n");
        }

        if(!have_odom_){
            exect_msg = "don't have odometry!";
            //printf("don't have odometry!\n");
            // return;
        }
            
        if(!has_point_map_)
        {
            exect_msg = "don't have point cloud!";
            //printf("don't have point cloud! \n");
            // return;
        }
        if(!have_goal_){
            exect_msg = "wait goal!";
            //printf("*** wait goal!*** \n");
            // return;
        }
    }

    if(!trigger_ || !have_odom_ || !has_point_map_ || !have_goal_)
    {
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME,exect_msg);
        exec_num=0;
        return;
    }

    Astar_ptr->reset();
    int astar_state = Astar_ptr->search(start_pt_, end_pt_);
    if(astar_state==Astar::NO_PATH){
          pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "a star find no path, please reset the goal!");
    }
    else{
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "astart find path success!");
        std::vector<Eigen::Vector3d> A_star_path = Astar_ptr->getPath();
        visualization_->drawPath(A_star_path, 0.1,Eigen::Matrix<double, 4, 1>(1.0, 0, 0, 1), 1);
        generate_CMD(A_star_path);
    }
}

void GlobalPlanner::generate_CMD(std::vector<Eigen::Vector3d> path){
/*
nav_msgs/Path
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/PoseStamped[] poses
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
*/
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
        // printf("%d: >>> %f,   %f,   %f >>>>\n", i, path[i](0), path[i](1), path[i](2));
        A_star_path_cmd.poses.push_back(path_i_pose);
    }
    // printf("goal position: %f, %f, %f\n", end_pt_(0), end_pt_(1), end_pt_(2));
    control_time = ros::Time::now();
    replan.data = 0;
    path_cmd_Pub.publish(A_star_path_cmd);
    
}

void GlobalPlanner::odomCallback(const nav_msgs::OdometryConstPtr &msg){
    odom_ = *msg;
    odom_.header.frame_id = msg->header.frame_id;
    have_odom_ = true;
    start_pt_ << odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z; 

    if (is_2D == 1)
    {
        start_pt_(2) = fly_height;
    }
}

void GlobalPlanner::globalcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg){
    /* need odom_ for center radius sensing */
    if (!have_odom_) {
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "global point cloud: --- no odom!---");
        return;
    }

    pcl::fromROSMsg(*msg, latest_global_pcl_);
    has_point_map_ = true;

    global_map_ptr_ = msg;

    // 二维平面规划
    if (is_2D == 1)
    {
        //global_map_ptr_
    }

    Astar_ptr->setEnvironment(global_map_ptr_);

    visualization_msgs::Marker m;
    getOccupancyMarker(m, 0, Eigen::Vector4d(0, 0.5, 0.5, 1.0));
    global_map_marker_Pub.publish(m);
}

void GlobalPlanner::getOccupancyMarker(visualization_msgs::Marker &m, int id, Eigen::Vector4d color) {
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

void GlobalPlanner::safetyCallback(const ros::TimerEvent& e){
//     rosmsg show nav_msgs/Odometry 
// std_msgs/Header header
//   uint32 seq
//   time stamp
//   string frame_id
// string child_frame_id
// geometry_msgs/PoseWithCovariance pose
//   geometry_msgs/Pose pose
//     geometry_msgs/Point position
//       float64 x
//       float64 y
//       float64 z
//     geometry_msgs/Quaternion orientation
//       float64 x
//       float64 y
//       float64 z
//       float64 w
//   float64[36] covariance
// geometry_msgs/TwistWithCovariance twist
//   geometry_msgs/Twist twist
//     geometry_msgs/Vector3 linear
//       float64 x
//       float64 y
//       float64 z
//     geometry_msgs/Vector3 angular
//       float64 x
//       float64 y
//       float64 z
//   float64[36] covariance

    Eigen::Vector3d cur_pos(odom_.pose.pose.position.x, 
                                                        odom_.pose.pose.position.y, 
                                                        odom_.pose.pose.position.z);
    bool is_safety = Astar_ptr->check_safety(cur_pos, safe_distance);

    // give some for replan.
    if(!is_safety /*&& (ros::Time::now()-control_time).toSec()>3.0*/){
        // printf("[safetyCallback]: not safety, pls re select the goal point.\n");
        replan.data = 1;
        // replan_cmd_Pub.publish(replan);
    }
    else{
        replan.data = 0;
        
    }
    replan_cmd_Pub.publish(replan);
}

void GlobalPlanner::switchCallback(const std_msgs::Bool::ConstPtr &msg){
    trigger_= msg->data;
}


}