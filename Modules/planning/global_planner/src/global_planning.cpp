#include "global_planning.h"

namespace global_planner
{

void GlobalPlanner::init(ros::NodeHandle& nh){
    // set mode

    // set algorithm
    sensor_msgs::PointCloud2ConstPtr init_global_map(new sensor_msgs::PointCloud2());
    global_map_ptr_ = init_global_map;

    // init visualization
    ROS_INFO("---init visualization!---");
    visualization_.reset(new PlanningVisualization(nh));

    /* ---------- callback ---------- */
    ROS_INFO("---init sub and pub!---");
    waypoint_sub_ = node_.subscribe("/planning/goal", 1, &GlobalPlanner::waypointCallback, this);

    odom_sub_ = node_.subscribe<nav_msgs::Odometry>("/planning/odom_world", 10, &GlobalPlanner::odomCallback, this);

    global_point_clound_sub_ = node_.subscribe<sensor_msgs::PointCloud2>("/rtabmap/cloud_map", 1, &GlobalPlanner::globalcloudCallback, this);

    // publish 
    global_map_marker_Pub   = node_.advertise<visualization_msgs::Marker>("/planning/global_map_marker",  10);  
    path_cmd_Pub   = node_.advertise<nav_msgs::Path>("/planning/path_cmd",  10);  
    // a* algorithm
    Astar_ptr.reset(new Astar);
    Astar_ptr->setParam(nh);
    Astar_ptr->init(nh);

    flight_type_  = FLIGHT_TYPE::MANUAL_GOAL;

    ros::spin(); 
}


void GlobalPlanner::waypointCallback(const geometry_msgs::PoseStampedConstPtr& msg){
    cout << "[waypointCallback]: Triggered!" << endl;

    if (msg->pose.position.z < 0.1)  // the minimal goal height 
        return;

    trigger_ = true;
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
    {
        // end_pt_ << 0.0, 0.0, 1.0;
    }
    
    ROS_INFO("---global planning_: get waypoint: [ %f, %f, %f]!---", end_pt_(0),
                                                            end_pt_(1), 
                                                            end_pt_(2));

    visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));

    end_vel_.setZero();
    have_goal_ = true;

    // execute A star
    if(!trigger_)
    {
        printf("don't trigger!\n");
        return;
    }

    if(!have_odom_){
        printf("don't have odometry!\n");
        return;
    }
        
    if(!has_point_map_)
    {
        printf("don't have point cloud! \n");
        return;
    }
    if(!have_goal_){
        printf("*** wait goal!*** \n");
        return;
    }

    printf("begin  astar !!\n");
    Astar_ptr->reset();
    int astar_state = Astar_ptr->search(start_pt_, end_pt_);
    if(astar_state==Astar::NO_PATH){
        printf("a star find no path, please reset the goal!\n");
    }
    else{
        printf("astart find path success!\n");
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
    A_star_path_cmd.header.frame_id = "map";
    A_star_path_cmd.header.stamp = ros::Time::now();
    printf("Path:  \n");
    for (int i=0; i<path.size(); ++i){
        geometry_msgs::PoseStamped path_i_pose;
        path_i_pose .header.frame_id = "map";
        path_i_pose.pose.position.x = path[i](0);
        path_i_pose.pose.position.y = path[i](1);
        path_i_pose.pose.position.z = path[i](2);
        printf("%d: >>> %f,   %f,   %f >>>>\n", path[i](0), path[i](1), path[i](2));
        A_star_path_cmd.poses.push_back(path_i_pose);
    }
    printf("goal position: %f, %f, %f\n", end_pt_(0), end_pt_(1), end_pt_(2));
    path_cmd_Pub.publish(A_star_path_cmd);
    
}

void GlobalPlanner::odomCallback(const nav_msgs::OdometryConstPtr &msg){
    odom_ = *msg;
    odom_.header.frame_id = "map";
    have_odom_ = true;
    start_pt_ << odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z; 
}

void GlobalPlanner::globalcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg){
    /* need odom_ for center radius sensing */
    if (!have_odom_) {
        ROS_INFO("global point cloud: --- no odom!---");
        return;
    }
    global_map_ptr_ = msg;

    // ros::Time begin_load_point_cloud = ros::Time::now();
    pcl::fromROSMsg(*msg, latest_global_pcl_);
    has_point_map_ = true;

    Astar_ptr->setEnvironment(global_map_ptr_);

    // localframe2global();
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

}