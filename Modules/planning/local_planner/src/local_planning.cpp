//
// Created by taojiang on 2019/12/9.
//

#include "local_planning.h"

namespace local_planner
{

void PotentialFiledPlanner::init(ros::NodeHandle& nh){
    // set mode
    flight_type_ = FLIGHT_TYPE::MANUAL_GOAL;
    // set algorithm
    apf_planner_ptr.reset(new APF);
    apf_planner_ptr->init(nh);

    sensor_msgs::PointCloud2ConstPtr init_local_map(new sensor_msgs::PointCloud2());
    local_map_ptr_ = init_local_map;
    
    // ros param
    nh.param("local_planning/is_simulation", is_simulation, 0);

    // init visualization
    ROS_INFO("---init visualization!---");
    visualization_.reset(new PlanningVisualization(nh));
    local_map_marker_Pub   = node_.advertise<visualization_msgs::Marker>("/planning/local_map",  10);  

    /* ---------- callback ---------- */
    ROS_INFO("---init sub and pub!---");
    waypoint_sub_ = node_.subscribe("/planning/goal", 1, &PotentialFiledPlanner::waypointCallback, this);

    odom_sub_ = node_.subscribe<nav_msgs::Odometry>("/planning/odom_world", 10, &PotentialFiledPlanner::odomCallback, this);

    global_point_clound_sub_ = node_.subscribe<sensor_msgs::PointCloud2>("/planning/global_point_cloud", 1, &PotentialFiledPlanner::globalcloudCallback,
                                                                            this);

    local_point_clound_sub_ = node_.subscribe<sensor_msgs::PointCloud2>("/rtabmap/cloud_map", 1, &PotentialFiledPlanner::localcloudCallback,
    this);

    pos_cmd_pub = node_.advertise<prometheus_msgs::PositionReference>("/planning/position_cmd", 10);
    px4_pos_cmd_pub = node_.advertise<geometry_msgs::Point>("/prometheus/local_planner/desired_vel", 10);
    exec_timer_ = node_.createTimer(ros::Duration(0.05), &PotentialFiledPlanner::execFSMCallback, this, false);

    /*   bool  state    */
    trigger_=false;
    have_goal_=false;
    has_point_map_=false;
    have_odom_=false;
    ROS_INFO("---planning_fsm: init finished!---");

    // loop
    ros::spin();
}


void PotentialFiledPlanner::execFSMCallback(const ros::TimerEvent& e){

    // if(!trigger_)
    // {
    //     printf("don't trigger!\n");
    //     return;
    // }

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

    printf("begin  apf \n");
    apf_planner_ptr->set_local_map(local_map_ptr_);
    
    apf_planner_ptr->set_odom(odom_);

    apf_planner_ptr->compute_force(end_pt_, start_pt_, desired_vel);

    if(desired_vel.norm() > 1.0)
    {
        desired_vel = desired_vel / desired_vel.norm() * 0.8;  // the max velocity is 0.2m
    }
    printf("desired vel: [%f, %f, %f]\n", desired_vel(0), desired_vel(1), desired_vel(2));
    // 发布控制指令
    generate_cmd(desired_vel);
    control_time = ros::Time::now();
    // pos_cmd_pub.publish(cmd);
    if((end_pt_-start_pt_).norm() < 0.05){
        printf("reach the goal!\n");
    }

    // visualization_->drawPath(kino_path, 0.1, Eigen::Vector4d(1, 0, 0, 1));  // red

}

void PotentialFiledPlanner::generate_cmd(Eigen::Vector3d desired_vel){
    ros::Time time_now = ros::Time::now();
    cmd.header.stamp = time_now;
    cmd.header.frame_id = "map";

    cmd.Move_mode = prometheus_msgs::PositionReference::TRAJECTORY;  //TRAJECTORY
    cmd.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME; //ENU_FRAME
    cmd.time_from_start = 0.0;

    cmd.position_ref[0] = start_pt_(0) + desired_vel(0) * 0.01;
    cmd.position_ref[1] = start_pt_(1) + desired_vel(1)* 0.01;
    cmd.position_ref[2] = start_pt_(2) + desired_vel(2) * 0.01;;

    cmd.velocity_ref[0] = desired_vel(0);
    cmd.velocity_ref[1] = desired_vel(1);
    cmd.velocity_ref[2] = desired_vel(2);

    cmd.acceleration_ref[0] = 0.0;
    cmd.acceleration_ref[1] = 0.0;
    cmd.acceleration_ref[2] = 0.0;

    cmd.yaw_ref = 0.0;

    // 发布控制指令
    if(is_simulation==1){
        pos_cmd_pub.publish(cmd);
    }
    else
    {
        px4_cmd.x = desired_vel(0);
        px4_cmd.y = desired_vel(1);
        px4_cmd.z = desired_vel(2);
        px4_pos_cmd_pub.publish(px4_cmd);
    }
    
    // 可视化
    visualization_->drawVel(start_pt_, desired_vel * 0.5, Eigen::Vector4d(0, 1, 0, 1.0), 0);

}

//  the goal is in the world frame. 
void PotentialFiledPlanner::waypointCallback(const geometry_msgs::PoseStampedConstPtr& msg){
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
    {}
    
    ROS_INFO("---planning_fsm: get waypoint: [ %f, %f, %f]!---", end_pt_(0),
                                                            end_pt_(1), 
                                                            end_pt_(2));

    visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));

    end_vel_.setZero();
    have_goal_ = true;

    // if (exec_state_ == WAIT_GOAL)
    //     changeExecState(GEN_NEW_TRAJ, "TRIG");
    // else if (exec_state_ == EXEC_TRAJ)
    //     changeExecState(REPLAN_TRAJ, "TRIG");

}



void PotentialFiledPlanner::odomCallback(const nav_msgs::OdometryConstPtr &msg){
    odom_ = *msg;
    odom_.header.frame_id = "map";
    have_odom_ = true;
    start_pt_ << odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z; 
}


void PotentialFiledPlanner::globalcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg){

}

//  the local cloud is in the local frame. 
void PotentialFiledPlanner::localcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg){
    /* need odom_ for center radius sensing */
    if (!have_odom_) {
        ROS_INFO("local point cloud: --- no odom!---");
        return;
    }
    // printf("receive the local point cloud\n");
    local_map_ptr_ = msg;

    // ros::Time begin_load_point_cloud = ros::Time::now();
    pcl::fromROSMsg(*msg, latest_local_pcl_);
    has_point_map_ = true;

    localframe2global();

    visualization_msgs::Marker m;
    getOccupancyMarker(m, 0, Eigen::Vector4d(0, 0.5, 0.5, 1.0));
    local_map_marker_Pub.publish(m);

}

void PotentialFiledPlanner::getOccupancyMarker(visualization_msgs::Marker &m, int id, Eigen::Vector4d color) {
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


}


