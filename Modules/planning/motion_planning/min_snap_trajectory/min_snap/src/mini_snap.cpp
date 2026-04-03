#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include "min_snap/min_snap_closeform.h"

using namespace std;
using namespace my_planner;

ros::Publisher poly_coef_pub,way_point_list_pub;
ros::Subscriber odom_sub;


bool get_goal{false};
bool odom_valid;
geometry_msgs::PoseArray way_point_list;
minsnapCloseform minsnap_solver;
quadrotor_msgs::PolynomialTrajectory poly_pub_topic;

void mainloop(const ros::TimerEvent &e);

void cal_poly_coefs(geometry_msgs::PoseArray goal_list, double mean_vel)
{
    // 航点容器
    std::vector<Eigen::Vector3d> waypoints;
    Eigen::Vector3d wp;
    waypoints.clear();

    for (int i = 0; i < int(goal_list.poses.size()); i++)
    {
        wp << goal_list.poses[i].position.x, goal_list.poses[i].position.y, goal_list.poses[i].position.z;
        waypoints.push_back(wp);
    }
    // 初始化
    minsnap_solver.Init(waypoints, mean_vel);
    // 计算轨迹
    minsnap_solver.calMinsnap_polycoef();

    // 读取轨迹
    Eigen::MatrixXd poly_coef = minsnap_solver.getPolyCoef();
    Eigen::MatrixXd dec_vel = minsnap_solver.getDecVel();
    Eigen::VectorXd time = minsnap_solver.getTime();

    // poly_pub_topic.num_segment = 1;
    poly_pub_topic.num_segment = goal_list.poses.size() - 1;
    poly_pub_topic.coef_x.clear();
    poly_pub_topic.coef_y.clear();
    poly_pub_topic.coef_z.clear();
    poly_pub_topic.time.clear();
    poly_pub_topic.trajectory_id = 0;

    for (int i = 0; i < time.size(); i++)
    {
        for (int j = (i + 1) * 8 - 1; j >= i * 8; j--)
        {
            poly_pub_topic.coef_x.push_back(poly_coef(j, 0));
            poly_pub_topic.coef_y.push_back(poly_coef(j, 1));
            poly_pub_topic.coef_z.push_back(poly_coef(j, 2));
        }
        poly_pub_topic.time.push_back(time(i));
    }

    poly_pub_topic.header.frame_id = "world";
    poly_pub_topic.header.stamp = ros::Time::now();

    // 发布轨迹多项式
    poly_coef_pub.publish(poly_pub_topic);
}

void goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal_pt = msg->pose;
    if (goal_pt.position.z < 0)
    {
        goal_pt.position.z = GOAL_HEIGHT;
        goal_list.poses.push_back(goal_pt);
        goal_pt.position = odom.pose.pose.position;
        goal_list.poses.insert(goal_list.poses.begin(), goal_pt);
        goal_list.header.stamp = ros::Time::now();
        goal_list.header.frame_id = "world";
        goal_list.header.seq = id++;
        goal_list_pub.publish(goal_list);
        solve_min_snap();
        ROS_INFO("solver finished");
        goal_list.poses.clear();
    }
    else
    {
        goal_pt.position.z = GOAL_HEIGHT;
        goal_list.poses.push_back(goal_pt);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mini_snap");
    ros::NodeHandle nh("~");
    ros::Rate rate(200.0);

    //【订阅】odom
    uav_odom_sub = nh.subscribe<nav_msgs::Odometry>("/prometheus/uav_odom_ned", 10, uav_odom_cb);

    rviz_goal_sub = nh.subscribe("/prometheus/goal", 10, goal_cb);

    way_point_list_pub = nh.advertise<geometry_msgs::PoseArray>("/goal_list", 10);

    poly_coef_pub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("/poly_coefs", 10);

    // 【定时器】mainloop
    mainloop_timer = nh.createTimer(ros::Duration(0.01), mainloop);

    odom_valid = false;

    ros::spin();

    return 0;
}

void mainloop(const ros::TimerEvent &e)
{
    if(!odom_valid)
    {
        return;
    }

    if(!get_goal)
    {
        return;
    }

    // 航点清空
    way_point_list.poses.clear();
    geometry_msgs::Pose goal_pt;

    // 加入当前位置
    goal_pt.position = uav_odom_enu.pose.pose.position;
    way_point_list.poses.push_back(goal_pt);

    goal_pt.position.x = 1.2;
    goal_pt.position.y = -1.2;
    goal_pt.position.z = 2.0;
    way_point_list.poses.push_back(goal_pt);

    goal_pt.position.x = circle_origin_pos_1[0] - 5.0*cos(circle_origin_yaw_1);
    goal_pt.position.y = circle_origin_pos_1[1] - 5.0*sin(circle_origin_yaw_1);
    goal_pt.position.z = circle_origin_pos_1[2] - 0.3;      // circle_origin_pos_1[2]:2.77
    way_point_list.poses.push_back(goal_pt);

    goal_pt.position.x = circle_origin_pos_1[0] - 3.0*cos(circle_origin_yaw_1);
    goal_pt.position.y = circle_origin_pos_1[1] - 3.0*sin(circle_origin_yaw_1);
    goal_pt.position.z = circle_origin_pos_1[2];
    way_point_list.poses.push_back(goal_pt);

    way_point_list.header.stamp = ros::Time::now();
    way_point_list.header.frame_id = "world";
    way_point_list.header.seq = 1;
    way_point_list_pub.publish(way_point_list);

    cal_poly_coefs(way_point_list, 4.0f);
}