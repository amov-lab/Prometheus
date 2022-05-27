#include <ros/ros.h>
#include <my_visualization/plan_visual.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include "quadrotor_msgs/PolynomialTrajectory.h"
#include "quadrotor_msgs/PositionCommand.h"

using namespace my_planner;
my_planner::PlanVisual::Ptr visual;
nav_msgs::Odometry odom;

void goal_visual_cb(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    visual->deleteAllMarker();
    Eigen::Vector3d goalPoint;
    for (int i = 0; i < int(msg->poses.size()); i++)
    {
        goalPoint << msg->poses[i].position.x, msg->poses[i].position.y, msg->poses[i].position.z;
        visual->displayGoalPoint(goalPoint, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }
}

void pos_cmd_visual_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
    Eigen::Vector3d start, end;
    start(0) = odom.pose.pose.position.x;
    start(1) = odom.pose.pose.position.y;
    start(2) = odom.pose.pose.position.z;

    end(0) = start(0) + msg->velocity.x;
    end(1) = start(1) + msg->velocity.y;
    end(2) = start(2) + msg->velocity.z;

    visual->displayArrow(visual->pub_type::VEL, start, end, Eigen::Vector4d(0.2, 0.8, 0, 1), 0);

    end(0) = start(0) + msg->acceleration.x;
    end(1) = start(1) + msg->acceleration.y;
    end(2) = start(2) + msg->acceleration.z;

    visual->displayArrow(visual->pub_type::ACC, start, end, Eigen::Vector4d(0.2, 0, 0.8, 1), 0);

    end(0) = start(0) + msg->jerk.x;
    end(1) = start(1) + msg->jerk.y;
    end(2) = start(2) + msg->jerk.z;

    visual->displayArrow(visual->pub_type::JERK, start, end, Eigen::Vector4d(0.8, 0, 0.2, 1), 0);
}

void poly_traj_visual_cb(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    std::vector<Eigen::Vector3d> list;
    Eigen::Vector3d pt;

    for (int i = 0; i < int(msg->poses.size()); i++)
    {
        pt(0) = msg->poses[i].position.x;
        pt(1) = msg->poses[i].position.y;
        pt(2) = msg->poses[i].position.z;
        list.push_back(pt);
    }

    visual->displayTraj(list, 0);
}

void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "min_snap_visual");
    ros::NodeHandle nh("~");

    ros::Rate rate(100.0);

    visual.reset(new my_planner::PlanVisual(nh));

    ros::Subscriber Goal_rviz_sub = nh.subscribe<geometry_msgs::PoseArray>("/goal_list", 10, goal_visual_cb);
    ros::Subscriber Pos_cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/position_cmd", 10, pos_cmd_visual_cb);
    ros::Subscriber Odom_sub = nh.subscribe<nav_msgs::Odometry>("/odometry", 10, odom_cb);
    ros::Subscriber Poly_coef_sub = nh.subscribe<geometry_msgs::PoseArray>("/traj_pts", 10, poly_traj_visual_cb);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}