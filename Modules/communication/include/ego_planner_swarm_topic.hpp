#ifndef EGO_PLANNER_SWARM_TOPIC_HPP
#define EGO_PLANNER_SWARM_TOPIC_HPP

#include <ros/ros.h>
#include "communication.hpp"
#include "prometheus_msgs/MultiBsplines.h"
#include "prometheus_msgs/Bspline.h"
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>

class EGOPlannerSwarm
{
public:
    EGOPlannerSwarm(ros::NodeHandle &nh);
    ~EGOPlannerSwarm();

    void swarmTrajPub(struct MultiBsplines multi_bsplines);

    void oneTrajPub(struct Bspline bspline);

private:

    void multitrajSubTcpCb(const prometheus_msgs::MultiBsplines::ConstPtr &msg);

    void oneTrajSubUdpCb(const prometheus_msgs::Bspline::ConstPtr &msg);

private:
    ros::Subscriber swarm_trajs_sub_, one_traj_sub_;
    ros::Publisher swarm_trajs_pub_, one_traj_pub_;

    int drone_id_;
    std::string tcp_ip_,udp_ip_;

    Communication* communication;
};
#endif