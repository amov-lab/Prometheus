#include "ego_planner_swarm_topic.hpp"

EGOPlannerSwarm::EGOPlannerSwarm(ros::NodeHandle &nh)
{
    nh.param("drone_id", drone_id_, 1);
    nh.param("next_drone_ip", tcp_ip_, std::string("127.0.0.1"));
    nh.param("broadcast_ip", udp_ip_, std::string("127.0.0.255"));

    this->communication = new Communication();

    // bsplines_msg_.reset(new prometheus_msgs::MultiBsplines);
    // odom_msg_.reset(new nav_msgs::Odometry);
    // stop_msg_.reset(new std_msgs::Empty);
    // bspline_msg_.reset(new tprometheus_msgs::Bspline);
    std::string sub_traj_topic_name = std::string("/uav") + std::to_string(drone_id_) + std::string("_planning/swarm_trajs");
    swarm_trajs_sub_ = nh.subscribe(sub_traj_topic_name.c_str(), 10, &EGOPlannerSwarm::multitrajSubTcpCb, this, ros::TransportHints().tcpNoDelay());

    if (drone_id_ >= 2)
    {
        std::string pub_traj_topic_name = std::string("/uav") + std::to_string(drone_id_ - 1) + std::string("_planning/swarm_trajs");
        swarm_trajs_pub_ = nh.advertise<prometheus_msgs::MultiBsplines>(pub_traj_topic_name.c_str(), 10);
    }

    one_traj_sub_ = nh.subscribe("/broadcast_bspline", 100, &EGOPlannerSwarm::oneTrajSubUdpCb, this, ros::TransportHints().tcpNoDelay());
    one_traj_pub_ = nh.advertise<prometheus_msgs::Bspline>("/broadcast_bspline2", 100);
}

EGOPlannerSwarm::~EGOPlannerSwarm()
{

}

void EGOPlannerSwarm::multitrajSubTcpCb(const prometheus_msgs::MultiBsplines::ConstPtr &msg)
{
    struct MultiBsplines multi_bsplines;
    multi_bsplines.drone_id_from = msg->drone_id_from;
    for (int i = 0; i < msg->traj.size(); i++)
    {
        struct Bspline bspline;
        bspline.drone_id = msg->traj[i].drone_id;
        bspline.order = msg->traj[i].order;
        bspline.traj_id = msg->traj[i].traj_id;
        bspline.sec = msg->traj[i].start_time.toSec();
        bspline.drone_id = msg->traj[i].drone_id;
        for (int j = 0; j < msg->traj[i].knots.size(); j++)
        {
            bspline.knots.push_back(msg->traj[i].knots[j]);
        }
        for (int j = 0; j < msg->traj[i].pos_pts.size(); j++)
        {
            struct Point point;
            point.x = msg->traj[i].pos_pts[j].x;
            point.y = msg->traj[i].pos_pts[j].y;
            point.z = msg->traj[i].pos_pts[j].z;
            bspline.pos_pts.push_back(point);
        }
        for (int j = 0; j < msg->traj[i].yaw_pts.size(); j++)
        {
            bspline.yaw_pts.push_back(msg->traj[i].yaw_pts[j]);
        }
        bspline.yaw_dt = msg->traj[i].yaw_dt;
    }
    this->communication->sendMsgByTcp(this->communication->encodeMsg(Send_Mode::TCP, multi_bsplines), tcp_ip_);
}

void EGOPlannerSwarm::oneTrajSubUdpCb(const prometheus_msgs::Bspline::ConstPtr &msg)
{
    struct Bspline bspline;
    bspline.drone_id = msg->drone_id;
    bspline.order = msg->order;
    bspline.traj_id = msg->traj_id;
    bspline.sec = msg->start_time.toSec();
    bspline.drone_id = msg->drone_id;
    for (int j = 0; j < msg->knots.size(); j++)
    {
        bspline.knots.push_back(msg->knots[j]);
    }
    for (int j = 0; j < msg->pos_pts.size(); j++)
    {
        struct Point point;
        point.x = msg->pos_pts[j].x;
        point.y = msg->pos_pts[j].y;
        point.z = msg->pos_pts[j].z;
        bspline.pos_pts.push_back(point);
    }
    for (int j = 0; j < msg->yaw_pts.size(); j++)
    {
        bspline.yaw_pts.push_back(msg->yaw_pts[j]);
    }
    bspline.yaw_dt = msg->yaw_dt;

    this->communication->sendMsgByUdp(this->communication->encodeMsg(Send_Mode::UDP, bspline), udp_ip_);
}

void EGOPlannerSwarm::swarmTrajPub(struct MultiBsplines multi_bsplines)
{
    prometheus_msgs::MultiBsplines msg;
    msg.drone_id_from = multi_bsplines.drone_id_from;
    for (int i = 0; i < multi_bsplines.traj.size(); i++)
    {
        prometheus_msgs::Bspline bspline;
        bspline.drone_id = multi_bsplines.traj[i].drone_id;
        bspline.order = multi_bsplines.traj[i].order;
        bspline.traj_id = multi_bsplines.traj[i].traj_id;
        bspline.start_time.fromSec(multi_bsplines.traj[i].sec);
        bspline.drone_id = multi_bsplines.traj[i].drone_id;
        for (int j = 0; j < multi_bsplines.traj[i].knots.size(); j++)
        {
            bspline.knots.push_back(multi_bsplines.traj[i].knots[j]);
        }
        for (int j = 0; j < multi_bsplines.traj[i].pos_pts.size(); j++)
        {
            geometry_msgs::Point point;
            point.x = multi_bsplines.traj[i].pos_pts[j].x;
            point.y = multi_bsplines.traj[i].pos_pts[j].y;
            point.z = multi_bsplines.traj[i].pos_pts[j].z;
            bspline.pos_pts.push_back(point);
        }
        for (int j = 0; j < multi_bsplines.traj[i].yaw_pts.size(); j++)
        {
            bspline.yaw_pts.push_back(multi_bsplines.traj[i].yaw_pts[j]);
        }
        bspline.yaw_dt = multi_bsplines.traj[i].yaw_dt;
        msg.traj.push_back(bspline);
    }
    this->swarm_trajs_pub_.publish(msg);
}

void EGOPlannerSwarm::oneTrajPub(struct Bspline bspline)
{
    prometheus_msgs::Bspline msg;
    msg.drone_id = bspline.drone_id;
    msg.order = bspline.order;
    msg.traj_id = bspline.traj_id;
    msg.start_time.fromSec(bspline.sec);
    msg.drone_id = bspline.drone_id;
    for (int j = 0; j < bspline.knots.size(); j++)
    {
        msg.knots.push_back(bspline.knots[j]);
    }
    for (int j = 0; j < bspline.pos_pts.size(); j++)
    {
        geometry_msgs::Point point;
        point.x = bspline.pos_pts[j].x;
        point.y = bspline.pos_pts[j].y;
        point.z = bspline.pos_pts[j].z;
        msg.pos_pts.push_back(point);
    }
    for (int j = 0; j < bspline.yaw_pts.size(); j++)
    {
        msg.yaw_pts.push_back(bspline.yaw_pts[j]);
    }
    msg.yaw_dt = bspline.yaw_dt;
    this->one_traj_pub_.publish(msg);
}