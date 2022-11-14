#include "ego_planner_swarm_topic.hpp"

EGOPlannerSwarm::EGOPlannerSwarm(ros::NodeHandle &nh)
{
    nh.param("ROBOT_ID", drone_id_, 1);
    nh.param("next_drone_ip", tcp_ip_, std::string("127.0.0.1"));
    nh.param("broadcast_ip", udp_ip_, std::string("127.0.0.255"));
    nh.param("ground_stationt_ip", rviz_ip_, std::string("127.0.0.1"));

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

    point_cloud_sub_ = nh.subscribe("/map_generator/global_cloud", 100, &EGOPlannerSwarm::pointCloudSubCb, this);

    point_cloud_ex_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "/map_generator/local_cloud", 100, &EGOPlannerSwarm::pointCloudExSubCb, this);
    ///map_generator/global_cloud

    tf_sub_ = nh.subscribe("/tf", 10, &EGOPlannerSwarm::tfCb, this);
    tf_static_sub_ = nh.subscribe("/tf_static", 10, &EGOPlannerSwarm::tfStaticCb, this);
    trajectory_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "/prometheus/trajectory", 10, &EGOPlannerSwarm::trajectoryCb, this);

    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/uav" + std::to_string(drone_id_) + "/prometheus/motion_planning/goal", 100);

    uav_mesh_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "/prometheus/uav_mesh",10 , &EGOPlannerSwarm::uavMeshCb, this);

    scan_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "/scan",10 , &EGOPlannerSwarm::scanCb, this);

    optimal_traj_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "_ego_planner_node/optimal_list",10 , &EGOPlannerSwarm::optimalTrajCb, this);

    goal_point_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "_ego_planner_node/goal_point",10 , &EGOPlannerSwarm::goalPointCb, this);

    goal_sub_ =  nh.subscribe("/uav" + std::to_string(drone_id_) + "/prometheus/motion_planning/goal", 10 , &EGOPlannerSwarm::goalCb, this);
}

EGOPlannerSwarm::EGOPlannerSwarm(ros::NodeHandle &nh,int id,std::string ground_stationt_ip):drone_id_(id),rviz_ip_(ground_stationt_ip)
{
    tf_sub_ = nh.subscribe("/tf", 10, &EGOPlannerSwarm::tfCb, this);
    tf_static_sub_ = nh.subscribe("/tf_static", 10, &EGOPlannerSwarm::tfStaticCb, this);
    uav_mesh_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "/prometheus/uav_mesh",10 , &EGOPlannerSwarm::uavMeshCb, this);
    trajectory_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "/prometheus/trajectory", 10, &EGOPlannerSwarm::trajectoryCb, this);
    ref_trajectory_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "/prometheus/reference_trajectory", 10, &EGOPlannerSwarm::referenceTrajectoryCb, this);
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

void EGOPlannerSwarm::goalPub(struct Goal goal)
{
    geometry_msgs::PoseStamped msg;
    msg.header.seq = goal.seq;
    msg.header.frame_id = goal.frame_id;
    msg.pose.position.x = goal.position_x;
    msg.pose.position.y = goal.position_y;
    msg.pose.position.z = this->current_height;
    msg.pose.orientation.x = goal.orientation_x;
    msg.pose.orientation.y = goal.orientation_y;
    msg.pose.orientation.z = goal.orientation_z;
    msg.pose.orientation.w = goal.orientation_w;
    goal_pub_.publish(msg);
}

void EGOPlannerSwarm::sendRvizByUdp(int msg_len, std::string target_ip)
{
    //如果是发送给本机，则不需要发送。因为如果话题相同、会一顿一顿的来回跳转、影响流畅性。
    if(target_ip == "127.0.0.1") return;

    //std::cout << "rviz:" << msg_len << std::endl;
    rviz_socket = socket(PF_INET, SOCK_DGRAM, 0);
    if (rviz_socket < 0)
    {
        printf("Socket creation error \n");
        return;
    }

    memset(&rviz_addr, 0, sizeof(rviz_addr));
    rviz_addr.sin_family = AF_INET;
    rviz_addr.sin_port = htons(8890);
    rviz_addr.sin_addr.s_addr = inet_addr(target_ip.c_str());

    // Convert IPv4 and IPv6 addresses from text to binary form
    if (inet_pton(AF_INET, target_ip.c_str(), &rviz_addr.sin_addr) <= 0)
    {
        printf("Invalid address/ Address not supported \n");
        return;
    }


    char *ptr = rviz_recv_buf;
    if (msg_len < BUF_LEN)
        sendto(rviz_socket, rviz_recv_buf, msg_len, 0, (struct sockaddr *)&rviz_addr, sizeof(rviz_addr));
    else
    {
        int len = msg_len;
        while (true)
        {
            //std::cout << "len: " << len << std::endl;
            len = len - BUF_LEN;
            if (len > 0)
            {
                sendto(rviz_socket, ptr, BUF_LEN, 0, (struct sockaddr *)&rviz_addr, sizeof(rviz_addr));
            }
            else if (len < 0)
            {
                sendto(rviz_socket, ptr, len + BUF_LEN, 0, (struct sockaddr *)&rviz_addr, sizeof(rviz_addr));
                break;
            }
            //偏移量
            ptr += BUF_LEN;
            usleep(100);
        }
    }
    close(rviz_socket);
}

void EGOPlannerSwarm::pointCloudSubCb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    sensor_msgs::PointCloud2 point_cloud = *msg;
    sendRvizByUdp(encodeRvizMsg(point_cloud,RvizMsgId::PointClound2),rviz_ip_);
}

void EGOPlannerSwarm::pointCloudExSubCb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    sensor_msgs::PointCloud2 point_cloud = *msg;
    sendRvizByUdp(encodeRvizMsg(point_cloud,RvizMsgId::PointClound2Ex),rviz_ip_);
}

void EGOPlannerSwarm::tfCb(const tf2_msgs::TFMessage::ConstPtr &msg)
{
    tf2_msgs::TFMessage tf = *msg;
    sendRvizByUdp(encodeRvizMsg(tf,RvizMsgId::TF),rviz_ip_);
}

void EGOPlannerSwarm::tfStaticCb(const tf2_msgs::TFMessage::ConstPtr &msg)
{
    tf2_msgs::TFMessage tf_static = *msg;
    sendRvizByUdp(encodeRvizMsg(tf_static,RvizMsgId::TFStatic),rviz_ip_);
}

void EGOPlannerSwarm::trajectoryCb(const nav_msgs::Path::ConstPtr &msg)
{
    nav_msgs::Path trajectory = *msg;
    sendRvizByUdp(encodeRvizMsg(trajectory,RvizMsgId::Trajectory),rviz_ip_);
}

void EGOPlannerSwarm::referenceTrajectoryCb(const nav_msgs::Path::ConstPtr &msg)
{
    nav_msgs::Path ref_trajectory = *msg;
    sendRvizByUdp(encodeRvizMsg(ref_trajectory,RvizMsgId::ReferenceTrajectory),rviz_ip_);
}

void EGOPlannerSwarm::uavMeshCb(const visualization_msgs::Marker::ConstPtr &msg)
{
    visualization_msgs::Marker uav_mesh = *msg;
    this->current_height = msg->pose.position.z;
    sendRvizByUdp(encodeRvizMsg(uav_mesh,RvizMsgId::UAVMesh),rviz_ip_);
}

void EGOPlannerSwarm::scanCb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    sensor_msgs::LaserScan scan = *msg;
    sendRvizByUdp(encodeRvizMsg(scan,RvizMsgId::Scan),rviz_ip_);
}

void EGOPlannerSwarm::optimalTrajCb(const visualization_msgs::Marker::ConstPtr &msg)
{
    visualization_msgs::Marker marker = *msg;
    sendRvizByUdp(encodeRvizMsg(marker,RvizMsgId::OptimalTraj),rviz_ip_);
}

void EGOPlannerSwarm::goalPointCb(const visualization_msgs::Marker::ConstPtr &msg)
{
    visualization_msgs::Marker marker = *msg;
    int i = 100;
    while(i > 0)
    {
        sendRvizByUdp(encodeRvizMsg(marker,RvizMsgId::GoalPoint),rviz_ip_);
        i--;
        usleep(10);
    }
}

void EGOPlannerSwarm::goalCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    geometry_msgs::PoseStamped goal = *msg;
    int i = 100;
    while(i > 0)
    {
        sendRvizByUdp(encodeRvizMsg(goal,RvizMsgId::Goal),rviz_ip_);
        i--;
        usleep(10);
    }
}