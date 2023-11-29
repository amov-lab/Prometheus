#include "rviz_topic.hpp"

GFKD::GFKD(ros::NodeHandle &nh)
{
    nh.param("ROBOT_ID", drone_id_, 1);
    nh.param("next_drone_ip", tcp_ip_, std::string("127.0.0.1"));
    nh.param("broadcast_ip", udp_ip_, std::string("127.0.0.255"));
    nh.param("ground_station_ip", rviz_ip_, std::string("127.0.0.1"));

    this->communication = new Communication();

    octomap_point_cloud_centers_sub_ = nh.subscribe("/octomap_point_cloud_centers", 100, &GFKD::octomapPointCloudCentersCb, this);

    scan_point_cloud_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "/prometheus/scan_point_cloud", 100, &GFKD::scanPointCloudCb, this);
    ///map_generator/global_cloud

    tf_sub_ = nh.subscribe("/tf", 10, &GFKD::tfCb, this);
    tf_static_sub_ = nh.subscribe("/tf_static", 10, &GFKD::tfStaticCb, this);
    trajectory_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "/prometheus/trajectory", 10, &GFKD::trajectoryCb, this);

    //goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/uav" + std::to_string(drone_id_) + "/prometheus/motion_planning/goal", 100);

    uav_mesh_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "/prometheus/uav_mesh",10 , &GFKD::uavMeshCb, this);

    scan_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "/prometheus/scan_in_system_time",10 , &GFKD::scanCb, this);

    optimal_list_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "_ego_planner_node/optimal_list",10 , &GFKD::optimalListCb, this);

    goal_point_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "_ego_planner_node/goal_point",10 , &GFKD::goalPointCb, this);

    //goal_sub_ =  nh.subscribe("/uav" + std::to_string(drone_id_) + "/prometheus/motion_planning/goal", 10 , &GFKD::goalCb, this);

    occupancy_inflate_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "_ego_planner_node/grid_map/occupancy_inflate", 10 , &GFKD::occupancyInflateCb, this);

   // camera_depth_color_points_sub_ = nh.subscribe("/camera/depth/color/points",10, &GFKD::cameraDepthColorPointsCb, this);
}

GFKD::GFKD(ros::NodeHandle &nh,int id,std::string ground_station_ip):drone_id_(id),rviz_ip_(ground_station_ip)
{
    tf_sub_ = nh.subscribe("/tf", 10, &GFKD::tfCb, this);
    tf_static_sub_ = nh.subscribe("/tf_static", 10, &GFKD::tfStaticCb, this);
    uav_mesh_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "/prometheus/uav_mesh",10 , &GFKD::uavMeshCb, this);
    trajectory_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "/prometheus/trajectory", 10, &GFKD::trajectoryCb, this);
    // ref_trajectory_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "/prometheus/reference_trajectory", 10, &GFKD::referenceTrajectoryCb, this);
}

GFKD::~GFKD()
{

}

// void GFKD::goalPub(struct Goal goal)
// {
//     geometry_msgs::PoseStamped msg;
//     msg.header.seq = goal.seq;
//     msg.header.frame_id = goal.frame_id;
//     msg.pose.position.x = goal.position_x;
//     msg.pose.position.y = goal.position_y;
//     msg.pose.position.z = this->current_height;
//     msg.pose.orientation.x = goal.orientation_x;
//     msg.pose.orientation.y = goal.orientation_y;
//     msg.pose.orientation.z = goal.orientation_z;
//     msg.pose.orientation.w = goal.orientation_w;
//     goal_pub_.publish(msg);
// }

void GFKD::sendRvizByUdp(int msg_len, std::string target_ip)
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

void GFKD::octomapPointCloudCentersCb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    sensor_msgs::PointCloud2 point_cloud = *msg;
    sendRvizByUdp(encodeRvizMsg(point_cloud,GFKDRvizMsgId::GFKD_OctomapPointCloudCenters),rviz_ip_);
}

void GFKD::scanPointCloudCb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    sensor_msgs::PointCloud2 point_cloud = *msg;
    sendRvizByUdp(encodeRvizMsg(point_cloud,GFKDRvizMsgId::GFKD_ScanPointCloud),rviz_ip_);
}

void GFKD::tfCb(const tf2_msgs::TFMessage::ConstPtr &msg)
{
    tf2_msgs::TFMessage tf = *msg;
    sendRvizByUdp(encodeRvizMsg(tf,GFKDRvizMsgId::GFKD_TF),rviz_ip_);
}

void GFKD::tfStaticCb(const tf2_msgs::TFMessage::ConstPtr &msg)
{
    tf2_msgs::TFMessage tf_static = *msg;
    sendRvizByUdp(encodeRvizMsg(tf_static,GFKDRvizMsgId::GFKD_TFStatic),rviz_ip_);
}

void GFKD::trajectoryCb(const nav_msgs::Path::ConstPtr &msg)
{
    nav_msgs::Path trajectory = *msg;
    sendRvizByUdp(encodeRvizMsg(trajectory,GFKDRvizMsgId::GFKD_Trajectory),rviz_ip_);
}

// void GFKD::referenceTrajectoryCb(const nav_msgs::Path::ConstPtr &msg)
// {
//     nav_msgs::Path ref_trajectory = *msg;
//     sendRvizByUdp(encodeRvizMsg(ref_trajectory,GFKDRvizMsgId::ReferenceTrajectory),rviz_ip_);
// }

void GFKD::uavMeshCb(const visualization_msgs::Marker::ConstPtr &msg)
{
    visualization_msgs::Marker uav_mesh = *msg;
    this->current_height = msg->pose.position.z;
    sendRvizByUdp(encodeRvizMsg(uav_mesh,GFKDRvizMsgId::GFKD_UAVMesh),rviz_ip_);
}

void GFKD::scanCb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    sensor_msgs::LaserScan scan = *msg;
    sendRvizByUdp(encodeRvizMsg(scan,GFKDRvizMsgId::GFKD_Scan),rviz_ip_);
}

void GFKD::optimalListCb(const visualization_msgs::Marker::ConstPtr &msg)
{
    visualization_msgs::Marker marker = *msg;
    sendRvizByUdp(encodeRvizMsg(marker,GFKDRvizMsgId::GFKD_OptimalList),rviz_ip_);
}

void GFKD::goalPointCb(const visualization_msgs::Marker::ConstPtr &msg)
{
    visualization_msgs::Marker marker = *msg;
    int i = 100;
    while(i > 0)
    {
        sendRvizByUdp(encodeRvizMsg(marker,GFKDRvizMsgId::GFKD_GoalPoint),rviz_ip_);
        i--;
        usleep(10);
    }
}

void GFKD::goalCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    geometry_msgs::PoseStamped goal = *msg;
    int i = 100;
    while(i > 0)
    {
        sendRvizByUdp(encodeRvizMsg(goal,GFKDRvizMsgId::GFKD_Goal),rviz_ip_);
        i--;
        usleep(10);
    }
}

void GFKD::occupancyInflateCb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    sensor_msgs::PointCloud2 point_cloud = *msg;
    sendRvizByUdp(encodeRvizMsg(point_cloud,GFKDRvizMsgId::GFKD_OccupancyInflate),rviz_ip_);
}

void GFKD::cameraDepthColorPointsCb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    sensor_msgs::PointCloud2 point_cloud = *msg;
    sendRvizByUdp(encodeRvizMsg(point_cloud,GFKDRvizMsgId::CameraDepthColorPoints),rviz_ip_);
}
