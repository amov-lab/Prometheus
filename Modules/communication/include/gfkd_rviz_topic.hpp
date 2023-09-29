#ifndef GFKD_RVIZ_TOPIC_HPP
#define GFKD_RVIZ_TOPIC_HPP

#include "sensor_msgs/PointCloud2.h"
#include "tf2_msgs/TFMessage.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"

#include <ros/serialization.h>
#include "communication.hpp"
#include <ros/ros.h>

enum GFKDRvizMsgId
{
    GFKD_OctomapPointCloudCenters = 220,
    GFKD_ScanPointCloud = 221,
    GFKD_TF = 222,
    GFKD_TFStatic = 223,
    GFKD_Trajectory = 224,
    GFKD_UAVMesh = 225,
    GFKD_Scan = 226,
    GFKD_OptimalList = 227,
    GFKD_GoalPoint = 228,
    GFKD_Goal = 229,
    GFKD_OccupancyInflate = 230,

    // /camera/depth/color/points
    CameraDepthColorPoints = 231
    
};

class GFKD
{
public:
    GFKD(ros::NodeHandle &nh);
    GFKD(ros::NodeHandle &nh,int id,std::string ground_stationt_ip);
    ~GFKD();

private:
    void octomapPointCloudCentersCb(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void scanPointCloudCb(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void tfCb(const tf2_msgs::TFMessage::ConstPtr &msg);

    void tfStaticCb(const tf2_msgs::TFMessage::ConstPtr &msg);

    void trajectoryCb(const nav_msgs::Path::ConstPtr &msg);
    // void referenceTrajectoryCb(const nav_msgs::Path::ConstPtr &msg);

    void uavMeshCb(const visualization_msgs::Marker::ConstPtr &msg);

    void scanCb(const sensor_msgs::LaserScan::ConstPtr &msg);

    void optimalListCb(const visualization_msgs::Marker::ConstPtr &msg);

    void goalPointCb(const visualization_msgs::Marker::ConstPtr &msg);

    void goalCb(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void occupancyInflateCb(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void cameraDepthColorPointsCb(const sensor_msgs::PointCloud2::ConstPtr &msg);

    template <typename T>
    int encodeRvizMsg(T msg, int msg_id = 0);

    void sendRvizByUdp(int msg_len, std::string target_ip);

private:
    ros::Subscriber octomap_point_cloud_centers_sub_,tf_sub_,tf_static_sub_,goal_sub_,scan_sub_,scan_point_cloud_sub_,trajectory_sub_,uav_mesh_sub_,goal_point_sub_,occupancy_inflate_sub_,optimal_list_sub_;
    ros::Subscriber camera_depth_color_points_sub_;
    ros::Publisher goal_pub_;

    //
    // ros::Subscriber ref_trajectory_sub_;

    int drone_id_;
    std::string tcp_ip_, udp_ip_,rviz_ip_;

    Communication *communication;

    struct sockaddr_in rviz_addr;
    int rviz_socket;
    char rviz_recv_buf[BUF_LEN * 500];

    float current_height = 1.0;
};

//第二个参数适用于要传输的数据类型相同，但是接收端处理的方式不同所以为了区别msg_id采用自定义。一般情况不使用。
template <typename T>
int GFKD::encodeRvizMsg(T msg, int msg_id)
{
    namespace ser = ros::serialization;
    uint32_t serial_size = ros::serialization::serializationLength(msg);
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    ser::OStream stream(buffer.get(), serial_size);
    ser::serialize(stream, msg);

    // std::cout << "rviz信息长度:" << serial_size << std::endl;

    char *ptr = rviz_recv_buf;

    // HEAD:消息帧头
    int8_t HEAD = 97;
    *((int8_t *)ptr) = HEAD;
    ptr += sizeof(int8_t);
    HEAD = 109;
    *((int8_t *)ptr) = HEAD;
    ptr += sizeof(int8_t);

    // LENGTH:PAYLOAD的长度
    uint32_t LENGTH = serial_size;
    *((uint32_t *)ptr) = LENGTH;
    ptr += sizeof(uint32_t);

    uint8_t MSG_ID = msg_id;
    *((uint8_t *)ptr) = MSG_ID;
    ptr += sizeof(uint8_t);

    // ROBOT_ID
    *((int8_t *)ptr) = 1;
    ptr += sizeof(int8_t);

    // PAYLOAD
    for (int i = 0; i < (int)LENGTH; i++)
    {
        *((char *)ptr) = buffer[i];
        ptr += sizeof(char);
    }

    // CHECK
    uint16_t CHECK = this->communication->checksum(rviz_recv_buf, ptr - rviz_recv_buf);
    *((uint16_t *)ptr) = CHECK;
    ptr += sizeof(uint16_t);

    return ptr - rviz_recv_buf;
}

#endif