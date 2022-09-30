#ifndef EGO_PLANNER_SWARM_TOPIC_HPP
#define EGO_PLANNER_SWARM_TOPIC_HPP

#include <ros/ros.h>
#include "communication.hpp"
#include "prometheus_msgs/MultiBsplines.h"
#include "prometheus_msgs/Bspline.h"
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>

#include <ros/serialization.h>
#include "sensor_msgs/PointCloud2.h"
#include "tf2_msgs/TFMessage.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"

enum RvizMsgId
{
    PointClound2 = 220,
    PointClound2Ex = 221,
    TF = 222,
    TFStatic = 223,
    Trajectory = 224,
    UAVMesh = 225
};

class EGOPlannerSwarm
{
public:
    EGOPlannerSwarm(ros::NodeHandle &nh);
    ~EGOPlannerSwarm();

    void swarmTrajPub(struct MultiBsplines multi_bsplines);

    void oneTrajPub(struct Bspline bspline);

    void goalPub(struct Goal goal);

private:
    void multitrajSubTcpCb(const prometheus_msgs::MultiBsplines::ConstPtr &msg);

    void oneTrajSubUdpCb(const prometheus_msgs::Bspline::ConstPtr &msg);

    void pointCloudSubCb(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void pointCloudExSubCb(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void tfCb(const tf2_msgs::TFMessage::ConstPtr &msg);

    void tfStaticCb(const tf2_msgs::TFMessage::ConstPtr &msg);

    void trajectoryCb(const nav_msgs::Path::ConstPtr &msg);

    void uavMeshCb(const visualization_msgs::Marker::ConstPtr &msg);

    template <typename T>
    uint8_t getRvizMsgId(T msg);

    template <typename T>
    int encodeRvizMsg(T msg, int msg_id = 0);

    void sendRvizByUdp(int msg_len, std::string target_ip);

private:
    ros::Subscriber swarm_trajs_sub_, one_traj_sub_;
    ros::Publisher swarm_trajs_pub_, one_traj_pub_;

    ros::Subscriber point_cloud_sub_,point_cloud_ex_sub_,tf_sub_,tf_static_sub_,trajectory_sub_,uav_mesh_sub_;
    ros::Publisher goal_pub_;

    int drone_id_;
    std::string tcp_ip_, udp_ip_,rviz_ip_;

    Communication *communication;

    struct sockaddr_in rviz_addr;
    int rviz_socket;
    char rviz_recv_buf[BUF_LEN * 500];
};

template <typename T>
uint8_t EGOPlannerSwarm::getRvizMsgId(T msg)
{
    if (typeid(msg) == typeid(sensor_msgs::PointCloud2))
        return RvizMsgId::PointClound2;
    
    return 0;
}

//第二个参数适用于要传输的数据类型相同，但是接收端处理的方式不同所以为了区别msg_id采用自定义。一般情况不使用。
template <typename T>
int EGOPlannerSwarm::encodeRvizMsg(T msg, int msg_id)
{
    namespace ser = ros::serialization;
    uint32_t serial_size = ros::serialization::serializationLength(msg);
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    ser::OStream stream(buffer.get(), serial_size);
    ser::serialize(stream, msg);

    std::cout << "rviz信息长度:" << serial_size << std::endl;

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

    uint8_t MSG_ID;
    // MSG_ID
    if (msg_id == 0)
        MSG_ID = getRvizMsgId(msg);
    else
        MSG_ID = msg_id;

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