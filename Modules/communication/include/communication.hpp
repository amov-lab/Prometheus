#ifndef COMUNICATION_HPP
#define COMUNICATION_HPP

#include <ros/ros.h>
#include <boost/format.hpp>
#include <tf/transform_datatypes.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>

#include "../include/CRC.h"
#include "Struct.hpp"

#include <ros/serialization.h>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_msgs/TFMessage.h"
#include "visualization_msgs/MarkerArray.h"


#define BUF_LEN 1024 * 10 // 1024*10 bytes
#define SERV_PORT 20168

typedef unsigned char byte;

enum Send_Mode
{
    TCP = 1,
    UDP = 2
};

class Communication
{
public:
    Communication(ros::NodeHandle &nh);
    ~Communication();

    //编码
    template <typename T>
    int encodeMsg(int8_t send_mode, T msg);

    //根据传入的struct返回对应的MSG_ID
    template <typename T>
    uint8_t getMsgId(T msg);

    // UDP client
    int connectToUdpMulticast(const char *ip, const int port);

    // TCP client  返回套接字
    int connectToDrone(const char *ip, const int port);

    void sendMsgByUdp(int msg_len, std::string target_ip);

    void sendMsgByUdp(int msg_len, const char* udp_msg ,std::string target_ip, int port);

    void sendMsgByTcp(int msg_len, std::string target_ip);

    //第二个参数适用于要传输的数据类型相同，但是接收端处理的方式不同所以为了区别msg_id采用自定义。一般情况不使用。
    template <typename T>
    int encodeRvizMsg(T msg, int msg_id = 0);

    void sendRvizByUdp(int msg_len, std::string target_ip);

    // TCP server
    int waitConnectionFromGroundStation(const int port);

    // UDP server
    int waitConnectionFromMulticast(const int port);

    unsigned short checksum(char *buff, int len);

protected:
    int ROBOT_ID = 0;

    // tcp/udp
    struct sockaddr_in tcp_addr, udp_addr;
    int tcp_send_sock, udp_send_sock, server_fd, udp_fd, recv_sock, udp_socket, rviz_socket, UDP_PORT, TCP_PORT, TCP_HEARTBEAT_PORT, RVIZ_PORT;
    char udp_send_buf[BUF_LEN], udp_recv_buf[BUF_LEN], tcp_send_buf[BUF_LEN], tcp_recv_buf[BUF_LEN], rviz_recv_buf[BUF_LEN * 500];
    std::string udp_ip, multicast_udp_ip;

    int try_connect_num = 0, disconnect_num = 0;
};

//编码
template <typename T>
int Communication::encodeMsg(int8_t send_mode, T msg)
{
    std::ostringstream os;
    boost::archive::text_oarchive oa(os);
    oa << msg;
    std::string temp = os.str();

    char *ptr;
    if (send_mode == Send_Mode::TCP)
        ptr = tcp_send_buf;
    else if (send_mode == Send_Mode::UDP)
        ptr = udp_send_buf;
    else
        return 0;

    //HEAD:消息帧头
    int8_t HEAD = 97;
    *((int8_t *)ptr) = HEAD;
    ptr += sizeof(int8_t);
    HEAD = 109;
    *((int8_t *)ptr) = HEAD;
    ptr += sizeof(int8_t);

    //LENGTH:PAYLOAD的长度
    uint32_t LENGTH = temp.length();
    *((uint32_t *)ptr) = LENGTH;
    ptr += sizeof(uint32_t);

    //MSG_ID
    uint8_t MSG_ID = getMsgId(msg);
    *((uint8_t *)ptr) = MSG_ID;
    ptr += sizeof(uint8_t);

    //ROBOT_ID
    *((int8_t *)ptr) = ROBOT_ID;
    ptr += sizeof(int8_t);

    //PAYLOAD
    for (int i = 0; i < (int)LENGTH; i++)
    {
        *((char *)ptr) = temp[i];
        ptr += sizeof(char);
    }

    //CHECK
    uint16_t CHECK = 0;
    if (send_mode == Send_Mode::TCP)
        CHECK = checksum(tcp_send_buf, ptr - tcp_send_buf);
    else if (send_mode == Send_Mode::UDP)
        CHECK = checksum(udp_send_buf, ptr - udp_send_buf);
    *((uint16_t *)ptr) = CHECK;
    ptr += sizeof(uint16_t);

    if (send_mode == Send_Mode::TCP)
        return ptr - tcp_send_buf;
    else if (send_mode == Send_Mode::UDP)
        return ptr - udp_send_buf;
    else
        return 0;
}
#if true
//根据传入的struct返回对应的MSG_ID
template <typename T>
uint8_t Communication::getMsgId(T msg)
{
    //判断传入参数属于那个数据类型，然后返回对应的MSG_ID
    if (typeid(msg) == typeid(struct UAVState))
        return MsgId::UAVSTATE;
    else if (typeid(msg) == typeid(struct SwarmCommand))
        return MsgId::SWARMCOMMAND;
    else if (typeid(msg) == typeid(struct TextInfo))
        return MsgId::TEXTINFO;
    else if (typeid(msg) == typeid(struct Heartbeat))
        return MsgId::HEARTBEAT;
    else if (typeid(msg) == typeid(struct ConnectState))
        return MsgId::CONNECTSTATE;
    else if (typeid(msg) == typeid(struct GimbalState))
        return MsgId::GIMBALSTATE;
    else if (typeid(msg) == typeid(struct VisionDiff))
        return MsgId::VISIONDIFF;
    else if (typeid(msg) == typeid(struct MultiDetectionInfo))
        return MsgId::MULTIDETECTIONINFO;
    else if (typeid(msg) == typeid(struct GimbalControl))
        return MsgId::GIMBALCONTROL;
    else if (typeid(msg) == typeid(struct RheaState))
        return MsgId::RHEASTATE;
    else if (typeid(msg) == typeid(struct UAVControlState))
        return MsgId::UAVCONTROLSTATE;
    else if (typeid(msg) == typeid(struct ModeSelection))
        return MsgId::MODESELECTION;
    //rviz 部分数据、相同类型 不同形式（相同的类型，表示不同的东西）自定义MSG_ID
    else if (typeid(msg) == typeid(sensor_msgs::LaserScan))
        return MsgId::UGVLASERSCAN;
    else if (typeid(msg) == typeid(sensor_msgs::PointCloud2))
        return MsgId::UGVPOINTCLOUND2;
    else if (typeid(msg) == typeid(tf2_msgs::TFMessage))
        return MsgId::UGVTFMESSAGE;
    else if (typeid(msg) == typeid(visualization_msgs::MarkerArray))
        return MsgId::UGVMARKERARRAY;
    return 0;
}
#endif
//第二个参数适用于要传输的数据类型相同，但是接收端处理的方式不同所以为了区别msg_id采用自定义。一般情况不使用。
template <typename T>
int Communication::encodeRvizMsg(T msg, int msg_id)
{
    namespace ser = ros::serialization;
    uint32_t serial_size = ros::serialization::serializationLength(msg);
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    ser::OStream stream(buffer.get(), serial_size);
    ser::serialize(stream, msg);

    std::cout << "rviz信息长度:" << serial_size << std::endl;

    char *ptr = rviz_recv_buf;

    //HEAD:消息帧头
    int8_t HEAD = 97;
    *((int8_t *)ptr) = HEAD;
    ptr += sizeof(int8_t);
    HEAD = 109;
    *((int8_t *)ptr) = HEAD;
    ptr += sizeof(int8_t);

    //LENGTH:PAYLOAD的长度
    uint32_t LENGTH = serial_size;
    *((uint32_t *)ptr) = LENGTH;
    ptr += sizeof(uint32_t);

    uint8_t MSG_ID;
    //MSG_ID
    if (msg_id == 0)
        MSG_ID = getMsgId(msg);
    else
        MSG_ID = msg_id;
    MSG_ID = getMsgId(msg);
    *((uint8_t *)ptr) = MSG_ID;
    ptr += sizeof(uint8_t);

    //ROBOT_ID
    *((int8_t *)ptr) = ROBOT_ID;
    ptr += sizeof(int8_t);

    //PAYLOAD
    for (int i = 0; i < (int)LENGTH; i++)
    {
        *((char *)ptr) = buffer[i];
        ptr += sizeof(char);
    }

    //CHECK
    uint16_t CHECK = checksum(rviz_recv_buf, ptr - rviz_recv_buf);
    *((uint16_t *)ptr) = CHECK;
    ptr += sizeof(uint16_t);

    return ptr - rviz_recv_buf;
}

#endif