#ifndef COMUNICATION_HPP
#define COMUNICATION_HPP

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>
#include <sstream>

#include "CRC.h"
#include "Struct.hpp"


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
    Communication();
    ~Communication();

    void init(int id, int udp_port, int tcp_port, int tcp_heart_port);

    //编码
    template <typename T>
    int encodeMsg(int8_t send_mode, T msg,int id = 0);

    //解码
    int decodeMsg(char *buff,int8_t send_mode);

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

    // TCP server
    int waitConnectionFromGroundStation(const int port);

    // UDP server
    int waitConnectionFromMulticast(const int port);

    unsigned short checksum(char *buff, int len);

protected:
    int ROBOT_ID = 0;
    int recv_id = 0;

    // tcp/udp
    struct sockaddr_in tcp_addr, udp_addr;
    int tcp_send_sock, udp_send_sock, server_fd, udp_fd, recv_sock, udp_socket, rviz_socket, UDP_PORT, TCP_PORT, TCP_HEARTBEAT_PORT;
    char udp_send_buf[BUF_LEN], udp_recv_buf[BUF_LEN], tcp_send_buf[BUF_LEN], tcp_recv_buf[BUF_LEN];
    std::string udp_ip, multicast_udp_ip;

    int try_connect_num = 0, disconnect_num = 0;

public:
    struct SwarmCommand recv_swarm_command_;
    struct UAVState recv_uav_state_;
    struct ConnectState recv_connect_state_;
    struct GimbalControl recv_gimbal_control_;
    struct ModeSelection recv_mode_selection_;
    struct GimbalService recv_gimbal_service_;
    struct WindowPosition recv_window_position_;
    struct RheaControl recv_rhea_control_;
    struct GimbalParamSet recv_param_set_;
    struct RheaState recv_rhea_state_;
    struct ImageData recv_image_data_;
    struct UAVCommand recv_uav_cmd_;
    struct UAVSetup recv_uav_setup_;
    struct TextInfo recv_text_info_;
    struct GimbalState recv_gimbal_state_;
    struct VisionDiff recv_vision_diff_;
    struct ParamSettings recv_param_settings_;
    struct Bspline recv_bspline_;
    struct MultiBsplines recv_multi_bsplines_;
    struct Goal recv_goal_;
    struct CustomDataSegment_1 recv_custom_data_1_;
};

#endif
