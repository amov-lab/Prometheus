#ifndef COMUNICATION_HPP
#define COMUNICATION_HPP

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>
#include <sstream>

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

    // 初始化 udp_port组播地址端口号  tcp_port本程序tcp服务端口 
    void init(int id, int multicast_udp_port, int tcp_server_port, int tcp_send_port,bool is_communication = true);

    // 编码
    template <typename T>
    int encodeMsg(int8_t send_mode, T msg,int id = 0);

    // 解码
    int decodeMsg(char *buff,int8_t send_mode);

    // 根据传入的struct返回对应的MSG_ID
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

    struct UAVState getUAVState();
    struct TextInfo getTextInfo();
    struct GimbalState getGimbalState();
    struct VisionDiff getVisionDiff();
    struct Heartbeat getHeartbeat();
    struct RheaState getRheaState();
    struct MultiDetectionInfo getMultiDetectionInfo();
    struct UAVControlState getUAVControlState();
    struct UAVCommand getUAVCommand();
    struct ParamSettings getParamSettings();
    struct SwarmCommand getSwarmCommand();
    
    struct ConnectState getConnectState();
    struct GimbalControl getGimbalControl();
    struct GimbalService getGimbalService();
    struct GimbalParamSet getGimbalParamSet();
    struct WindowPosition getWindowPosition();
    struct RheaControl getRheaControl();
    struct ImageData getImageData();
    struct UAVSetup getUAVSetup();
    struct ModeSelection getModeSelection();
    struct Bspline getBspline();
    struct MultiBsplines getMultiBsplines();
    struct CustomDataSegment_1 getCustomDataSegment_1();
    struct Goal getGoal();
protected:
    int ROBOT_ID = 0;
    int recv_id = 0;

    // tcp/udp
    struct sockaddr_in tcp_addr, udp_addr;
    int tcp_send_sock, udp_send_sock, server_fd, udp_fd, recv_sock, udp_socket, rviz_socket, UDP_PORT, TCP_PORT, TCP_HEARTBEAT_PORT;
    char udp_send_buf[BUF_LEN], udp_recv_buf[BUF_LEN], tcp_send_buf[BUF_LEN], tcp_recv_buf[BUF_LEN];
    std::string udp_ip, multicast_udp_ip;

    int try_connect_num = 0, disconnect_num = 0;

    bool is_communication_node = true;

private:
    struct SwarmCommand recv_swarm_command_;
    struct UAVState recv_uav_state_;
    struct Heartbeat recv_heartbeat_;
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
    struct MultiDetectionInfo recv_multi_detection_info_;
    struct UAVControlState recv_uav_control_state_;
    struct CustomDataSegment_1 recv_custom_data_1_;
};

#endif
