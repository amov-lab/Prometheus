#ifndef COMUNICATION_HPP
#define COMUNICATION_HPP

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>
#include <sstream>
// 包含数据的结构体、结构体对应的ID信息等
#include "Struct.hpp"

#define BUF_LEN 1024 * 10 // 1024*10 bytes
#define SERV_PORT 20168

typedef unsigned char byte;

// 发送方式
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

    /**
     * @brief 初始化函数
     * 
     * @param id 通信节点ID(跟无人机ID对应)
     * @param multicast_udp_port 组播地址端口号
     * @param tcp_server_port TCP服务器端口号
     * @param tcp_send_port TCP发送到的目标端口
     * @param is_communication 是否是通信节点(机载端)
     * 
     * 在机载端示例代码：
     * Communication communication;
     * communication.init(1,8889,55555,55556);
     * 
     * 与之对应的在地面端示例代码：
     * Communication communication;
     * communication.init(1,8889,55556,55555,false);
     */
    void init(int id, int multicast_udp_port, int tcp_server_port, int tcp_send_port,bool is_communication = true);

    /**
     * @brief 模版函数，编码函数
     * 
     * @param send_mode 发送方式，TCP或者UDP
     * @param msg 进行编码的结构体
     * @param id 该数据的发送者ID，输入为0时，则按初始化函数中的无人机ID为准
     * @return int 返回编码后的长度
     */
    template <typename T>
    int encodeMsg(int8_t send_mode, T msg,int id = 0);

    /**
     * @brief 解码函数
     * 
     * @param buff 接收到的数据
     * @param send_mode 接收到的方式，TCP或者UDP
     * @return int 返回解码后得到的结构体对应的ID
     */
    int decodeMsg(char *buff,int8_t send_mode);

    /**
     * @brief 返回结构体ID
     * 
     * @param msg 结构体
     * @return uint8_t 结构体对应的ID
     */
    template <typename T>
    uint8_t getMsgId(T msg);

    /**
     * @brief 连接组播或udp服务器并返回其socket标识符
     * 
     * @param ip 组播或目标端的地址
     * @param port 端口号
     * @return int 返回udp客户端的socket标识符
     */
    int connectToUdpMulticast(const char *ip, const int port);

    /**
     * @brief 连接TCP服务端并返回其socket标识符
     * 
     * @param ip TCP服务端的地址
     * @param port 端口号
     * @return int 返回tcp客户端的socket标识符
     */
    int connectToDrone(const char *ip, const int port);

    /**
     * @brief UDP发送函数
     * 
     * @param msg_len 发送的数据的长度
     * @param target_ip 发送的目标地址
     */
    void sendMsgByUdp(int msg_len, std::string target_ip);

    /**
     * @brief UDP发送函数
     * 
     * @param msg_len 发送的数据的长度
     * @param udp_msg 发送的具体数据
     * @param target_ip 目标地址
     * @param port 目标端口
     */
    void sendMsgByUdp(int msg_len, const char* udp_msg ,std::string target_ip, int port);

    /**
     * @brief TCP发送函数
     * 
     * @param msg_len 发送的数据的长度
     * @param target_ip 发送的目标地址
     */
    void sendMsgByTcp(int msg_len, std::string target_ip);

    /**
     * @brief TCP服务端并返回socket标识符
     * 
     * @param port 端口号
     * @return int TCP服务端socket标识符
     */
    int waitConnectionFromGroundStation(const int port);

    /**
     * @brief UDP服务端并返回socket标识符
     * 
     * @param port 端口号
     * @return int UDP服务端socket标识符
     */
    int waitConnectionFromMulticast(const int port);

    /**
     * @brief 校验函数
     * 
     * @param buff 需要校验的数据
     * @param len 需要校验数据的长度
     * @return unsigned short 返回校验后的结果
     */
    unsigned short checksum(char *buff, int len);

    /**
     * @brief 返回接收到的数据的发送方ID
     * 
     * @return int 返回发送方ID
     */
    int getRecvID();

    /**
     * @brief 返回接收到的解码后的结构体
     * 
     * @return 返回各类型的结构体
     * 
     * 示例代码：
     * 其中 buff 为 udp接收到的数据：
     * if(decodeMsg(buff,Send_Mode::UDP) == MsgId::UAVSTATE)
     * {
     *      struct UAVState uav_state = getUAVState();
     * }
     * ...
     */
    struct UAVState getUAVState();
    struct TextInfo getTextInfo();
    struct GimbalState getGimbalState();
    struct VisionDiff getVisionDiff();
    struct Heartbeat getHeartbeat();
    struct UGVState getUGVState();
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
    struct UGVCommand getUGVCommand();
    struct ImageData getImageData();
    struct UAVSetup getUAVSetup();
    struct ModeSelection getModeSelection();
    struct Bspline getBspline();
    struct MultiBsplines getMultiBsplines();
    struct CustomDataSegment_1 getCustomDataSegment_1();
    struct Goal getGoal();
protected:
    // 无人机的ID（通信节点的ID），初始值为0
    int ROBOT_ID = 0;
    // 接收到数据后，通过解码得到的该数据由无人机的ID（通信节点ID）发送，初始值为0
    int recv_id = 0;

    // tcp/udp 相关变量
    struct sockaddr_in tcp_addr, udp_addr;
    int tcp_send_sock, udp_send_sock, server_fd, udp_fd, recv_sock, udp_socket, rviz_socket, UDP_PORT, TCP_PORT, TCP_HEARTBEAT_PORT;
    char udp_send_buf[BUF_LEN], udp_recv_buf[BUF_LEN], tcp_send_buf[BUF_LEN], tcp_recv_buf[BUF_LEN];
    std::string udp_ip, multicast_udp_ip;

    // TCP客户端连接服务端如果失败，尝试重新连接的次数，初始值为0；重新连接多少次后判断为断联，此时触发安全保护信号（降落或返航），初始值为0
    int try_connect_num = 0, disconnect_num = 0;

    // 是否是通信节点(机载端)，初始值为true
    bool is_communication_node = true;

private:
    // 接收到数据，解码后保存的数据
    struct SwarmCommand recv_swarm_command_;
    struct UAVState recv_uav_state_;
    struct Heartbeat recv_heartbeat_;
    struct ConnectState recv_connect_state_;
    struct GimbalControl recv_gimbal_control_;
    struct ModeSelection recv_mode_selection_;
    struct GimbalService recv_gimbal_service_;
    struct WindowPosition recv_window_position_;
    struct UGVCommand recv_ugv_command_;
    struct GimbalParamSet recv_param_set_;
    struct UGVState recv_ugv_state_;
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