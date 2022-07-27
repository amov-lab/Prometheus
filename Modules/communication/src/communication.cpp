#include "communication.hpp"

Communication::Communication(ros::NodeHandle &nh)
{
    // tcp/udpping
    nh.param<int>("udp_port", UDP_PORT, 8889);
    nh.param<int>("tcp_port", TCP_PORT, 55555);
    nh.param<int>("tcp_heartbeat_port", TCP_HEARTBEAT_PORT, 55556);
    nh.param<int>("rviz_port", RVIZ_PORT, 8890);
    nh.param<int>("ROBOT_ID", ROBOT_ID, 1);
    nh.param<std::string>("ground_stationt_ip", udp_ip, "127.0.0.1");
    nh.param<std::string>("multicast_udp_ip", multicast_udp_ip, "224.0.0.88");
    nh.param<int>("try_connect_num", try_connect_num, 3);
}

Communication::~Communication()
{
    // close socket
    close(tcp_send_sock);
    close(udp_send_sock);
    close(recv_sock);
    close(server_fd);
    close(udp_socket);
    close(rviz_socket);
}

// UDP client
int Communication::connectToUdpMulticast(const char *ip, const int port)
{
    int fd = socket(PF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        printf("Socket creation error \n");
        return -1;
    }

    memset(&udp_addr, 0, sizeof(udp_addr));
    udp_addr.sin_family = AF_INET;
    udp_addr.sin_port = htons(port);
    udp_addr.sin_addr.s_addr = inet_addr(ip);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if (inet_pton(AF_INET, ip, &udp_addr.sin_addr) <= 0)
    {
        printf("Invalid address/ Address not supported \n");
        return -1;
    }

    return fd;
}

// TCP client  返回套接字
int Communication::connectToDrone(const char *ip, const int port)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
    {
        printf("Socket creation error \n");
        return -1;
    }

    // struct timeval tcp_timeout = {0,500000};
    // setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (char*)&tcp_timeout, sizeof(struct timeval));

    memset(&tcp_addr, 0, sizeof(tcp_addr));
    tcp_addr.sin_family = AF_INET;
    tcp_addr.sin_port = htons(port);
    tcp_addr.sin_addr.s_addr = inet_addr(ip);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if (inet_pton(AF_INET, ip, &tcp_addr.sin_addr) <= 0)
    {
        printf("Invalid address/ Address not supported \n");
        return -1;
    }

    if (connect(fd, (struct sockaddr *)&tcp_addr, sizeof(tcp_addr)) < 0)
    {
        printf("connect error: %s(errno: %d)\n", strerror(errno), errno);

        disconnect_num++;

        return -1;
    }
    disconnect_num = 0;
    return fd;
}

void Communication::sendMsgByUdp(int msg_len, std::string target_ip)
{
    //std::cout << "udp len: " << msg_len << std::endl;
    udp_send_sock = connectToUdpMulticast(target_ip.c_str(), UDP_PORT);
    sendto(udp_send_sock, udp_send_buf, msg_len, 0, (struct sockaddr *)&udp_addr, sizeof(udp_addr));
    close(udp_send_sock);
    usleep(100);
}

void Communication::sendMsgByUdp(int msg_len, const char* udp_msg , std::string target_ip, int port)
{
    udp_send_sock = connectToUdpMulticast(target_ip.c_str(), port);
    sendto(udp_send_sock, udp_msg , msg_len, 0, (struct sockaddr *)&udp_addr, sizeof(udp_addr));
    close(udp_send_sock);
    usleep(100);
}

void Communication::sendMsgByTcp(int msg_len, std::string target_ip)
{
    //std::cout << "tcp len: " << msg_len <<std::endl;
    tcp_send_sock = connectToDrone(target_ip.c_str(), TCP_HEARTBEAT_PORT);
    send(tcp_send_sock, tcp_send_buf, msg_len, 0);
    close(tcp_send_sock);
}

void Communication::sendRvizByUdp(int msg_len, std::string target_ip)
{
    //std::cout << "rviz:" << msg_len << std::endl;
    rviz_socket = connectToUdpMulticast(target_ip.c_str(), RVIZ_PORT);
    char *ptr = rviz_recv_buf;
    if (msg_len < BUF_LEN)
        sendto(rviz_socket, rviz_recv_buf, msg_len, 0, (struct sockaddr *)&udp_addr, sizeof(udp_addr));
    else
    {
        int len = msg_len;
        while (true)
        {
            //std::cout << "len: " << len << std::endl;
            len = len - BUF_LEN;
            if (len > 0)
            {
                sendto(rviz_socket, ptr, BUF_LEN, 0, (struct sockaddr *)&udp_addr, sizeof(udp_addr));
            }
            else if (len < 0)
            {
                sendto(rviz_socket, ptr, len + BUF_LEN, 0, (struct sockaddr *)&udp_addr, sizeof(udp_addr));
                break;
            }
            //偏移量
            ptr += BUF_LEN;
            usleep(100);
        }
    }
    close(rviz_socket);
}

// TCP server
int Communication::waitConnectionFromGroundStation(const int port)
{
    struct sockaddr_in address;
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(TCP_PORT);

    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    //监听
    if (listen(server_fd, BUF_LEN) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }

    return server_fd;
}

// UDP server
int Communication::waitConnectionFromMulticast(const int port)
{
    udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    //local_addr.sin_addr.s_addr = inet_addr(local_ip.c_str());//指定发送的网口
    local_addr.sin_port = htons(port);

    if (bind(udp_socket, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0)
    {
        perror("udp bind failed");
        exit(EXIT_FAILURE);
    }

    return udp_socket;
}

unsigned short Communication::checksum(char *buff, int len)
{
    unsigned short crc = CRC::Calculate(buff, len, CRC::CRC_16_ARC());
    return crc;
}
