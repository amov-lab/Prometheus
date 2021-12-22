#include <ros/ros.h>
#include <string>
#include <map>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <tf/transform_datatypes.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/ControlCommand.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#define BUF_LEN 100 // 100 bytes

enum Send_Msg_Type : int8_t {DroneStateHead = 1, UgvStateHead, Case2ResultHead, Case3ResultHead} send_msg_type;
enum Rcv_Msg_Type : int8_t {Msg101 = 101, Msg102, Msg103, Msg104, Msg31} rcv_msg_type;
enum Rcv_Msg_Length : int8_t {Msg1Length = 45, Msg101Length = 16, Msg102Length = 3, Msg103Length = 15, Msg104Length = 3, Msg31Length = 14} rcv_msg_length;

class ground_station_bridge
{
public:
    ground_station_bridge(ros::NodeHandle &nh)
    {
        // tcp/udp
        nh.param<std::string>("ground_stationt_ip",udp_ip,"192.168.1.10");
        nh.param<int>("udp_port",UDP_PORT,8887);
        nh.param<int>("tcp_port",TCP_PORT,8888);
        nh.param<int>("uav_id",uav_id,0);
        // debug mode
        nh.param<bool>("debug_mode",debug_mode,false);

        if(uav_id == 0)
        {
            ROS_ERROR("uav_id is not set correctly, shutdown node!");
            ros::shutdown();
        }

        robot_name = "uav";
        robot_id = uav_id;

        // 【订阅】无人机状态 - #1
        drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/"+robot_name+std::to_string(robot_id)+"/prometheus/drone_state",1,&ground_station_bridge::drone_state_cb,this);
        
        // 【订阅】无人机odom - #2 todo
        // drone_odom_sub = nh.subscribe<nav_msgs::Odometry>("/"+robot_name+std::to_string(robot_id)+"/prometheus/drone_odom",1,&ground_station_bridge::drone_odom_cb,this);

        // 【订阅】无人机轨迹 - #3 todo
        // trajectory_sub = nh.subscribe<nav_msgs::Path>("/"+robot_name+std::to_string(robot_id)+"/prometheus/drone_trajectory",1,&ground_station_bridge::drone_trajectory_cb,this);

        // 【发布】无人机指令 - #100
        uav_cmd_pub = nh.advertise<prometheus_msgs::ControlCommand>("/"+robot_name+std::to_string(robot_id)+"/prometheus/control_command",1); 

        // 【发布】其他无人机状态 - #100
        for(int i = 1; i <= 10 && i != robot_id; i++)
        {
            uav_state_pub[i] = nh.advertise<prometheus_msgs::DroneState>("/"+robot_name+std::to_string(i)+"/prometheus/drone_state",1);
        }

        // 线程1：接收消息
        boost::thread recv_thread(&ground_station_bridge::recv_thread_fun,this);
        recv_thread.detach();
        ros::Duration(1).sleep(); // wait

        // 线程2：发送消息至其他无人机
        boost::thread send_thread(&ground_station_bridge::send_thread_fun,this);
        send_thread.detach();
        ros::Duration(1).sleep(); // wait

        drone_state_timeout = 0;
    }

    ~ground_station_bridge()
    {
        // close socket
        close(tcp_send_sock);
        close(udp_send_sock);
        close(recv_sock);
        close(server_fd);
    }

    // 订阅无人机状态
    void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
    {
        // drone_state发布的频率是50Hz
        to_drone_msg = *msg;
        if(drone_state_timeout < 10) drone_state_timeout++;
        else 
        {
            drone_state_timeout = 0;
            // 通过UDP发送给地面站，此处定时是为了降低发送频率（#1）
            send_msg_by_udp(encode_msg_1(msg),udp_ip);
        }
    }

    // 接收的消息线程
    void recv_thread_fun()
    {
        int valread;
        if(wait_connection_from_ground_station(TCP_PORT) < 0)
        {
            ROS_ERROR("[bridge_node]Socket recever creation error!");
            exit(EXIT_FAILURE);
        }

        while(true)
        {
            if((recv_sock = accept(server_fd,(struct sockaddr *)NULL,NULL) )< 0)
            {
                perror("accept");
                exit(EXIT_FAILURE);
            }

            valread = recv(recv_sock,tcp_recv_buf,BUF_LEN,0);

            if(valread <= 0)
            {
                ROS_ERROR("Received message length <= 0, maybe connection has lost");
                close(recv_sock);
                continue;
            }

            int8_t HEAD = *((int8_t*)tcp_recv_buf);

            // decode_msg_xxx
            if(HEAD == Rcv_Msg_Type::Msg101) decode_msg_101(valread);
            else if(HEAD == Rcv_Msg_Type::Msg102) decode_msg_102(valread);
            else if(HEAD == Rcv_Msg_Type::Msg103) decode_msg_103(valread);
            else if(HEAD == Rcv_Msg_Type::Msg104) decode_msg_104(valread);
            else if(HEAD == Send_Msg_Type::DroneStateHead) decode_msg_1(valread);
            else ROS_ERROR("Unknown HEAD %d",HEAD);

            close(recv_sock);
        }
    }
    void decode_msg_101(int msg_len)
    {
        if(msg_len != Rcv_Msg_Length::Msg101Length)
        {
            ROS_ERROR("[msg_101]Wrong Msg Length!");
            return;
        }

        char* ptr = tcp_recv_buf;

        ptr += sizeof(int8_t); // head

        msg_robot_id = *((int8_t*)ptr); // id
        ptr += sizeof(int8_t);

        if(msg_robot_id != robot_id)
        {
            ROS_ERROR("[msg_101]Wrong ROBOT_ID:%d",msg_robot_id);
            return;
        }

        Command = *((int8_t*)ptr);
        ptr += sizeof(int8_t);

        Mode = *((int8_t*)ptr);
        ptr += sizeof(int8_t);   

        des_x = *((float*)ptr) * 100.0;
        ptr += sizeof(float);

        des_y = *((float*)ptr) * 100.0;
        ptr += sizeof(float);

        des_z = *((float*)ptr) * 100.0;
        ptr += sizeof(float);

        if(debug_mode)
        {
            printf("Get msg_101: Command: [%d] Mode: [%d] ",Command,Mode);
            printf("des: [%.2f %.2f %.2f]\n",des_x,des_y,des_z);
        }

        // 目前默认使用Move，XYZ_POS；即只支持指点飞行，无法控制起飞、降落等等
        uav_cmd_msg_101.Mode = prometheus_msgs::ControlCommand::Move;
        uav_cmd_msg_101.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_POS;
        uav_cmd_msg_101.Reference_State.position_ref[0] = des_x;
        uav_cmd_msg_101.Reference_State.position_ref[1] = des_y;
        uav_cmd_msg_101.Reference_State.position_ref[2] = des_z;
        uav_cmd_msg_101.Reference_State.yaw_ref = 0.0;
        uav_cmd_pub.publish(uav_cmd_msg_101);
    }
    void decode_msg_102(int msg_len)
    {
        if(msg_len != Rcv_Msg_Length::Msg102Length)
        {
            ROS_ERROR("[msg_102]Wrong Msg Length!");
            return;
        }

        // TODO
    }
    void decode_msg_103(int msg_len)
    {
        if(msg_len != Rcv_Msg_Length::Msg103Length)
        {
            ROS_ERROR("[msg_103]Wrong Msg Length!");
            return;
        }
        
        // TODO
    }
    void decode_msg_104(int msg_len)
    {
        if(msg_len != Rcv_Msg_Length::Msg104Length)
        {
            ROS_ERROR("[msg_104]Wrong Msg Length!");
            return;
        }
        // TODO
    }
    void decode_msg_1(int msg_len)
    {
        if(msg_len != Rcv_Msg_Length::Msg1Length)
        {
            ROS_ERROR("[msg_1]Wrong Msg Length!");
            return;
        }

        char* ptr = tcp_recv_buf;

        ptr += sizeof(int8_t);

        msg_robot_id = *((int8_t*)ptr);
        ptr += sizeof(int8_t);

        if(msg_robot_id > 8 || msg_robot_id < 1)
        {
            ROS_ERROR("[msg_1]Wrong ROBOT_ID:%d",msg_robot_id);
            return;
        }
        
        for(int i = 0; i <3; ++i){
            msg_1.position[i] = *((float*)ptr);
            ptr += sizeof(float);
        }

        for(int i = 0; i <3; ++i){
            msg_1.velocity[i] = *((float*)ptr);
            ptr += sizeof(float);
        }

        for(int i = 0; i <3; ++i){
            msg_1.attitude[i] = *((float*)ptr);
            ptr += sizeof(float);
        }

        msg_1.connected = *((int8_t*)ptr);
        ptr += sizeof(int8_t);

        msg_1.armed = *((int8_t*)ptr);
        ptr += sizeof(int8_t);

        msg_1.mode = *((int8_t*)ptr);
        ptr += sizeof(int8_t);

        msg_1.battery_state = *((int8_t*)ptr);
        ptr += sizeof(float);

        if(debug_mode) printf("Get msg_1 from [%d]\n",msg_robot_id);

        uav_state_pub[msg_robot_id].publish(msg_1);
    }

    void send_thread_fun()
    {
        while(ros::ok())
        {
            if(debug_mode) printf("send to drone! ");
            int len = encode_msg_1(to_drone_msg);
            for(int i = 1; i < 9; ++i)
            {
                if(i == robot_id) continue;
                send_msg_by_tcp(len,uav_ip_pair[i]);
            }
        }
    }

    // UDP client
    int connect_to_ground_station(const char *ip, const int port)
    {
        int fd = socket(PF_INET,SOCK_DGRAM,0);
        if(fd < 0)
        {
            printf("Socket creation error \n");
            return -1;
        }
  
        memset(&udp_addr,0,sizeof(udp_addr));
        udp_addr.sin_family = AF_INET;
        udp_addr.sin_port = htons(port);
        udp_addr.sin_addr.s_addr = inet_addr(ip);

        // Convert IPv4 and IPv6 addresses from text to binary form
        if(inet_pton(AF_INET,ip,&udp_addr.sin_addr) <= 0)
        {
            printf("Invalid address/ Address not supported \n");
            return -1;
        }

        return fd;
    }

    // TCP client
    int connect_to_drone(const char *ip, const int port)
    {
        int fd = socket(AF_INET,SOCK_STREAM,0);
        if(fd < 0)
        {
            printf("Socket creation error \n");
            return -1;
        }

        // struct timeval tcp_timeout = {0,500000};
        // setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (char*)&tcp_timeout, sizeof(struct timeval));

        memset(&tcp_addr,0,sizeof(tcp_addr));
        tcp_addr.sin_family = AF_INET;
        tcp_addr.sin_port = htons(port);
        tcp_addr.sin_addr.s_addr = inet_addr(ip);

        // Convert IPv4 and IPv6 addresses from text to binary form
        if(inet_pton(AF_INET,ip,&tcp_addr.sin_addr) <= 0)
        {
            printf("Invalid address/ Address not supported \n");
            return -1;
        }

        if(connect(fd, (struct sockaddr*)&tcp_addr, sizeof(tcp_addr)) < 0)
        {
            printf("connect error: %s(errno: %d)\n",strerror(errno),errno);
            return -1;
        }

        return fd;
    }
    // 通过UDP发送消息
    void send_msg_by_udp(int msg_len, std::string target_ip)
    {
        udp_send_sock = connect_to_ground_station(target_ip.c_str(),UDP_PORT);
        sendto(udp_send_sock,udp_send_buf,msg_len,0,(struct sockaddr*)&udp_addr,sizeof(udp_addr));
        close(udp_send_sock);
    }
    // 通过TCP发送消息
    void send_msg_by_tcp(int msg_len, std::string target_ip)
    {
        tcp_send_sock = connect_to_drone(target_ip.c_str(),TCP_PORT);
        send(tcp_send_sock, udp_send_buf, msg_len, 0);
        close(tcp_send_sock);
    }
    int encode_msg_1(const prometheus_msgs::DroneState::ConstPtr& msg)
    {
        char *ptr = udp_send_buf;
        
        *((int8_t *)ptr) = Send_Msg_Type::DroneStateHead; // msg head
        ptr += sizeof(int8_t);
        
        *((int8_t *)ptr) = uav_id; // id
        ptr += sizeof(int8_t);

        *((float *)ptr) = msg->position[0]; //position
        ptr += sizeof(float);
        *((float *)ptr) = msg->position[1];
        ptr += sizeof(float);
        *((float *)ptr) = msg->position[2];
        ptr += sizeof(float);

        *((float *)ptr) = msg->velocity[0]; //velocity
        ptr += sizeof(float);
        *((float *)ptr) = msg->velocity[1];
        ptr += sizeof(float);
        *((float *)ptr) = msg->velocity[2];
        ptr += sizeof(float);

        *((float *)ptr) = msg->attitude[0]; //attitude
        ptr += sizeof(float);
        *((float *)ptr) = msg->attitude[1];
        ptr += sizeof(float);
        *((float *)ptr) = msg->attitude[2];
        ptr += sizeof(float);

        *((int8_t *)ptr) = msg->connected; //status
        ptr += sizeof(int8_t);
        *((int8_t *)ptr) = int8_t(msg->armed);
        ptr += sizeof(int8_t);
        *((int8_t *)ptr) = uav_mode_pair[msg->mode];
        ptr += sizeof(int8_t);
        *((float *)ptr) = msg->battery_state;
        ptr += sizeof(float);

        // if(debug_mode) 
        // {
        //     printf("Send drone state msg: \n head: %d \t uav_%d ",Send_Msg_Type::DroneStateHead,uav_id);
        //     printf("\tpos:[%.2f %.2f %.2f]",msg->position[0],msg->position[1],msg->position[2]);
        //     printf("\tvel:[%.2f %.2f %.2f]",msg->velocity[0],msg->velocity[1],msg->velocity[2]);
        //     printf("\tatt:[%.2f %.2f %.2f]",msg->attitude[0],msg->attitude[1],msg->attitude[2]);
        //     printf(" connect: [%d] arm: %d mode_num: %d battery_state: %.2f\n",msg->connected,msg->armed,uav_mode_pair[msg->mode],msg->battery_state);
        // }

        return ptr - udp_send_buf; // msg length
    }

    int encode_msg_1(const prometheus_msgs::DroneState msg)
    {
        char *ptr = udp_send_buf;
        
        *((int8_t *)ptr) = Send_Msg_Type::DroneStateHead; // msg head
        ptr += sizeof(int8_t);
        
        *((int8_t *)ptr) = uav_id; // id
        ptr += sizeof(int8_t);

        *((float *)ptr) = msg.position[0]; //position
        ptr += sizeof(float);
        *((float *)ptr) = msg.position[1];
        ptr += sizeof(float);
        *((float *)ptr) = msg.position[2];
        ptr += sizeof(float);

        *((float *)ptr) = msg.velocity[0]; //velocity
        ptr += sizeof(float);
        *((float *)ptr) = msg.velocity[1];
        ptr += sizeof(float);
        *((float *)ptr) = msg.velocity[2];
        ptr += sizeof(float);

        *((float *)ptr) = msg.attitude[0]; //attitude
        ptr += sizeof(float);
        *((float *)ptr) = msg.attitude[1];
        ptr += sizeof(float);
        *((float *)ptr) = msg.attitude[2];
        ptr += sizeof(float);

        *((int8_t *)ptr) = msg.connected; //status
        ptr += sizeof(int8_t);
        *((int8_t *)ptr) = int8_t(msg.armed);
        ptr += sizeof(int8_t);
        *((int8_t *)ptr) = uav_mode_pair[msg.mode];
        ptr += sizeof(int8_t);
        *((float *)ptr) = msg.battery_state;
        ptr += sizeof(float);

        // if(debug_mode) 
        // {
        //     printf("Send drone state msg: \n head: %d \t uav_%d ",Send_Msg_Type::DroneStateHead,uav_id);
        //     printf("\tpos:[%.2f %.2f %.2f]",msg.position[0],msg.position[1],msg.position[2]);
        //     printf("\tvel:[%.2f %.2f %.2f]",msg.velocity[0],msg.velocity[1],msg.velocity[2]);
        //     printf("\tatt:[%.2f %.2f %.2f]",msg.attitude[0],msg.attitude[1],msg.attitude[2]);
        //     printf(" connect: [%d] arm: %d mode_num: %d battery_state: %.2f\n",msg.connected,msg.armed,uav_mode_pair[msg.mode],msg.battery_state);
        // }

        return ptr - udp_send_buf; // msg length
    }

    // TCP server
    int wait_connection_from_ground_station(const int port)
    {   
        struct sockaddr_in address;
        memset(&address,0,sizeof(address));
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_ANY);
        address.sin_port = htons(TCP_PORT);

        // Creating socket file descriptor
        if((server_fd = socket(AF_INET,SOCK_STREAM,0)) == 0)
        {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }

        // Forcefully attaching socket to the port
        if(bind(server_fd,(struct sockaddr *)&address,sizeof(address)) < 0)
        {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }

        if(listen(server_fd, BUF_LEN) < 0)
        {
            perror("listen");
            exit(EXIT_FAILURE);
        }

        return server_fd;
    }


private:
    // ros
    ros::Subscriber drone_state_sub;
    ros::Publisher uav_cmd_pub;
    ros::Publisher uav_state_pub[11];

    // msg
    prometheus_msgs::ControlCommand uav_cmd_msg_101;
    prometheus_msgs::DroneState msg_1;
    prometheus_msgs::DroneState to_drone_msg;

    int8_t msg_robot_id, Command, Mode;
    float des_x, des_y, des_z;
    
    // tcp/udp
    std::string udp_ip, tcp_ip;
    struct sockaddr_in tcp_addr,udp_addr;
    int tcp_send_sock, udp_send_sock, server_fd, recv_sock, UDP_PORT, TCP_PORT;
    char udp_send_buf[BUF_LEN], udp_recv_buf[BUF_LEN], tcp_send_buf[BUF_LEN], tcp_recv_buf[BUF_LEN];

    // id
    std::string robot_name;
    int uav_id, robot_id;

    // others
    bool debug_mode;
    int drone_state_timeout;

    // predefine msg cmd pair (using std::map)
    std::map<std::string,int8_t> uav_mode_pair = {  {"MANUAL",0},{"OFFBOARD",1},{"STABILIZED",2},{"ACRO",3},{"ALTCTL",4},{"POSCTL",5},
                                                    {"RATTITUDE",6},{"AUTO.MISSION",7},{"AUTO.LOITER",8},{"AUTO.RTL",9},{"AUTO.LAND",10},
                                                    {"AUTO.RTGS",11},{"AUTO.READY",12},{"AUTO.TAKEOFF",13}};

    std::map<int8_t,std::string> uav_ip_pair = {{1,"192.168.1.11"},{2,"192.168.1.12"},{3,"192.168.1.13"},{4,"192.168.1.14"},
                                                {5,"192.168.1.15"},{6,"192.168.1.16"},{7,"192.168.1.17"},{8,"192.168.1.18"}};
};

int main(int argc, char **argv) 
{
    ros::init(argc,argv,"prometheus_bridge");
    ros::NodeHandle nh("~");

    printf("\033[1;32m---->[prometheus_bridge] start running\n\033[0m");

    ground_station_bridge mb(nh);

    ros::spin();

    return 0;
}