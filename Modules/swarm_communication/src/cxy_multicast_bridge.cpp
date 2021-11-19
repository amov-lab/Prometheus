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
#include <prometheus_msgs/UgvState.h>
#include <prometheus_msgs/SwarmCommand.h>
#include <prometheus_msgs/UgvCommand.h>
#include <prometheus_msgs/StationCommand.h>
#include <prometheus_msgs/Case2Result.h>
#include <prometheus_msgs/Case3Result.h>
#include <prometheus_msgs/Msg103.h>
#include <prometheus_msgs/Msg104.h>
#include <nav_msgs/Odometry.h>

#define BUF_LEN 100 // 100 bytes

enum Send_Msg_Type : int8_t {DroneStateHead = 1, UgvStateHead, Case2ResultHead, Case3ResultHead} send_msg_type;
enum Rcv_Msg_Type : int8_t {Msg101 = 101, Msg102, Msg103, Msg104, Msg31} rcv_msg_type;
enum Rcv_Msg_Length : int8_t {Msg1Length = 45, Msg101Length = 16, Msg102Length = 3, Msg103Length = 15, Msg104Length = 3, Msg31Length = 14} rcv_msg_length;

class multicast_bridge
{
public:
    multicast_bridge(ros::NodeHandle &nh)
    {
        // tcp/udp
        nh.param<std::string>("ground_stationt_ip",udp_ip,"192.168.1.10");
        nh.param<int>("udp_port",UDP_PORT,8887);
        nh.param<int>("tcp_port",TCP_PORT,8888);
        nh.param<int>("case_flag",case_flag,0);
        nh.param<int>("uav_id",uav_id,0);
        nh.param<int>("ugv_id",ugv_id,0);
        // for ugv2ugv
        nh.param<int>("other_ugv_id",other_ugv_id,0);
        nh.param<std::string>("other_ugv_ip",other_ugv_ip,"192.168.1.0");
        // debug mode
        nh.param<bool>("debug_mode",debug_mode,false);
        // swarm
        nh.param<float>("formation_size",formation_size,1.0);

        if(ugv_id == 0 && uav_id == 0 || ugv_id != 0 && uav_id != 0)
        {
            ROS_ERROR("ugv_id/uav_id is not set correctly, shutdown node!");
            ros::shutdown();
        }

        if(uav_id != 0) // UAV
        {
            robot_name = "uav";
            robot_id = uav_id;
            rest = 9;
            pub_msg_101 = &multicast_bridge::pub_uav_msg_101;
            pub_msg_103 = &multicast_bridge::pub_uav_msg_103;
            drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/"+robot_name+std::to_string(robot_id%rest)+"/prometheus/drone_state",1,&multicast_bridge::drone_state_cb,this);
            uav_cmd_pub = nh.advertise<prometheus_msgs::SwarmCommand>("/"+robot_name+std::to_string(robot_id%rest)+"/prometheus/swarm_command",1); // publish(msg_101)(msg_102)
            for(int i = 1; i < 8 && i != robot_id; i++)
            {
                uav_state_pub[i] = nh.advertise<prometheus_msgs::DroneState>("/"+robot_name+std::to_string(i)+"/prometheus/drone_state",1);
            }
        }
        else // UGV
        {
            robot_name = "ugv";
            robot_id = ugv_id + 8;
            rest = 8;
            pub_msg_101 = &multicast_bridge::pub_ugv_msg_101;
            pub_msg_103 = &multicast_bridge::pub_ugv_msg_103;
            ugv_state_sub = nh.subscribe<prometheus_msgs::UgvState>("/"+robot_name+std::to_string(robot_id%rest)+"/prometheus/ugv_state",1,&multicast_bridge::ugv_state_cb,this);
            ugv_odom_sub = nh.subscribe<nav_msgs::Odometry>("/"+robot_name+std::to_string(robot_id%rest)+"/prometheus/ugv_odom",1,&multicast_bridge::ugv_odom_cb,this);  
            ugv_cmd_pub = nh.advertise<prometheus_msgs::UgvCommand>("/"+robot_name+std::to_string(robot_id%rest)+"/prometheus/ugv_command",1); // publish(msg_101)
            if(other_ugv_id != 0) other_ugv_odom_pub = nh.advertise<nav_msgs::Odometry>("/"+robot_name+std::to_string(other_ugv_id)+"/prometheus/ugv_odom",10);
            else ROS_ERROR("other_ugv_id is not set correctly, but node still working!");
        }

        case2_result_sub = nh.subscribe<prometheus_msgs::Case2Result>("/"+robot_name+std::to_string(robot_id%rest)+"/ground_station/"+robot_name+"_result_case2",10,&multicast_bridge::case2_result_cb,this);
        case3_result_sub = nh.subscribe<prometheus_msgs::Case3Result>("/"+robot_name+std::to_string(robot_id%rest)+"/ground_station/"+robot_name+"_result_case3",10,&multicast_bridge::case3_result_cb,this);

        msg103_pub = nh.advertise<prometheus_msgs::Msg103>("/kongdijiqun/msg103", 1);
        msg104_pub = nh.advertise<prometheus_msgs::Msg104>("/kongdijiqun/msg104", 1);

        // thread_receiver
        boost::thread recv_thd(&multicast_bridge::server_fun,this);
        recv_thd.detach();
        ros::Duration(1).sleep(); // wait

        // thread_receiver
        boost::thread to_drone_thd(&multicast_bridge::to_drone_fun,this);
        to_drone_thd.detach();
        ros::Duration(1).sleep(); // wait

        drone_state_timeout = 0;
        ugv_state_timeout = 0;

        // predefine msg 102
        uav_cmd_msg_102.swarm_size = formation_size;
        uav_cmd_msg_102.position_ref[0] = 0.0; 
        uav_cmd_msg_102.position_ref[1] = 0.0;
        uav_cmd_msg_102.position_ref[2] = 1.0;
        uav_cmd_msg_102.yaw_ref = 0.0;
    }

    ~multicast_bridge()
    {
        // close socket
        close(tcp_send_sock);
        close(udp_send_sock);
        close(recv_sock);
        close(server_fd);
    }

    void to_drone_fun()
    {
        while(true && case_flag == 1)
        {
            if(debug_mode) printf("send to drone! ");
            int len = encode_drone_state_msg(to_drone_msg);
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

    void send_msg_by_udp(int msg_len, std::string target_ip)
    {
        udp_send_sock = connect_to_ground_station(target_ip.c_str(),UDP_PORT);
        sendto(udp_send_sock,udp_send_buf,msg_len,0,(struct sockaddr*)&udp_addr,sizeof(udp_addr));
        close(udp_send_sock);
    }

    void send_msg_by_tcp(int msg_len, std::string target_ip)
    {
        tcp_send_sock = connect_to_drone(target_ip.c_str(),TCP_PORT);
        send(tcp_send_sock, udp_send_buf, msg_len, 0);
        close(tcp_send_sock);
    }

    int encode_drone_state_msg(const prometheus_msgs::DroneState::ConstPtr& msg)
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

    int encode_drone_state_msg(const prometheus_msgs::DroneState msg)
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

    void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
    {
        // drone_state发布的频率是50Hz
        to_drone_msg = *msg;
        if(drone_state_timeout < 10) drone_state_timeout++;
        else 
        {
            drone_state_timeout = 0;
            send_msg_by_udp(encode_drone_state_msg(msg),udp_ip);
        }
    }

    int encode_ugv_state_msg(const prometheus_msgs::UgvState::ConstPtr& msg)
    {
        char *ptr = udp_send_buf;

        *((int8_t *)ptr) = Send_Msg_Type::UgvStateHead; // msg head
        ptr += sizeof(int8_t);
        
        *((int8_t *)ptr) = ugv_id; // id
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

        *((int8_t *)ptr) = 1; //connected
        ptr += sizeof(int8_t);

        *((float *)ptr) = msg->battery;
        ptr += sizeof(float);

        // if(debug_mode)
        // {
        //     printf("Send msg_2: head: %d ugv %d",Send_Msg_Type::UgvStateHead,ugv_id);
        //     printf("\tpos:[%.2f %.2f %.2f]",msg->position[0],msg->position[1],msg->position[2]);
        //     printf("\tvel:[%.2f %.2f %.2f]",msg->velocity[0],msg->velocity[1],msg->velocity[2]);
        //     printf("\tatt:[%.2f %.2f %.2f]",msg->attitude[0],msg->attitude[1],msg->attitude[2]);
        //     printf(" connection: [%d] battery: [%.2f]\n",1, msg->battery);
        // }

        return ptr - udp_send_buf; // msg length
    }

    int encode_ugv_odom(const nav_msgs::Odometry::ConstPtr& msg)
    {
        char *ptr = udp_send_buf;
        
        *((int8_t *)ptr) = Rcv_Msg_Type::Msg31; // MSG_ID:31, send to ugv
        ptr += sizeof(int8_t);
        
        *((int8_t *)ptr) = ugv_id; // ugv id
        ptr += sizeof(int8_t); 

        *((float *)ptr) = msg->pose.pose.position.x; //position
        ptr += sizeof(float);
        *((float *)ptr) = msg->pose.pose.position.y;
        ptr += sizeof(float);
        *((float *)ptr) = msg->pose.pose.position.z;
        ptr += sizeof(float);

        if(debug_mode)
        {
            printf("Send ugv odom MSG_ID: [%d] UGV_ID: [%d]",31,ugv_id);
            printf(" position: [%f %f %f]\n",msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
        }
        
        return ptr - udp_send_buf;
    }

    void ugv_state_cb(const prometheus_msgs::UgvState::ConstPtr& msg)
    {
        // std::cout << other_ugv_ip << std::endl;
        // if(other_ugv_id != 0) send_msg_by_tcp(encode_ugv_odom(msg),other_ugv_ip);
        if(ugv_state_timeout < 10) ugv_state_timeout++;
        else
        {
            ugv_state_timeout = 0;
            send_msg_by_udp(encode_ugv_state_msg(msg),udp_ip);
        }
    }

    void ugv_odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // std::cout << other_ugv_ip << std::endl;
        // if(other_ugv_id != 0) send_msg_by_tcp(encode_ugv_odom(msg),other_ugv_ip);
        // if(ugv_state_timeout < 10) ugv_state_timeout++;
        // else
        // {
        //     ugv_state_timeout = 0;
        //     send_msg_by_udp(encode_ugv_state_msg(msg),udp_ip);
        // }
    }

    int encode_case2_result_msg(const prometheus_msgs::Case2Result::ConstPtr& msg)
    {
        char *ptr = udp_send_buf;

        *((int8_t *)ptr) = Send_Msg_Type::Case2ResultHead; // msg head
        ptr += sizeof(int8_t);

        *((int8_t *)ptr) = robot_id; // id
        ptr += sizeof(int8_t);

        *((int8_t *)ptr) = 1; // 1 for cxy_tank ,2 for cxy_uav, 3 for cxy_ugv 
        ptr += sizeof(int8_t);

        *((int8_t *)ptr) = msg->moving_target;
        ptr += sizeof(int8_t);

        *((float *)ptr) = msg->enu_position[0]; //position
        ptr += sizeof(float);
        *((float *)ptr) = msg->enu_position[1];
        ptr += sizeof(float);
        *((float *)ptr) = msg->enu_position[2];
        ptr += sizeof(float);

        if(debug_mode)
        {
            printf("Send case2 result msg: head[%d] %s[%d] ",Send_Msg_Type::Case2ResultHead,robot_name.c_str(),robot_id%rest);
            printf("pos:[%.2f %.2f %.2f]\n",msg->enu_position[0],msg->enu_position[1],msg->enu_position[2]);
        }

        return ptr - udp_send_buf; // msg length
    }

    void case2_result_cb(const prometheus_msgs::Case2Result::ConstPtr& msg)
    {
        send_msg_by_udp(encode_case2_result_msg(msg),udp_ip);
    }

    int encode_case3_result_msg(const prometheus_msgs::Case3Result::ConstPtr& msg)
    {
        char *ptr = udp_send_buf;

        *((int8_t *)ptr) = Send_Msg_Type::Case3ResultHead; // msg head
        ptr += sizeof(int8_t);
        
        *((int8_t *)ptr) = robot_id; // id
        ptr += sizeof(int8_t);

        *((int8_t *)ptr) = msg->object_name; // 1 for cxy_tank ,2 for cxy_uav, 3 for cxy_ugv 
        ptr += sizeof(int8_t);

        *((int8_t *)ptr) = msg->score;
        ptr += sizeof(int8_t);

        if(debug_mode)
        {
            printf("Send case3 result msg: head[%d] %s[%d] ",Send_Msg_Type::Case3ResultHead,robot_name.c_str(),robot_id%rest);
            printf("object_name:[%d] score:[%d]\n",msg->object_name,msg->score);
        }

        return ptr - udp_send_buf; // msg length
    }

    void case3_result_cb(const prometheus_msgs::Case3Result::ConstPtr& msg)
    {
        send_msg_by_udp(encode_case3_result_msg(msg),udp_ip);
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

    void server_fun()
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
            else if(HEAD == Send_Msg_Type::DroneStateHead) decode_msg_1(valread);
            else if(HEAD == Rcv_Msg_Type::Msg102) decode_msg_102(valread);
            else if(HEAD == Rcv_Msg_Type::Msg103) decode_msg_103(valread);
            else if(HEAD == Rcv_Msg_Type::Msg104) decode_msg_104(valread);
            else if(HEAD == Rcv_Msg_Type::Msg31) decode_msg_31(valread);
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

        (this->*pub_msg_101)(); // publish msg 101
    }

    void pub_uav_msg_101()
    {
        uav_cmd_msg_101.Mode = prometheus_msgs::SwarmCommand::Move;
        uav_cmd_msg_101.Move_mode = prometheus_msgs::SwarmCommand::XYZ_POS;
        uav_cmd_msg_101.position_ref[0] = des_x;
        uav_cmd_msg_101.position_ref[1] = des_y;
        uav_cmd_msg_101.position_ref[2] = des_z;
        uav_cmd_msg_101.yaw_ref = 0.0;
        uav_cmd_pub.publish(uav_cmd_msg_101);
    }

    void pub_ugv_msg_101()
    {
        ugv_cmd_msg_101.Mode = prometheus_msgs::UgvCommand::Point_Control;
        ugv_cmd_msg_101.position_ref[0] = des_x;
        ugv_cmd_msg_101.position_ref[1] = des_y;
        ugv_cmd_msg_101.yaw_ref = 0.0;
        ugv_cmd_pub.publish(ugv_cmd_msg_101);
    }

    void decode_msg_102(int msg_len)
    {
        if(msg_len != Rcv_Msg_Length::Msg102Length)
        {
            ROS_ERROR("[msg_102]Wrong Msg Length!");
            return;
        }

        if(ugv_id != 0)
        {
            ROS_ERROR("[msg_102]Wrong uav cmd.");
            return;
        }

        char* ptr = tcp_recv_buf;

        ptr += sizeof(int8_t);

        msg_robot_id = *((int8_t*)ptr);
        ptr += sizeof(int8_t);


        if(msg_robot_id != uav_id)
        {
            ROS_ERROR("[msg_102]Wrong ROBOT_ID:%d",msg_robot_id);
            return;
        }

        Command = *((int8_t*)ptr);
        ptr += sizeof(int8_t);

        if(debug_mode) printf("Get msg_102: Command: [%d]\n ",Command);

        find_msg_102_cmd = msg_102_cmd_pair.find(Command);
        if(find_msg_102_cmd == msg_102_cmd_pair.end())
        {
            ROS_ERROR("[msg_102]wrong command: %d",Command);
            return;
        }
        uav_cmd_msg_102.Mode = find_msg_102_cmd->second.first;
        uav_cmd_msg_102.swarm_shape = find_msg_102_cmd->second.second;

        uav_cmd_pub.publish(uav_cmd_msg_102);  // publish msg 102
    }

    void decode_msg_103(int msg_len)
    {
        if(msg_len != Rcv_Msg_Length::Msg103Length)
        {
            ROS_ERROR("[msg_103]Wrong Msg Length!");
            return;
        }

        char* ptr = tcp_recv_buf;

        ptr += sizeof(int8_t);

        msg_robot_id = *((int8_t*)ptr);
        ptr += sizeof(int8_t);

        if(msg_robot_id != robot_id)
        {
            ROS_ERROR("[msg_103]Wrong ROBOT_ID:%d",msg_robot_id);
            return;
        }

        Command = *((int8_t*)ptr);
        ptr += sizeof(int8_t);

        des_x = *((float*)ptr);
        ptr += sizeof(float);

        des_y = *((float*)ptr);
        ptr += sizeof(float);

        des_z = *((float*)ptr);
        ptr += sizeof(float);

        if(debug_mode)
        {
            printf("Get msg_103: Command: [%d] ",Command);
            printf("des: [%.2f %.2f %.2f]\n",des_x,des_y,des_z);
        }

        (this->*pub_msg_103)(); // publish msg 103
    }

    void pub_uav_msg_103()
    {
        find_msg_103_uav_cmd = msg_103_uav_cmd_pair.find(Command);
        if(find_msg_103_uav_cmd == msg_103_uav_cmd_pair.end())
        {
            ROS_ERROR("[msg_103]Wrong Command: %d",Command);
            return;
        }
        msg_103.command = find_msg_103_uav_cmd->second;

        msg103_pub.publish(msg_103);
    }

    void pub_ugv_msg_103()
    {
        find_msg_103_ugv_cmd = msg_103_ugv_cmd_pair.find(Command);
        if(find_msg_103_ugv_cmd == msg_103_ugv_cmd_pair.end())
        {
            ROS_ERROR("[msg_103]Wrong Command: %d",Command);
            return;
        }
        msg_103.command = find_msg_103_ugv_cmd->second;

        msg_103.enu_position[0] = des_x;
        msg_103.enu_position[1] = des_y;
        msg_103.enu_position[2] = 0.0;

        msg103_pub.publish(msg_103);
    }

    void decode_msg_104(int msg_len)
    {
        if(msg_len != Rcv_Msg_Length::Msg104Length)
        {
            ROS_ERROR("[msg_104]Wrong Msg Length!");
            return;
        }

        char* ptr = tcp_recv_buf;

        ptr += sizeof(int8_t);

        msg_robot_id = *((int8_t*)ptr);
        ptr += sizeof(int8_t);

        if(msg_robot_id != robot_id)
        {
            ROS_ERROR("[msg_104]Wrong ROBOT_ID:%d",msg_robot_id);
            return;
        }

        Command = *((int8_t*)ptr);
        ptr += sizeof(int8_t);

        if(debug_mode) printf("Get msg_104: Command: [%d]\n",Command);

        find_msg_104_cmd = msg_104_cmd_pair.find(Command);
        if(find_msg_104_cmd == msg_104_cmd_pair.end())
        {
            ROS_ERROR("[msg_104]wrong command: %d",Command);
            return;
        }
        msg_104.command = msg_104_cmd_pair[Command];

        msg104_pub.publish(msg_104); // publish msg 104
    }

    void decode_msg_31(int msg_len)
    {
        if(msg_len != Rcv_Msg_Length::Msg31Length)
        {
            ROS_ERROR("[msg_31]Wrong Msg Length!");
            return;
        }

        char *ptr = tcp_recv_buf;
        
        ptr += sizeof(int8_t); // HEAD

        msg_robot_id = *((int8_t*)ptr); // ugv id
        ptr += sizeof(int8_t);

        if(msg_robot_id != other_ugv_id)
        {
            ROS_ERROR("[msg_31]Wrong ROBOT_ID:%d",msg_robot_id);
            return;
        }

        msg_31.pose.pose.position.x = *((float *)ptr);
        ptr += sizeof(float);
        msg_31.pose.pose.position.y = *((float *)ptr);
        ptr += sizeof(float);
        msg_31.pose.pose.position.z = *((float *)ptr);
        ptr += sizeof(float);

        if(debug_mode)
        {
            printf("Get msg_31: UGV:%d ",msg_robot_id);
            printf("[%f %f %f]\n",msg_31.pose.pose.position.x,msg_31.pose.pose.position.y,msg_31.pose.pose.position.z);
        }
        
        other_ugv_odom_pub.publish(msg_31);
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

private:
    // ros
    ros::Subscriber drone_state_sub, case2_result_sub, case3_result_sub;
    ros::Subscriber ugv_state_sub, ugv_odom_sub;
    ros::Publisher uav_cmd_pub, ugv_cmd_pub;
    ros::Publisher msg103_pub, msg104_pub;
    ros::Publisher other_ugv_odom_pub, uav_state_pub[9];

    // msg
    prometheus_msgs::SwarmCommand uav_cmd_msg_101;
    prometheus_msgs::UgvCommand ugv_cmd_msg_101;
    prometheus_msgs::SwarmCommand uav_cmd_msg_102;
    prometheus_msgs::Msg103 msg_103;
    prometheus_msgs::Msg104 msg_104;
    prometheus_msgs::DroneState msg_1;
    prometheus_msgs::DroneState to_drone_msg;
    nav_msgs::Odometry msg_31;
    int8_t msg_robot_id, Command, Mode;
    float des_x, des_y, des_z;
    
    // tcp/udp
    std::string udp_ip, tcp_ip, other_ugv_ip;
    struct sockaddr_in tcp_addr,udp_addr;
    int tcp_send_sock, udp_send_sock, server_fd, recv_sock, UDP_PORT, TCP_PORT;
    char udp_send_buf[BUF_LEN], udp_recv_buf[BUF_LEN], tcp_send_buf[BUF_LEN], tcp_recv_buf[BUF_LEN];

    // id
    std::string robot_name;
    int uav_id, ugv_id, robot_id, other_ugv_id;

    // others
    float formation_size;
    bool debug_mode;
    int case_flag, rest, drone_state_timeout, ugv_state_timeout;

    // function ptr
    void (multicast_bridge::*pub_msg_101)();
    void (multicast_bridge::*pub_msg_103)();

    // predefine msg cmd pair (using std::map)
    std::map<std::string,int8_t> uav_mode_pair = {  {"MANUAL",0},{"OFFBOARD",1},{"STABILIZED",2},{"ACRO",3},{"ALTCTL",4},{"POSCTL",5},
                                                    {"RATTITUDE",6},{"AUTO.MISSION",7},{"AUTO.LOITER",8},{"AUTO.RTL",9},{"AUTO.LAND",10},
                                                    {"AUTO.RTGS",11},{"AUTO.READY",12},{"AUTO.TAKEOFF",13}};
    std::map<int8_t,std::pair<int8_t,int8_t>> msg_102_cmd_pair = {{1,{prometheus_msgs::SwarmCommand::Takeoff,0}},
                        {2,{prometheus_msgs::SwarmCommand::Land,0}},{3,{prometheus_msgs::SwarmCommand::Hold,0}},
                        {4,{prometheus_msgs::SwarmCommand::Position_Control,prometheus_msgs::SwarmCommand::One_column}},
                        {5,{prometheus_msgs::SwarmCommand::Position_Control,prometheus_msgs::SwarmCommand::Triangle}},
                        {6,{prometheus_msgs::SwarmCommand::Position_Control,prometheus_msgs::SwarmCommand::Square}},
                        {7,{prometheus_msgs::SwarmCommand::Position_Control,prometheus_msgs::SwarmCommand::Circular}}};
    std::map<int8_t,std::pair<int8_t,int8_t>>::iterator find_msg_102_cmd;
    std::map<int8_t,int8_t> msg_103_uav_cmd_pair = {{1,1}, {2,2}, {3,3}};
    std::map<int8_t,int8_t>::iterator find_msg_103_uav_cmd;
    std::map<int8_t,int8_t> msg_103_ugv_cmd_pair = {{1,1}, {2,2}, {3,3}, {4,4}};
    std::map<int8_t,int8_t>::iterator find_msg_103_ugv_cmd;
    std::map<int8_t,int8_t> msg_104_cmd_pair = {{1,1}, {2,2}, {3,3}};
    std::map<int8_t,int8_t>::iterator find_msg_104_cmd;
    std::map<int8_t,std::string> uav_ip_pair = {{1,"192.168.1.11"},{2,"192.168.1.12"},{3,"192.168.1.13"},{4,"192.168.1.14"},
                                                {5,"192.168.1.15"},{6,"192.168.1.16"},{7,"192.168.1.17"},{8,"192.168.1.18"}};
};

int main(int argc, char **argv) 
{
    ros::init(argc,argv,"cxy_multicast_bridge");
    ros::NodeHandle nh("~");

    printf("\033[1;32m---->[cxy_multicast_bridge] start running\n\033[0m");

    multicast_bridge mb(nh);

    ros::spin();

    return 0;
}