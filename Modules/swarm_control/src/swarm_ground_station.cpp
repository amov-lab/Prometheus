//头文件
#include <ros/ros.h>
#include <boost/format.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "swarm_ground_station.h"

using namespace std;

//主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_ground_station");
    ros::NodeHandle nh("~");

    nh.param<int>("swarm_num", swarm_num, 1);
    nh.param<bool>("flag_ros2groundstation", flag_ros2groundstation, false);

    for(int i = 1; i <= swarm_num; i++) 
    {
        // 设置无人机名字，none代表无
        boost::format fmt2("uav%d_id");
        nh.param<int>((fmt2%(i)).str(), uav_id[i], 0);
        if(uav_id[i] == 0)
        {
            continue;
        }
        uav_name[i] = "/uav" + std::to_string(uav_id[i]);
        // 订阅
        command_sub[i] = nh.subscribe<prometheus_msgs::SwarmCommand>(uav_name[i] + "/prometheus/swarm_command", 10, swarm_command_cb[i]);
        drone_state_sub[i] = nh.subscribe<prometheus_msgs::DroneState>(uav_name[i] + "/prometheus/drone_state", 10, drone_state_cb[i]);
    }
    
    boost::format fmt3("uav%d,%f,%f,%f,%f,%f,%f,%f,%f,%f");
    while(ros::ok())
    {
        ros::spinOnce();
        cout << ">>>>>>>>>>>>>>>>>>>> Swarm Ground Station <<<<<<<<<<<<<<<<<<< "<< endl;
        for(int i = 1; i <= swarm_num; i++)
        {
            if(uav_id[i] != 0)
            {
                printf_swarm_state(swarm_num, uav_id[i], uav_name[i], State_uav[i], Command_uav[i]);
            }
            if(flag_ros2groundstation)
            {
                //printf("send message to server: ");
                data = (fmt3%(i)%(State_uav[i].position[0])%(State_uav[i].position[1])%State_uav[i].position[2]%
                    (State_uav[i].velocity[0])%(State_uav[i].velocity[1])%(State_uav[i].velocity[2])%
                    (State_uav[i].attitude[0])%(State_uav[i].attitude[1])%(State_uav[i].attitude[2])).str();
                cout << data << endl;
                strcpy(sendline,data.c_str());
                //send by socket
                socketfd = socket(AF_INET,SOCK_STREAM,0);
                memset(&sockaddr,0,sizeof(sockaddr));
                sockaddr.sin_family = AF_INET;
                sockaddr.sin_port = htons(10004);
                inet_pton(AF_INET,servInetAddr,&sockaddr.sin_addr);
                if((connect(socketfd,(struct sockaddr*)&sockaddr,sizeof(sockaddr))) < 0 ) {
                    printf("connect error %s errno: %d\n",strerror(errno),errno);
                    printf("client connect failed!\n");
                }
                if((send(socketfd,sendline,strlen(sendline),0)) < 0)
                {
                    printf("send mes error: %s errno : %d\n",strerror(errno),errno);
                    printf("client send failed!\n");
                }
                close(socketfd);
            }
        }
        sleep(2.0); // frequence
    }
    return 0;
}