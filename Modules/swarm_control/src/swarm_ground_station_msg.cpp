//头文件
#include <ros/ros.h>
#include <boost/format.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "swarm_ground_station.h"

using namespace std;

void msg_cb(const prometheus_msgs::Message::ConstPtr& msg)
{
    prometheus_msgs::Message message = *msg;

    if(message.message_type == prometheus_msgs::Message::NORMAL)
    {
        cout << "[NORMAL] - " << "[ "<< message.source_node << " ]: " << message.content <<endl;
    }else if(message.message_type == prometheus_msgs::Message::WARN)
    {
        cout << "[ WARN ] - " << "[ "<< message.source_node << " ]: " <<message.content <<endl;
    }else if(message.message_type == prometheus_msgs::Message::ERROR)
    {
        cout << "[ERROR ] - " << "[ "<< message.source_node << " ]: " << message.content <<endl;
    }

    sleep(0.2);
}

//主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_ground_station_msg");
    ros::NodeHandle nh("~");

    nh.param<int>("swarm_num", swarm_num, 1);

    for(int i = 1; i <= swarm_num; i++) 
    {
        // 设置无人机名字，none代表无
        boost::format fmt1("uav%d_name");
        nh.param<string>((fmt1%(i)).str(), uav_name[i], "/none");
        boost::format fmt2("uav%d_id");
        nh.param<int>((fmt2%(i)).str(), uav_id[i], 0);
        // 订阅
        message_sub[i] = nh.subscribe<prometheus_msgs::Message>(uav_name[i] + "/prometheus/message/main", 100, msg_cb);
    }
    
    while(ros::ok())
    {
        ros::spinOnce();
        sleep(2.0); // frequence
    }
    return 0;
}