//头文件
#include <ros/ros.h>

#include "swarm_control_utils.h"
#include "message_utils.h"

using namespace std;
//---------------------------------------相关参数-----------------------------------------------
int swarm_num;
string uav1_name,uav2_name,uav3_name,uav4_name,uav5_name;
int uav1_id,uav2_id,uav3_id,uav4_id,uav5_id;

prometheus_msgs::DroneState State_uav1;
prometheus_msgs::DroneState State_uav2;
prometheus_msgs::DroneState State_uav3;
prometheus_msgs::DroneState State_uav4;
prometheus_msgs::DroneState State_uav5;

prometheus_msgs::SwarmCommand Command_uav1;  
prometheus_msgs::SwarmCommand Command_uav2;  
prometheus_msgs::SwarmCommand Command_uav3;  
prometheus_msgs::SwarmCommand Command_uav4;  
prometheus_msgs::SwarmCommand Command_uav5;  

geometry_msgs::PoseStamped ref_pose_uav1;
geometry_msgs::PoseStamped ref_pose_uav2;
geometry_msgs::PoseStamped ref_pose_uav3;
geometry_msgs::PoseStamped ref_pose_uav4;
geometry_msgs::PoseStamped ref_pose_uav5;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void swarm_command_cb_1(const prometheus_msgs::SwarmCommand::ConstPtr& msg)
{
    Command_uav1 = *msg;
}
void swarm_command_cb_2(const prometheus_msgs::SwarmCommand::ConstPtr& msg)
{
    Command_uav2 = *msg;
}
void swarm_command_cb_3(const prometheus_msgs::SwarmCommand::ConstPtr& msg)
{
    Command_uav3 = *msg;
}
void swarm_command_cb_4(const prometheus_msgs::SwarmCommand::ConstPtr& msg)
{
    Command_uav4 = *msg;
}
void swarm_command_cb_5(const prometheus_msgs::SwarmCommand::ConstPtr& msg)
{
    Command_uav5 = *msg;
}

void drone_state_cb1(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    State_uav1 = *msg;
}
void drone_state_cb2(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    State_uav2 = *msg;
}
void drone_state_cb3(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    State_uav3 = *msg;
}
void drone_state_cb4(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    State_uav4 = *msg;
}
void drone_state_cb5(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    State_uav5 = *msg;
}

void msg_cb(const prometheus_msgs::Message::ConstPtr& msg)
{
    prometheus_msgs::Message message = *msg;
    printf_message(message);
    sleep(0.2);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    //　目前最多支持５架飞机，可根据实际情况增加
    ros::init(argc, argv, "swarm_ground_station");
    ros::NodeHandle nh("~");

    nh.param<int>("swarm_num", swarm_num, 1);
    //　设置无人机名字，none代表无
    nh.param<string>("uav1_name", uav1_name, "/none");
    nh.param<string>("uav2_name", uav2_name, "/none");
    nh.param<string>("uav3_name", uav3_name, "/none");
    nh.param<string>("uav4_name", uav4_name, "/none");
    nh.param<string>("uav5_name", uav5_name, "/none");
    nh.param<int>("uav1_id", uav1_id, 0);
    nh.param<int>("uav2_id", uav2_id, 0);
    nh.param<int>("uav3_id", uav3_id, 0);
    nh.param<int>("uav4_id", uav4_id, 0);
    nh.param<int>("uav5_id", uav5_id, 0);


    ros::Subscriber command_sub1 = nh.subscribe<prometheus_msgs::SwarmCommand>(uav1_name + "/prometheus/swarm_command", 10, swarm_command_cb_1);
    ros::Subscriber drone_state_sub1 = nh.subscribe<prometheus_msgs::DroneState>(uav1_name + "/prometheus/drone_state", 10, drone_state_cb1);
    ros::Subscriber message_sub1 = nh.subscribe<prometheus_msgs::Message>(uav1_name + "/prometheus/message/main", 100, msg_cb);

    ros::Subscriber command_sub2 = nh.subscribe<prometheus_msgs::SwarmCommand>(uav2_name + "/prometheus/swarm_command", 10, swarm_command_cb_2);
    ros::Subscriber drone_state_sub2 = nh.subscribe<prometheus_msgs::DroneState>(uav2_name + "/prometheus/drone_state", 10, drone_state_cb2);
    ros::Subscriber message_sub2 = nh.subscribe<prometheus_msgs::Message>(uav2_name + "/prometheus/message/main", 100, msg_cb);

    ros::Subscriber command_sub3 = nh.subscribe<prometheus_msgs::SwarmCommand>(uav3_name + "/prometheus/swarm_command", 10, swarm_command_cb_3);
    ros::Subscriber drone_state_sub3 = nh.subscribe<prometheus_msgs::DroneState>(uav3_name + "/prometheus/drone_state", 10, drone_state_cb3);
    ros::Subscriber message_sub3 = nh.subscribe<prometheus_msgs::Message>(uav3_name + "/prometheus/message/main", 100, msg_cb);

    ros::Subscriber command_sub4 = nh.subscribe<prometheus_msgs::SwarmCommand>(uav4_name + "/prometheus/swarm_command", 10, swarm_command_cb_4);
    ros::Subscriber drone_state_sub4 = nh.subscribe<prometheus_msgs::DroneState>(uav4_name + "/prometheus/drone_state", 10, drone_state_cb4);
    ros::Subscriber message_sub4 = nh.subscribe<prometheus_msgs::Message>(uav4_name + "/prometheus/message/main", 10, msg_cb);

    ros::Subscriber command_sub5 = nh.subscribe<prometheus_msgs::SwarmCommand>(uav5_name + "/prometheus/swarm_command", 10, swarm_command_cb_5);
    ros::Subscriber drone_state_sub5 = nh.subscribe<prometheus_msgs::DroneState>(uav5_name + "/prometheus/drone_state", 10, drone_state_cb5);
    ros::Subscriber message_sub5 = nh.subscribe<prometheus_msgs::Message>(uav5_name + "/prometheus/message/main", 100, msg_cb);



//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();

        //打印
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>> Formation Flight Station <<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        if(uav1_id != 0 )
        {
            swarm_control_utils::printf_swarm_state(swarm_num, uav1_id, uav1_name, State_uav1, Command_uav1);
        }
        if(uav2_id != 0)
        {
            swarm_control_utils::printf_swarm_state(swarm_num, uav2_id, uav2_name, State_uav2, Command_uav2);
        }
        if(uav3_id != 0 )
        {
            swarm_control_utils::printf_swarm_state(swarm_num, uav3_id, uav3_name, State_uav3, Command_uav3);
        }
        if(uav4_id != 0 )
        {
            swarm_control_utils::printf_swarm_state(swarm_num, uav4_id, uav4_name, State_uav4, Command_uav4);
        }
        if(uav5_id != 0 )
        {
            swarm_control_utils::printf_swarm_state(swarm_num, uav5_id, uav5_name, State_uav5, Command_uav5);
        }

        sleep(2.0);
    }

    return 0;

}
