/******************************************************************************
*例程简介: 无人机集群控制,包含无人机集群的模式控制/位置控制/队形变换/一字队形/三角队形
*
*效果说明: -
*
*备注:该例程仅支持Prometheus仿真,真机测试需要熟练掌握相关接口的定义后以及真机适配修改后使用
******************************************************************************/

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/UAVSetup.h>
#include <prometheus_msgs/GPSData.h>
#include <chrono>
#include <thread>

prometheus_msgs::GPSData origin_gps;

void uav1_state_cb(const prometheus_msgs::UAVState::ConstPtr &msg)
{
    origin_gps.latitude  = msg->latitude;
    origin_gps.longitude = msg->longitude;
    origin_gps.altitude  = msg->altitude;
    origin_gps.x = msg->position[0];
    origin_gps.y = msg->position[1];
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_pose_offset");
    ros::NodeHandle n("~");
    int agent_num;
    n.param<int>("agent_num", agent_num, 3);


    ros::Publisher uav_command_pub[agent_num];
    ros::Publisher set_local_pose_offset_pub[agent_num];
    ros::Subscriber uav1_state_sub = n.subscribe<prometheus_msgs::UAVState>("/uav1/prometheus/state", 1, uav1_state_cb);

    for(int i=1; i<agent_num; i++)
    {
        set_local_pose_offset_pub[i+1] = n.advertise<prometheus_msgs::GPSData>("/uav" + std::to_string(i+1) + "/prometheus/set_local_offset_pose", 10);
    }

    // //此处做一个阻塞确保无人机全部正常启动后,能够将无人机原点坐标系统一到1号无人机所在位置
    std::this_thread::sleep_for(std::chrono::seconds(3));
    ros::spinOnce();
    for(int i=1; i<agent_num; i++)
    {
        set_local_pose_offset_pub[i+1].publish(origin_gps);
    }
    ros::spin();
}