//头文件
#include <ros/ros.h>

#include <ugv_ground_station.h>

using namespace std;

//主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_ground_station");
    ros::NodeHandle nh("~");

    nh.param<int>("swarm_num", swarm_num, 1);

    for(int i = 1; i <= swarm_num; i++) 
    {
        boost::format fmt2("ugv%d_id");
        nh.param<int>((fmt2%(i)).str(), ugv_id[i], 0);
        ugv_name[i] = "/ugv" + std::to_string(ugv_id[i]);
        // 订阅
        command_sub[i] = nh.subscribe<prometheus_msgs::UgvCommand>(ugv_name[i] + "/prometheus/ugv_command", 10, ugv_command_cb[i]);
        ugv_state_sub[i] = nh.subscribe<prometheus_msgs::UgvState>(ugv_name[i] + "/prometheus/ugv_state", 10, ugv_state_cb[i]);
    }

    // 不使用来自飞控的位置信息
    // 【订阅】无人机当前位置 坐标系:ENU系 （此处注意，所有状态量在飞控中均为NED系，但在ros中mavros将其转换为ENU系处理。所以，在ROS中，所有和mavros交互的量都为ENU系）
    //  本话题来自飞控(通过Mavros功能包 /plugins/local_position.cpp读取), 对应Mavlink消息为LOCAL_POSITION_NED (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>(ugv_name[1] + "/mavros/local_position/pose", 1, pos_cb);

    // 疑问：为什么只有/mavros/global_position/local才和vision的数据重合？
    ros::Subscriber global_position_sub = nh.subscribe<nav_msgs::Odometry>(ugv_name[1] + "/mavros/global_position/local", 1, pos_cb2);

    // 【订阅】无人机当前速度 坐标系:ENU系
    //  本话题来自飞控(通过Mavros功能包 /plugins/local_position.cpp读取), 对应Mavlink消息为LOCAL_POSITION_NED (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
    ros::Subscriber local_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>(ugv_name[1] + "/mavros/local_position/velocity_local", 1, vel_cb);

    // 【订阅】无人机当前欧拉角 坐标系:ENU系
    //  本话题来自飞控(通过Mavros功能包 /plugins/imu.cpp读取), 对应Mavlink消息为ATTITUDE (#30), 对应的飞控中的uORB消息为vehicle_attitude.msg
    ros::Subscriber attitude_sub = nh.subscribe<sensor_msgs::Imu>(ugv_name[1] + "/mavros/imu/data", 1, att_cb);


    boost::format fmt3("ugv%d,%f,%f,%f,%f,%f,%f,%f,%f,%f");
    while(ros::ok())
    {
        ros::spinOnce();
        cout << ">>>>>>>>>>>>>>>>>>>> CXY Ugv Ground Station <<<<<<<<<<<<<<<<<<< "<< endl;
        for(int i = 1; i <= swarm_num; i++)
        {
            if(ugv_id[i] != 0)
            {
                printf_ugv_state(swarm_num, ugv_id[i], ugv_name[i], State_ugv[i], Command_ugv[i]);
            }
        }

        printf_test_state();
        sleep(2.0); // frequence
    }
    return 0;
}