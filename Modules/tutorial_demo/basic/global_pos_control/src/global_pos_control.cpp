/******************************************************************************
*例程简介: 讲解如何调用uav_control的接口实现无人机的经纬度以及相对高度控制
*
*效果说明: 无人机起飞移动到目标经纬度以及相对高度位置点,悬停30秒后降落
*
*备注:该例程仅支持Prometheus仿真,真机测试需要熟练掌握相关接口的定义后以及真机适配修改后使用
******************************************************************************/

#include <ros/ros.h>
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVSetup.h>
#include <unistd.h>

//主函数
int main(int argc, char** argv)
{
    //ROS初始化,设定节点名
    ros::init(argc , argv, "global_pos_control");
    //创建句柄
    ros::NodeHandle n;
    //创建无人机控制命令发布者
    ros::Publisher uav_command_pub = n.advertise<prometheus_msgs::UAVCommand>("/uav1/prometheus/command", 10);
    //创建无人机设置命令发布者
    ros::Publisher uav_setup_pub = n.advertise<prometheus_msgs::UAVSetup>("/uav1/prometheus/setup", 10);
    //创建控制命令变量
    prometheus_msgs::UAVCommand uav_command;
    //创建设置命令变量
    prometheus_msgs::UAVSetup uav_setup;
    
    int start_flag;

    std::cout << "Please input 1 to start global position control demo" << std::endl;
    std::cin >> start_flag;

    if(start_flag != 1)
    {
        return 0;
    }

    /****************************起飞*******************************/
    //时间戳
    uav_command.header.stamp = ros::Time::now();
    //坐标系
    uav_command.header.frame_id = "ENU";
    //Init_Pos_Hover初始位置悬停,可在uav_control_indoor.yaml或uav_control_outdoor.yaml文件设置无人机悬停高度
    uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
    //发布的命令ID,每发一次,该ID加1
    uav_command.Command_ID = 1;
    //发布起飞命令
    uav_command_pub.publish(uav_command);

    //因数据传输存在延迟,等待1秒保证数据能够正常传输到飞控,才可以将无人机飞行模式切换为OFFBOARD模式并解锁
    sleep(1);

    //切换为OFFBOARD模式
    uav_setup.header.stamp = ros::Time::now();
    uav_setup.cmd = prometheus_msgs::UAVSetup::SET_PX4_MODE;
    uav_setup.px4_mode = "OFFBOARD";
    uav_setup_pub.publish(uav_setup);
    
    //等待1秒,确认无人机切换为OFFBOARD模式后解锁
    sleep(1);

    //无人机解锁
    uav_setup.header.stamp = ros::Time::now();
    uav_setup.cmd = prometheus_msgs::UAVSetup::ARMING;
    uav_setup.arming = true;
    uav_setup_pub.publish(uav_setup);

    ROS_INFO("UAV Takeoff");
    /****************************起飞*******************************/
    
    //等待5秒
    sleep(5);

    /****************************global_pos_control*******************************/
    //时间戳
    uav_command.header.stamp = ros::Time::now();
    //坐标系
    uav_command.header.frame_id = "WGS84";
    //Move模式
    uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
    //Move_mode
    uav_command.Move_mode = prometheus_msgs::UAVCommand::LAT_LON_ALT;
    //设置控制数据,需要注意此处经纬度为阿木实验室设置的默认经纬度
    //仿真需要设置PX4 HOME点环境变量
    //真机测试需注意,需将经纬度改为合适的值
    uav_command.latitude = 30.78526000;
    uav_command.longitude = 103.8610300;
    uav_command.altitude = 5.0;
    uav_command.yaw_ref = 0.0;
    //发布的命令ID加1
    uav_command.Command_ID += 1;
    //发布降落命令
    uav_command_pub.publish(uav_command);
    ROS_INFO("UAV Move");
    /****************************global_pos_control*******************************/

    //等待30秒
    sleep(30);

    /****************************降落*******************************/
    //时间戳
    uav_command.header.stamp = ros::Time::now();
    //坐标系
    uav_command.header.frame_id = "ENU";
    //Land降落,从当前位置降落至地面并自动上锁
    uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Land;
    //发布的命令ID加1
    uav_command.Command_ID += 1;
    //发布降落命令
    uav_command_pub.publish(uav_command);
    ROS_INFO("UAV Land");
    /****************************降落*******************************/

    return 0;

}