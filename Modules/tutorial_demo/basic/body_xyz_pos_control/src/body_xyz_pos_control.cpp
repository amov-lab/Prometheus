/******************************************************************************
*例程简介: 讲解如何调用uav_control的接口实现无人机ENU坐标系下的位置控制
*
*效果说明: 无人机首先起飞移动到第一个目标位置点,然后移动到第二个目标位置,悬停30秒后降落
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
    ros::init(argc , argv, "body_xyz_pos_control");
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

    std::cout << "Please input 1 to start BODY XYZ_POS control demo" << std::endl;
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

    /****************************body_xyz_pos_control*******************************/
    //时间戳
    uav_command.header.stamp = ros::Time::now();
    //坐标系
    uav_command.header.frame_id = "BODY";
    //Move模式
    uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
    //Move_mode
    uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS_BODY;
    //无人机将会以当前位置移动
    uav_command.position_ref[0] = 1.0;
    uav_command.position_ref[1] = 0.0;
    uav_command.position_ref[2] = 1.0;
    uav_command.yaw_ref = 0.0;
    //发布的命令ID加1
    uav_command.Command_ID += 1;
    //发布降落命令
    uav_command_pub.publish(uav_command);
    ROS_INFO("UAV Move");

    //等待5秒,让无人机移动到目标位置
    sleep(5);

    //我们再次发布该命令,在ENU坐标系下发布相同控制命令,无人机目标位置是不变的
    //而在机体坐标系下坐标位置原点为无人机所在位置
    //因此无人机移动后发布相同指令目标位置也会有区别
    uav_command.position_ref[0] = 1.0;
    uav_command.position_ref[1] = 0.0;
    uav_command.position_ref[2] = 1.0;
    uav_command.yaw_ref = 0.0;
    //发布的命令ID加1
    uav_command.Command_ID += 1;
    //发布降落命令
    uav_command_pub.publish(uav_command);
    ROS_INFO("UAV Move");

    /****************************body_xyz_pos_control*******************************/

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