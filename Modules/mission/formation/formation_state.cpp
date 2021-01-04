/*******************************************************************
 * 文件名:formation_control.cpp
 * 
 * 作者: BOSHEN97
 * 
 * 更新时间: 2020.10.14
 * 
 * 介绍:该cpp文件主要为动捕集群中集群控制相关函数的实现以及程序的运行
 * ****************************************************************/
#include "Formation.h"
#include <unistd.h>

void formation::init()
{
    //集群五台飞机状态订阅回调函数
    uav1_state_sub = n.subscribe("/uav1/mavros/state", 10, &formation::Uav1StateCallBack, this);
    uav2_state_sub = n.subscribe("/uav2/mavros/state", 10, &formation::Uav2StateCallBack, this);
    uav3_state_sub = n.subscribe("/uav3/mavros/state", 10, &formation::Uav3StateCallBack, this);
    uav4_state_sub = n.subscribe("/uav4/mavros/state", 10, &formation::Uav4StateCallBack, this);
    uav5_state_sub = n.subscribe("/uav5/mavros/state", 10, &formation::Uav5StateCallBack, this);

    uav1_pose_sub = n.subscribe("/uav1/mavros/local_position/pose", 10, &formation::Uav1PoseCallBack, this);
    uav2_pose_sub = n.subscribe("/uav2/mavros/local_position/pose", 10, &formation::Uav2PoseCallBack, this);
    uav3_pose_sub = n.subscribe("/uav3/mavros/local_position/pose", 10, &formation::Uav3PoseCallBack, this);
    uav4_pose_sub = n.subscribe("/uav4/mavros/local_position/pose", 10, &formation::Uav4PoseCallBack, this);
    uav5_pose_sub = n.subscribe("/uav5/mavros/local_position/pose", 10, &formation::Uav5PoseCallBack, this);

    //设置程序初始时间
    begin_time = ros::Time::now();
}

//获取1号机位置数据回调函数
void formation::Uav1PoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs)
{
    uav1_current_pose = *pose_msgs;
}

//获取2号机位置数据回调函数
void formation::Uav2PoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs)
{
    uav2_current_pose = *pose_msgs;
}

//获取3号机位置数据回调函数
void formation::Uav3PoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs)
{
    uav3_current_pose = *pose_msgs;
}

//获取4号机位置数据回调函数
void formation::Uav4PoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs)
{
    uav4_current_pose = *pose_msgs;
}

//获取5号机位置数据回调函数
void formation::Uav5PoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs)
{
    uav5_current_pose = *pose_msgs;
}

//打印无人机状态函数
void formation::printf_formation_state()
{
    //固定的浮点显示
    std::cout.setf(std::ios::fixed);
    //设置小数点后精度为2位
    std::cout<<std::setprecision(2);
    //左对齐
    std::cout.setf(std::ios::left);
    //显示小数点
    std::cout.setf(std::ios::showpoint);
    //强制显示符号
    std::cout.setf(std::ios::showpos);
    //打印时间戳
    std::cout << "Time :  " << ros::Time::now().sec - begin_time.sec << std::endl;
    //1号机
    std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV1 State<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    //是否与飞控连接
    if(uav1_state.connected == true)
    {
        std::cout << "  [Connected]  ";
    }
    else
    {
        std::cout << "  [Unconnected]  ";
    }
    //是否上锁
    if(uav1_state.armed == true)
    {
        std::cout << "  [ Armed ]  ";
    }
    else
    {
        std::cout << "  [ DisArmed ]  ";
    }

    //1号机当前模式
    std::cout << " [ " << uav1_state.mode << " ] "  << std::endl;
    //1号机当前位置
    std::cout << "Position_uav1 [X Y Z]: " << uav1_current_pose.pose.position.x << "[ m ]" << uav1_current_pose.pose.position.y << "[ m ]" << uav1_current_pose.pose.position.z << "[ m ]" << std::endl;

    //2号机
    std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV2 State<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    //是否与飞控连接
    if(uav2_state.connected == true)
    {
        std::cout << "  [Connected]  ";
    }
    else
    {
        std::cout << "  [Unconnected]  ";
    }
    //是否上锁
    if(uav2_state.armed == true)
    {
        std::cout << "  [ Armed ]  ";
    }
    else
    {
        std::cout << "  [ DisArmed ]  ";
    }

    //2号机当前模式
    std::cout << " [ " << uav2_state.mode << " ] "  << std::endl;
    //2号机当前位置
    std::cout << "Position_uav2 [X Y Z]: " << uav2_current_pose.pose.position.x << "[ m ]" << uav2_current_pose.pose.position.y << "[ m ]" << uav2_current_pose.pose.position.z << "[ m ]" << std::endl;

    //3号机
    std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV3 State<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    //是否与飞控连接
    if(uav3_state.connected == true)
    {
        std::cout << "  [Connected]  ";
    }
    else
    {
        std::cout << "  [Unconnected]  ";
    }
    //是否上锁
    if(uav3_state.armed == true)
    {
        std::cout << "  [ Armed ]  ";
    }
    else
    {
        std::cout << "  [ DisArmed ]  ";
    }

    //3号机当前模式
    std::cout << " [ " << uav3_state.mode << " ] "  << std::endl;
    //3号机当前位置
    std::cout << "Position_uav3 [X Y Z]: " << uav3_current_pose.pose.position.x << "[ m ]" << uav3_current_pose.pose.position.y << "[ m ]" << uav3_current_pose.pose.position.z << "[ m ]" << std::endl;

    //4号机
    std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV4 State<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    //是否与飞控连接
    if(uav4_state.connected == true)
    {
        std::cout << "  [Connected]  ";
    }
    else
    {
        std::cout << "  [Unconnected]  ";
    }
    //是否上锁
    if(uav4_state.armed == true)
    {
        std::cout << "  [ Armed ]  ";
    }
    else
    {
        std::cout << "  [ DisArmed ]  ";
    }

    //4号机当前模式
    std::cout << " [ " << uav4_state.mode << " ] "  << std::endl;
    //4号机当前位置
    std::cout << "Position_uav4 [X Y Z]: " << uav4_current_pose.pose.position.x << "[ m ]" << uav4_current_pose.pose.position.y << "[ m ]" << uav4_current_pose.pose.position.z << "[ m ]" << std::endl;

    //5号机
    std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV5 State<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    //是否与飞控连接
    if(uav5_state.connected == true)
    {
        std::cout << "  [Connected]  ";
    }
    else
    {
        std::cout << "  [Unconnected]  ";
    }
    //是否上锁
    if(uav5_state.armed == true)
    {
        std::cout << "  [ Armed ]  ";
    }
    else
    {
        std::cout << "  [ DisArmed ]  ";
    }

    //5号机当前模式
    std::cout << " [ " << uav5_state.mode << " ] "  << std::endl;
    //5号机当前位置
    std::cout << "Position_uav5 [X Y Z]: " << uav5_current_pose.pose.position.x << "[ m ]" << uav5_current_pose.pose.position.y << "[ m ]" << uav5_current_pose.pose.position.z << "[ m ]" << std::endl;
}

//集群控制函数
void formation::state()
{
    //初始化
    init();
    while(ros::ok())
    {
        //处理回调函数
        ros::spinOnce();
        //打印无人机相关信息
        printf_formation_state();
        //等待0.05秒
        usleep(50000);
    }
    
}

//集群五台无人机状态读取

//1号机
void formation::Uav1StateCallBack(const mavros_msgs::StateConstPtr &state_msgs)
{
    uav1_state = *state_msgs;
}

//2号机
void formation::Uav2StateCallBack(const mavros_msgs::StateConstPtr &state_msgs)
{
    uav2_state = *state_msgs;
}

//3号机
void formation::Uav3StateCallBack(const mavros_msgs::StateConstPtr &state_msgs)
{
    uav3_state = *state_msgs;
}

//4号机
void formation::Uav4StateCallBack(const mavros_msgs::StateConstPtr &state_msgs)
{
    uav4_state = *state_msgs;
}

//5号机
void formation::Uav5StateCallBack(const mavros_msgs::StateConstPtr &state_msgs)
{
    uav5_state = *state_msgs;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "formation_state");
    formation Formation;
    Formation.state();
    return 0;
}