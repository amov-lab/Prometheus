/*******************************************************************
 * 文件名:formation_setmode.cpp
 * 
 * 作者: BOSHEN97
 * 
 * 更新时间: 2020.10.14
 * 
 * 介绍:该cpp文件主要为动捕集群中模式切换相关函数的实现以及程序的运行
 * ****************************************************************/
#include "Formation.h"
#include <unistd.h>
#include <iostream>

void formation::init()
{
    //获取相关参数
    ros::param::param<int>("~OFFBOARD_intervals", offboard_intervals, 3);
    ros::param::param<int>("~LAND_intervals", land_intervals, 3);
    ros::param::param<string>("Flight_controller", flight_controller, "spm");
    ros::param::param<double>("~Takeoff_height", takeoff_height, 1.0);
}

void formation::is_wait(int time)
{
    //判断是否需要等待,time变量不为0,则等待
    if(time != 0)
    {
        sleep(time);
    }
}

void formation::set_formation_px4_offboard()
{
    //创建1~5号机的command_to_mavros类
    command_to_mavros ctm1("uav1");

    command_to_mavros ctm2("uav2");

    command_to_mavros ctm3("uav3");

    command_to_mavros ctm4("uav4");

    command_to_mavros ctm5("uav5");
    //设置1~5号机模式变量为offboard,解上锁变量为解锁
    ctm1.arm_cmd.request.value = true;
    ctm2.arm_cmd.request.value = true;
    ctm3.arm_cmd.request.value = true;
    ctm4.arm_cmd.request.value = true;
    ctm5.arm_cmd.request.value = true;

    ctm1.mode_cmd.request.custom_mode = "OFFBOARD";
    ctm2.mode_cmd.request.custom_mode = "OFFBOARD";
    ctm3.mode_cmd.request.custom_mode = "OFFBOARD";
    ctm4.mode_cmd.request.custom_mode = "OFFBOARD";
    ctm5.mode_cmd.request.custom_mode = "OFFBOARD";

    //集群按照23145的顺序对五台无人机分别解锁并切入offboard模式
    //当有一台无人机解锁或者切入offboard模式失败,该函数返回false
    //五台无人机成功解锁并切入offboard模式,该函数返回true
    if(ctm2.arming_client.call(ctm2.arm_cmd) && ctm2.set_mode_client.call(ctm2.mode_cmd))
    {
        ROS_INFO("uav2 armed and set offboard mode success");
        is_wait(offboard_intervals);
    }
    else
    {
        ROS_ERROR("uav2 armed and set offboard mode failed");
    }

    if(ctm3.arming_client.call(ctm3.arm_cmd) && ctm3.set_mode_client.call(ctm3.mode_cmd))
    {
        ROS_INFO("uav3 armed and set offboard mode success");
        is_wait(offboard_intervals);
    }
    else
    {
        ROS_ERROR("uav3 armed and set offboard mode failed");
    }

    if(ctm1.arming_client.call(ctm1.arm_cmd) && ctm1.set_mode_client.call(ctm1.mode_cmd))
    {
        ROS_INFO("uav1 armed and set offboard mode success");
        is_wait(offboard_intervals);
    }
    else
    {
        ROS_ERROR("uav1 armed and set offboard mode failed");
    }

    if(ctm4.arming_client.call(ctm4.arm_cmd) && ctm4.set_mode_client.call(ctm4.mode_cmd))
    {
        ROS_INFO("uav4 armed and set offboard mode success");
        is_wait(offboard_intervals);
    }
    else
    {
        ROS_ERROR("uav4 armed and set offboard mode failed");
    }

    if(ctm5.arming_client.call(ctm5.arm_cmd) && ctm5.set_mode_client.call(ctm5.mode_cmd))
    {
        ROS_INFO("uav5 armed and set offboard mode success");
    }
    else
    {
        ROS_ERROR("uav5 armed and set offboard mode failed");
    }
}

void formation::set_formation_apm_guided()
{
    //创建1~5号机的command_to_mavros类
    command_to_mavros ctm1("uav1");

    command_to_mavros ctm2("uav2");

    command_to_mavros ctm3("uav3");

    command_to_mavros ctm4("uav4");

    command_to_mavros ctm5("uav5");

    //设置1~5号机模式变量为offboard,解上锁变量为解锁以及设置起飞变量高度
    ctm1.arm_cmd.request.value = true;
    ctm2.arm_cmd.request.value = true;
    ctm3.arm_cmd.request.value = true;
    ctm4.arm_cmd.request.value = true;
    ctm5.arm_cmd.request.value = true;

    ctm1.mode_cmd.request.custom_mode = "GUIDED";
    ctm2.mode_cmd.request.custom_mode = "GUIDED";
    ctm3.mode_cmd.request.custom_mode = "GUIDED";
    ctm4.mode_cmd.request.custom_mode = "GUIDED";
    ctm5.mode_cmd.request.custom_mode = "GUIDED";

    uav1_takeoff_cmd.request.altitude = takeoff_height;
    uav2_takeoff_cmd.request.altitude = takeoff_height;
    uav3_takeoff_cmd.request.altitude = takeoff_height;
    uav4_takeoff_cmd.request.altitude = takeoff_height;
    uav5_takeoff_cmd.request.altitude = takeoff_height;

    if(ctm2.arming_client.call(ctm2.arm_cmd) && ctm2.set_mode_client.call(ctm2.mode_cmd))
    {
        ROS_INFO("uav2 armed and set guided mode success");
        sleep(1);
        if(uav2_takeoff_client.call(uav2_takeoff_cmd))
        {
            ROS_INFO("uav2 takeoff success");
            is_wait(offboard_intervals);
        }
        else
        {
            ROS_ERROR("uav2 takeoff failed");
        }
    }
    else
    {
        ROS_ERROR("uav2 armed and set guided mode failed");
    } 

    if(ctm3.arming_client.call(ctm3.arm_cmd) && ctm3.set_mode_client.call(ctm3.mode_cmd))
    {
        ROS_INFO("uav3 armed and set guided mode success");
        sleep(1);
        if(uav3_takeoff_client.call(uav3_takeoff_cmd))
        {
            ROS_INFO("uav3 takeoff success");
            is_wait(offboard_intervals);
        }
        else
        {
            ROS_ERROR("uav3 takeoff failed");
        }
    }
    else
    {
        ROS_ERROR("uav3 armed and set guided mode failed");
    }

    if(ctm1.arming_client.call(ctm1.arm_cmd) && ctm1.set_mode_client.call(ctm1.mode_cmd))
    {
        ROS_INFO("uav1 armed and set guided mode success");
        sleep(1);
        if(uav1_takeoff_client.call(uav1_takeoff_cmd))
        {
            ROS_INFO("uav1 takeoff success");
            is_wait(offboard_intervals);
        }
        else
        {
            ROS_ERROR("uav1 takeoff failed");
        }
    }
    else
    {
        ROS_ERROR("uav3 armed and set guided mode failed");
    }

    if(ctm4.arming_client.call(ctm4.arm_cmd) && ctm4.set_mode_client.call(ctm4.mode_cmd))
    {
        ROS_INFO("uav4 armed and set guided mode success");
        sleep(1);
        if(uav4_takeoff_client.call(uav4_takeoff_cmd))
        {
            ROS_INFO("uav4 takeoff success");
            is_wait(offboard_intervals);
        }
        else
        {
            ROS_ERROR("uav4 takeoff failed");
        }
    }
    else
    {
        ROS_ERROR("uav4 armed and set guided mode failed");
    }

    if(ctm5.arming_client.call(ctm5.arm_cmd) && ctm5.set_mode_client.call(ctm5.mode_cmd))
    {
        ROS_INFO("uav5 armed and set guided mode success");
        sleep(1);
        if(uav5_takeoff_client.call(uav5_takeoff_cmd))
        {
            ROS_INFO("uav5 takeoff success");
            is_wait(offboard_intervals);
        }
        else
        {
            ROS_ERROR("uav5 takeoff failed");
        }
    }
    else
    {
        ROS_ERROR("uav5 armed and set guided mode failed");
    }

}

void formation::set_formation_px4_land()
{
    //创建1~5号机的command_to_mavros类
    command_to_mavros ctm1("uav1");

    command_to_mavros ctm2("uav2");

    command_to_mavros ctm3("uav3");

    command_to_mavros ctm4("uav4");

    command_to_mavros ctm5("uav5");

    //设置1~5号机模式变量为land
    ctm1.mode_cmd.request.custom_mode = "AUTO.LAND";
    ctm2.mode_cmd.request.custom_mode = "AUTO.LAND";
    ctm3.mode_cmd.request.custom_mode = "AUTO.LAND";
    ctm4.mode_cmd.request.custom_mode = "AUTO.LAND";
    ctm5.mode_cmd.request.custom_mode = "AUTO.LAND";

    //切换为land模式,并对结果进行打印
    if(ctm2.set_mode_client.call(ctm2.mode_cmd))
    {
        ROS_INFO("uav2 set land mode success");
        is_wait(land_intervals);
    }
    else
    {
        ROS_ERROR("uav2 set land mode failed");
    }

    if(ctm3.set_mode_client.call(ctm3.mode_cmd))
    {
        ROS_INFO("uav3 set land mode success");
        is_wait(land_intervals);
    }
    else
    {
        ROS_ERROR("uav3 set land mode failed");
    }
    
    if(ctm1.set_mode_client.call(ctm1.mode_cmd))
    {
        ROS_INFO("uav1 set land mode success");
        is_wait(land_intervals);
    }
    else
    {
        ROS_ERROR("uav1 set land mode failed");
    }

    if(ctm4.set_mode_client.call(ctm4.mode_cmd))
    {
        ROS_INFO("uav4 set land mode success");
        is_wait(land_intervals);
    }
    else
    {
        ROS_ERROR("uav4 set land mode success");
    }

    if(ctm5.set_mode_client.call(ctm5.mode_cmd))
    {
        ROS_INFO("uav5 set land mode success");
    }
    else
    {
        ROS_ERROR("uav5 set land mode failed");
    }
}

void formation::set_formation_apm_land()
{
    //创建1~5号机的command_to_mavros类
    command_to_mavros ctm1("uav1");

    command_to_mavros ctm2("uav2");

    command_to_mavros ctm3("uav3");

    command_to_mavros ctm4("uav4");

    command_to_mavros ctm5("uav5");

    //设置1~5号机模式变量为land
    ctm1.mode_cmd.request.custom_mode = "LAND";
    ctm2.mode_cmd.request.custom_mode = "LAND";
    ctm3.mode_cmd.request.custom_mode = "LAND";
    ctm4.mode_cmd.request.custom_mode = "LAND";
    ctm5.mode_cmd.request.custom_mode = "LAND";

    //切换为land模式,并对结果进行打印
    if(ctm2.set_mode_client.call(ctm2.mode_cmd))
    {
        ROS_INFO("uav2 set land mode success");
        is_wait(land_intervals);
    }
    else
    {
        ROS_ERROR("uav2 set land mode failed");
    }

    if(ctm3.set_mode_client.call(ctm3.mode_cmd))
    {
        ROS_INFO("uav3 set land mode success");
        is_wait(land_intervals);
    }
    else
    {
        ROS_ERROR("uav3 set land mode failed");
    }
    
    if(ctm1.set_mode_client.call(ctm1.mode_cmd))
    {
        ROS_INFO("uav1 set land mode success");
        is_wait(land_intervals);
    }
    else
    {
        ROS_ERROR("uav1 set land mode failed");
    }

    if(ctm4.set_mode_client.call(ctm4.mode_cmd))
    {
        ROS_INFO("uav4 set land mode success");
        is_wait(land_intervals);
    }
    else
    {
        ROS_ERROR("uav4 set land mode success");
    }

    if(ctm5.set_mode_client.call(ctm5.mode_cmd))
    {
        ROS_INFO("uav5 set land mode success");
    }
    else
    {
        ROS_ERROR("uav5 set land mode failed");
    }
}

void formation::set_formation_disarmed()
{
    //创建1~5号机的command_to_mavros类
    command_to_mavros ctm1("uav1");

    command_to_mavros ctm2("uav2");

    command_to_mavros ctm3("uav3");

    command_to_mavros ctm4("uav4");

    command_to_mavros ctm5("uav5");

    //设置1~5号机解上锁变量为false
    ctm1.arm_cmd.request.value = false;
    ctm1.arm_cmd.request.value = false;
    ctm1.arm_cmd.request.value = false;
    ctm1.arm_cmd.request.value = false;
    ctm1.arm_cmd.request.value = false;

    //按照23145的顺序调用上锁服务,并根据结果打印相关提示信息
    if(ctm2.arming_client.call(ctm2.arm_cmd))
    {
        ROS_INFO("uav2 disarmed success");
    }
    else
    {
        ROS_ERROR("uav2 disarmed failed");
    }

    if(ctm3.arming_client.call(ctm3.arm_cmd))
    {
        ROS_INFO("uav3 disarmed success");
    }
    else
    {
        ROS_ERROR("uav3 disarmed failed");
    }
    
    if(ctm1.arming_client.call(ctm1.arm_cmd))
    {
        ROS_INFO("uav1 disarmed success");
    }
    else
    {
        ROS_ERROR("uav1 disarmed failed");
    }

    if(ctm4.arming_client.call(ctm4.arm_cmd))
    {
        ROS_INFO("uav4 disarmed success");
    }
    else
    {
        ROS_ERROR("uav4 disarmed failed");
    }

    if(ctm5.arming_client.call(ctm5.arm_cmd))
    {
        ROS_INFO("uav5 disarmed success");
    }
    else
    {
        ROS_ERROR("uav5 disarmed failed");
    }
}

void formation::set_mode()
{
    //初始化,创建服务调用后客户端以及获取参数
    init();
    while(ros::ok())
    {
        //创建变量用以获取用户输入的值
        int mode;
        //打印提示信息
        if(flight_controller == "px4")
        {
            std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>--------<<<<<<<<<<<<<<<<<<<<<<<<<<< " << std::endl;
            std::cout << "Input the drone state:  0 for armed and offboard, 1 for land, 2 for disarmed" << std::endl;
            //获取用户输入的值
            std::cin >> formation_mode;
            switch(formation_mode)
            {
                //offboard模式
                case 0:
                    set_formation_px4_offboard();
                    break;
                
                //land模式
                case 1:
                    set_formation_px4_land();
                    break;

                //上锁
                case 2:
                    set_formation_disarmed();
                    break;
            }
        }
        else
        {
            if(flight_controller == "apm")
            {
                std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>--------<<<<<<<<<<<<<<<<<<<<<<<<<<< " << std::endl;
                std::cout << "Input the drone state:  0 for armed  guided and takeoff, 1 for land, 2 for disarmed" << std::endl;
                //获取用户输入的值
                std::cin >> formation_mode;
                switch(formation_mode)
                {
                    //guided模式
                    case 0:
                        set_formation_apm_guided();
                        break;
                
                    //land模式
                    case 1:
                        set_formation_apm_land();
                        break;

                    //上锁
                    case 2:
                        set_formation_disarmed();
                        break;
                }
            }
        }  
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "formation_set_mode");
    formation Formation;
    Formation.set_mode();
    return 0;
}