/*******************************************************************
 * 文件名:mocap_formation_control.cpp
 * 
 * 作者: BOSHEN97
 * 
 * 更新时间: 2020.11.20
 * 
 * 介绍:该cpp文件主要为动捕集群中四机绕圈相关函数的实现以及程序的运行
 * ****************************************************************/
#include "ros/ros.h"
#include "Formation.h"
#include <unistd.h>
#include <Eigen/Eigen>
#include <mavros_msgs/PositionTarget.h>

void formation::init()
{

	//集群四台飞机位置控制数据发布者
  	uav1_local_pub = n.advertise<mavros_msgs::PositionTarget>("/uav1/mavros/setpoint_raw/local", 10);
  	uav2_local_pub = n.advertise<mavros_msgs::PositionTarget>("/uav2/mavros/setpoint_raw/local", 10);
  	uav3_local_pub = n.advertise<mavros_msgs::PositionTarget>("/uav3/mavros/setpoint_raw/local", 10);
  	uav4_local_pub = n.advertise<mavros_msgs::PositionTarget>("/uav4/mavros/setpoint_raw/local", 10);

	//读取相关参数
	ros::param::param<double>("~square_length", square_length, 4);
	ros::param::param<double>("~13_height", point13_height, 1);
	ros::param::param<double>("~24_height", point24_height, 0.5);
	ros::param::param<double>("~hold_time", hold_time, 10);
	ros::param::param<double>("~stage1_time", stage1_time, 5);
	ros::param::param<int>("~LAND_intervals", land_intervals, 3);
	ros::param::param<bool>("~sim", sim, false);
	ros::param::param<double>("uav1_x", uav1_gazebo_offset_pose[0], 0);
  	ros::param::param<double>("uav1_y", uav1_gazebo_offset_pose[1], 0);
  	ros::param::param<double>("uav2_x", uav2_gazebo_offset_pose[0], 0);
  	ros::param::param<double>("uav2_y", uav2_gazebo_offset_pose[1], 0);
  	ros::param::param<double>("uav3_x", uav3_gazebo_offset_pose[0], 0);
  	ros::param::param<double>("uav3_y", uav3_gazebo_offset_pose[1], 0);
  	ros::param::param<double>("uav4_x", uav4_gazebo_offset_pose[0], 0);
  	ros::param::param<double>("uav4_y", uav4_gazebo_offset_pose[1], 0);
}

void formation::is_sim()
{
    if(!sim)
    {
        uav1_gazebo_offset_pose[0] = 0;
        uav1_gazebo_offset_pose[1] = 0;

        uav2_gazebo_offset_pose[0] = 0;
        uav2_gazebo_offset_pose[1] = 0;

        uav3_gazebo_offset_pose[0] = 0;
        uav3_gazebo_offset_pose[1] = 0;

        uav4_gazebo_offset_pose[0] = 0;
        uav4_gazebo_offset_pose[1] = 0;
    }
}

void formation::square()
{
	init();
	is_sim();
	while(ros::ok())
	{
		std::cout << "Please input 1 start square formation" << std::endl;
		int start_flag;
		std::cin >> start_flag;
		if(start_flag == 1)
		{
			//阶段1
			int stage = 1;
			if(stage == 1)
			{
				int count = 0;
				int num = (stage1_time + hold_time) * 10;
				uav1_desired_pose.type_mask = 0b100111111000;
				uav1_desired_pose.coordinate_frame = 1;
				uav1_desired_pose.position.x = square_length/2 - uav1_gazebo_offset_pose[0];
				uav1_desired_pose.position.y = square_length/2 - uav1_gazebo_offset_pose[1];
				uav1_desired_pose.position.z = point13_height;

				uav2_desired_pose.type_mask = 0b100111111000;
				uav2_desired_pose.coordinate_frame = 1;
				uav2_desired_pose.position.x = square_length/2 - uav2_gazebo_offset_pose[0];
				uav2_desired_pose.position.y = -square_length/2 - uav2_gazebo_offset_pose[1];
				uav2_desired_pose.position.z = point24_height;

				uav3_desired_pose.type_mask = 0b100111111000;
				uav3_desired_pose.coordinate_frame = 1;
				uav3_desired_pose.position.x = -square_length/2 - uav3_gazebo_offset_pose[0];
				uav3_desired_pose.position.y = -square_length/2 - uav3_gazebo_offset_pose[1];
				uav3_desired_pose.position.z = point13_height;

				uav4_desired_pose.type_mask = 0b100111111000;
				uav4_desired_pose.coordinate_frame = 1;
				uav4_desired_pose.position.x = -square_length/2 - uav4_gazebo_offset_pose[0];
				uav4_desired_pose.position.y = square_length/2 - uav4_gazebo_offset_pose[1];
				uav4_desired_pose.position.z = point24_height;

				while(ros::ok())
				{
					uav1_desired_pose.header.stamp = ros::Time::now();
					uav1_local_pub.publish(uav1_desired_pose);
					uav2_desired_pose.header.stamp = ros::Time::now();
					uav2_local_pub.publish(uav2_desired_pose);
					uav3_desired_pose.header.stamp = ros::Time::now();
					uav3_local_pub.publish(uav3_desired_pose);
					uav4_desired_pose.header.stamp = ros::Time::now();
					uav4_local_pub.publish(uav4_desired_pose);
					if(count == 0)
					{
						set_formation_px4_offboard();
					}
					count++;
					if(count >= num)
					{
						stage = 2;
						break;
					}
					ROS_INFO("go to point 1");
					usleep(100000);
				}
			}
			
			if(stage == 2)
			{
				int count = 0;
				int num = hold_time * 10;
				uav1_desired_pose.type_mask = 0b100111111000;
				uav1_desired_pose.coordinate_frame = 1;
				uav1_desired_pose.position.x = square_length/2 - uav1_gazebo_offset_pose[0];
				uav1_desired_pose.position.y = -square_length/2 - uav1_gazebo_offset_pose[1];
				uav1_desired_pose.position.z = point24_height;

				uav2_desired_pose.type_mask = 0b100111111000;
				uav2_desired_pose.coordinate_frame = 1;
				uav2_desired_pose.position.x = -square_length/2 - uav2_gazebo_offset_pose[0];
				uav2_desired_pose.position.y = -square_length/2 - uav2_gazebo_offset_pose[1];
				uav2_desired_pose.position.z = point13_height;

				uav3_desired_pose.type_mask = 0b100111111000;
				uav3_desired_pose.coordinate_frame = 1;
				uav3_desired_pose.position.x = -square_length/2 - uav3_gazebo_offset_pose[0];
				uav3_desired_pose.position.y = square_length/2 - uav3_gazebo_offset_pose[1];
				uav3_desired_pose.position.z = point24_height;

				uav4_desired_pose.type_mask = 0b100111111000;
				uav4_desired_pose.coordinate_frame = 1;
				uav4_desired_pose.position.x = square_length/2 - uav4_gazebo_offset_pose[0];
				uav4_desired_pose.position.y = square_length/2 - uav4_gazebo_offset_pose[1];
				uav4_desired_pose.position.z = point13_height;

				while(ros::ok())
				{
					uav1_desired_pose.header.stamp = ros::Time::now();
					uav1_local_pub.publish(uav1_desired_pose);
					uav2_desired_pose.header.stamp = ros::Time::now();
					uav2_local_pub.publish(uav2_desired_pose);
					uav3_desired_pose.header.stamp = ros::Time::now();
					uav3_local_pub.publish(uav3_desired_pose);
					uav4_desired_pose.header.stamp = ros::Time::now();
					uav4_local_pub.publish(uav4_desired_pose);
					count++;
					if(count >= num)
					{
						stage = 3;
						break;
					}
					ROS_INFO("go to point 2");
					usleep(100000);
				}
			}

			if(stage == 3)
			{
				int count = 0;
				int num = hold_time * 10;
				uav1_desired_pose.type_mask = 0b100111111000;
				uav1_desired_pose.coordinate_frame = 1;
				uav1_desired_pose.position.x = -square_length/2 - uav1_gazebo_offset_pose[0];
				uav1_desired_pose.position.y = -square_length/2 - uav1_gazebo_offset_pose[1];
				uav1_desired_pose.position.z = point13_height;

				uav2_desired_pose.type_mask = 0b100111111000;
				uav2_desired_pose.coordinate_frame = 1;
				uav2_desired_pose.position.x = -square_length/2 - uav2_gazebo_offset_pose[0];
				uav2_desired_pose.position.y = square_length/2 - uav2_gazebo_offset_pose[1];
				uav2_desired_pose.position.z = point24_height;

				uav3_desired_pose.type_mask = 0b100111111000;
				uav3_desired_pose.coordinate_frame = 1;
				uav3_desired_pose.position.x = square_length/2 - uav3_gazebo_offset_pose[0];
				uav3_desired_pose.position.y = square_length/2 - uav3_gazebo_offset_pose[1];
				uav3_desired_pose.position.z = point13_height;

				uav4_desired_pose.type_mask = 0b100111111000;
				uav4_desired_pose.coordinate_frame = 1;
				uav4_desired_pose.position.x = square_length/2 - uav4_gazebo_offset_pose[0];
				uav4_desired_pose.position.y = -square_length/2 - uav4_gazebo_offset_pose[1];
				uav4_desired_pose.position.z = point24_height;

				while(ros::ok())
				{
					uav1_desired_pose.header.stamp = ros::Time::now();
					uav1_local_pub.publish(uav1_desired_pose);
					uav2_desired_pose.header.stamp = ros::Time::now();
					uav2_local_pub.publish(uav2_desired_pose);
					uav3_desired_pose.header.stamp = ros::Time::now();
					uav3_local_pub.publish(uav3_desired_pose);
					uav4_desired_pose.header.stamp = ros::Time::now();
					uav4_local_pub.publish(uav4_desired_pose);
					count++;
					if(count >= num)
					{
						stage = 4;
						break;
					}
					ROS_INFO("go to point 3");
					usleep(100000);
				}
			}

			if(stage == 4)
			{
				int count = 0;
				int num = hold_time * 10;
				uav1_desired_pose.type_mask = 0b100111111000;
				uav1_desired_pose.coordinate_frame = 1;
				uav1_desired_pose.position.x = -square_length/2 - uav1_gazebo_offset_pose[0];
				uav1_desired_pose.position.y = square_length/2 - uav1_gazebo_offset_pose[1];
				uav1_desired_pose.position.z = point24_height;

				uav2_desired_pose.type_mask = 0b100111111000;
				uav2_desired_pose.coordinate_frame = 1;
				uav2_desired_pose.position.x = square_length/2 - uav2_gazebo_offset_pose[0];
				uav2_desired_pose.position.y = square_length/2 - uav2_gazebo_offset_pose[1];
				uav2_desired_pose.position.z = point13_height;

				uav3_desired_pose.type_mask = 0b100111111000;
				uav3_desired_pose.coordinate_frame = 1;
				uav3_desired_pose.position.x = square_length/2 - uav3_gazebo_offset_pose[0];
				uav3_desired_pose.position.y = -square_length/2 - uav3_gazebo_offset_pose[1];
				uav3_desired_pose.position.z = point24_height;

				uav4_desired_pose.type_mask = 0b100111111000;
				uav4_desired_pose.coordinate_frame = 1;
				uav4_desired_pose.position.x = -square_length/2 - uav4_gazebo_offset_pose[0];
				uav4_desired_pose.position.y = -square_length/2 - uav4_gazebo_offset_pose[1];
				uav4_desired_pose.position.z = point13_height;

				while(ros::ok())
				{
					uav1_desired_pose.header.stamp = ros::Time::now();
					uav1_local_pub.publish(uav1_desired_pose);
					uav2_desired_pose.header.stamp = ros::Time::now();
					uav2_local_pub.publish(uav2_desired_pose);
					uav3_desired_pose.header.stamp = ros::Time::now();
					uav3_local_pub.publish(uav3_desired_pose);
					uav4_desired_pose.header.stamp = ros::Time::now();
					uav4_local_pub.publish(uav4_desired_pose);
					count++;
					if(count >= num)
					{
						stage = 5;
						break;
					}
					ROS_INFO("go to point 4");
					usleep(100000);
				}
			}
			
			if(stage == 5)
			{
				int count = 0;
				int num = hold_time * 10;
				uav1_desired_pose.type_mask = 0b100111111000;
				uav1_desired_pose.coordinate_frame = 1;
				uav1_desired_pose.position.x = square_length/2 - uav1_gazebo_offset_pose[0];
				uav1_desired_pose.position.y = square_length/2 - uav1_gazebo_offset_pose[1];
				uav1_desired_pose.position.z = point13_height;

				uav2_desired_pose.type_mask = 0b100111111000;
				uav2_desired_pose.coordinate_frame = 1;
				uav2_desired_pose.position.x = square_length/2 - uav2_gazebo_offset_pose[0];
				uav2_desired_pose.position.y = -square_length/2 - uav2_gazebo_offset_pose[1];
				uav2_desired_pose.position.z = point24_height;

				uav3_desired_pose.type_mask = 0b100111111000;
				uav3_desired_pose.coordinate_frame = 1;
				uav3_desired_pose.position.x = -square_length/2 - uav3_gazebo_offset_pose[0];
				uav3_desired_pose.position.y = -square_length/2 - uav3_gazebo_offset_pose[1];
				uav3_desired_pose.position.z = point13_height;

				uav4_desired_pose.type_mask = 0b100111111000;
				uav4_desired_pose.coordinate_frame = 1;
				uav4_desired_pose.position.x = -square_length/2 - uav4_gazebo_offset_pose[0];
				uav4_desired_pose.position.y = square_length/2 - uav4_gazebo_offset_pose[1];
				uav4_desired_pose.position.z = point24_height;

				while(ros::ok())
				{
					uav1_desired_pose.header.stamp = ros::Time::now();
					uav1_local_pub.publish(uav1_desired_pose);
					uav2_desired_pose.header.stamp = ros::Time::now();
					uav2_local_pub.publish(uav2_desired_pose);
					uav3_desired_pose.header.stamp = ros::Time::now();
					uav3_local_pub.publish(uav3_desired_pose);
					uav4_desired_pose.header.stamp = ros::Time::now();
					uav4_local_pub.publish(uav4_desired_pose);
					count++;
					if(count >= num)
					{ 
						//四机降落
						set_formation_px4_land();
						break;
					}
					ROS_INFO("go to point 1");
					usleep(100000);
				}
			}
			break;
		}
		else
		{
			ROS_WARN("input error, please input again");
		}
	}
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
    //创建1~4号机的command_to_mavros类
    command_to_mavros ctm1("uav1");

    command_to_mavros ctm2("uav2");

    command_to_mavros ctm3("uav3");

    command_to_mavros ctm4("uav4");

    //设置1~4号机模式变量为offboard,解上锁变量为解锁
    ctm1.arm_cmd.request.value = true;
    ctm2.arm_cmd.request.value = true;
    ctm3.arm_cmd.request.value = true;
    ctm4.arm_cmd.request.value = true;

    ctm1.mode_cmd.request.custom_mode = "OFFBOARD";
    ctm2.mode_cmd.request.custom_mode = "OFFBOARD";
    ctm3.mode_cmd.request.custom_mode = "OFFBOARD";
    ctm4.mode_cmd.request.custom_mode = "OFFBOARD";

    //集群按照1234的顺序对五台无人机分别解锁并切入offboard模式
    //当有一台无人机解锁或者切入offboard模式失败,该函数返回false
    //五台无人机成功解锁并切入offboard模式,该函数返回true
    if(ctm1.arming_client.call(ctm1.arm_cmd) && ctm1.set_mode_client.call(ctm1.mode_cmd))
    {
        ROS_INFO("uav1 armed and set offboard mode success");
        is_wait(offboard_intervals);
    }
    else
    {
        ROS_ERROR("uav1 armed and set offboard mode failed");
    }

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

    if(ctm4.arming_client.call(ctm4.arm_cmd) && ctm4.set_mode_client.call(ctm4.mode_cmd))
    {
        ROS_INFO("uav4 armed and set offboard mode success");
        is_wait(offboard_intervals);
    }
    else
    {
        ROS_ERROR("uav4 armed and set offboard mode failed");
    }

}

void formation::set_formation_px4_land()
{
    //创建1~4号机的command_to_mavros类
    command_to_mavros ctm1("uav1");

    command_to_mavros ctm2("uav2");

    command_to_mavros ctm3("uav3");

    command_to_mavros ctm4("uav4");

    //设置1~4号机模式变量为land
    ctm1.mode_cmd.request.custom_mode = "AUTO.LAND";
    ctm2.mode_cmd.request.custom_mode = "AUTO.LAND";
    ctm3.mode_cmd.request.custom_mode = "AUTO.LAND";
    ctm4.mode_cmd.request.custom_mode = "AUTO.LAND";

    //切换为land模式,并对结果进行打印
	if(ctm1.set_mode_client.call(ctm1.mode_cmd))
    {
        ROS_INFO("uav1 set land mode success");
        is_wait(land_intervals);
    }
    else
    {
        ROS_ERROR("uav1 set land mode failed");
    }

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

    if(ctm4.set_mode_client.call(ctm4.mode_cmd))
    {
        ROS_INFO("uav4 set land mode success");
        is_wait(land_intervals);
    }
    else
    {
        ROS_ERROR("uav4 set land mode success");
    }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "formation_square");
	formation Formation;
	Formation.square();
	return 0;
}


































