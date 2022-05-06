
// 要求:
// 1,不用写成类,用普通的ros文件 main函数写就行
// 2,配套的是P450,outdoor(GPS),目前可考虑仅支持仿真,不考虑真实情况
// 3,根据agent_num参数来确定发布多少个话题(位置控制 XYZ_POS): "/uav" + std::to_string(this->agent_id_) + "/prometheus/command"
// 4,简易阵型即可,可以使用我们之前写好的支持任意个飞机的阵型h文件
// 5,在本程序中可以尝试加入等待避障或者相关简易避障策略
// 6,本程序和 swarm_command这个自定义消息无关

// 大致效果
// roslaunch prometheus_gazebo sitl_outdoor_4uav.launch
// roslaunch prometheus_uav_control uav_control_main_outdoor_4uav.launch

// roslaunch prometheus_demo formation_control.launch

// 确定启动程序没问题,遥控器一键解锁,一键切换至hover_control,遥控器正常1控多
// 切换至command_control,飞机悬停等待本程序的指令
// 飞机可以切换3个左右阵型,实现机动
// 降落
// 本程序相关教学:
// 1, 多个sdf的编写,端口如何与px4对应
// 2, uav_control中相关的集群接口
// 3, ...

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVSetup.h>
#include <prometheus_msgs/GpsData.h>

// 输入参数：　阵型，阵型基本尺寸，集群数量
// 所有的阵型和数量必须提前预设!!
// swarm_shape: 0代表一字队形,1代表三角队形
Eigen::MatrixXf getFormationSeparation(int swarm_shape, float swarm_size, int swarm_num_uav)
{
    //矩阵大小为　swarm_num_uav＊4 , 对应x,y,z,yaw四个自由度的分离量
    Eigen::MatrixXf seperation(swarm_num_uav,4); 

    //判断是否为奇数,位操作符&,如果为奇数,则2的0次方一定为1,那么奇数&1就等于1,偶数&1就等于0
    if((swarm_num_uav & 1) == 1)
    {
        int diff_ont_column_odd_value = (swarm_num_uav - 1) / 2;
        int diff_triangle_odd_value = (swarm_num_uav - 1) / 2;
        switch(swarm_shape)
        {
            //一字队形
            case 0:
                for(int i = 0; i < swarm_num_uav; i++)
                {
                    seperation(i,0) = 0.0;
                    seperation(i,1) = i - diff_ont_column_odd_value;
                    seperation(i,2) = 0.0;
                }
                break;

            //三角队形
            case 1:
                for(int i = 0; i <=diff_triangle_odd_value; i++)
                {
                    seperation(i,0) = i;
                    seperation(i,1) = i - diff_triangle_odd_value;
                    seperation(i,2) = 0.0;
                    
                    seperation(swarm_num_uav - 1 - i,0) = i;
                    seperation(swarm_num_uav - 1 - i,1) = diff_triangle_odd_value - i;
                    seperation(swarm_num_uav - 1 - i,2) = 0.0; 
                }
                break;
        }    
    }
    else
    {
        float diff_one_column_even_value = swarm_num_uav / 2.0 - 0.5;
        int diff_triangle_even_value = swarm_num_uav / 2;
        switch (swarm_shape)
        {
            //一字队形
            case 0:
                for(int i = 0; i < swarm_num_uav; i++)
                {
                    seperation(i,0) = 0.0;
                    seperation(i,1) = i - diff_one_column_even_value;
                    seperation(i,2) = 0.0;
                }
                break;

            //三角队形
            case 1:
                for(int i = 0; i < diff_triangle_even_value; i++)
                {
                    seperation(i,0) = i;
                    seperation(i,1) = i + 1 - diff_triangle_even_value;
                    seperation(i,2) = 0.0;

                    if(i+1 == diff_triangle_even_value)
                    {
                        seperation(swarm_num_uav - 1 - i,0) = 0; 
                    }
                    else
                    {
                        seperation(swarm_num_uav - 1 - i,0) = i;
                    }
                    seperation(swarm_num_uav - 1 - i,1) = diff_triangle_even_value - i - 1;
                    seperation(swarm_num_uav - 1 - i,2) = 0.0;
                }
                break;
        }
    }
    
    for(int i = 0 ; i < swarm_num_uav ; i++)
    {
        for(int j = 0 ; j < 2; j++)
        {
            seperation(i,j) *= swarm_size;
        }
    }

    return seperation;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "formation_control");
    ros::NodeHandle n("~");
    int agent_num;
    int formation_size;
    n.param<int>("agent_num", agent_num, 3);
    n.param<int>("formation_size", formation_size, 1);
    
    int formation_shape;
    Eigen::Vector3d leader_pos;
    Eigen::MatrixXf separation = getFormationSeparation(0, formation_size, agent_num);

    prometheus_msgs::GpsData origin_gps;
    origin_gps.latitude = 30.7852600;
    origin_gps.longitude = 103.8610300;
    origin_gps.altitude = 100.0;

    ros::Publisher uav_command_pub[agent_num];
    ros::Publisher set_local_pose_offset_pub[agent_num];

    for(int i=0; i<agent_num; i++)
    {
        uav_command_pub[i] = n.advertise<prometheus_msgs::UAVCommand>("/uav" + std::to_string(i+1) + "/prometheus/command", 10);
        set_local_pose_offset_pub[i] = n.advertise<prometheus_msgs::GpsData>("/uav" + std::to_string(i+1) + "/prometheus/set_local_offset_pose", 10);
    }

    std::cout << "Please input 1 to continue" << std::endl;
    int start_flag;
    std::cin >>start_flag;

    for(int i=0; i<agent_num; i++)
    {
        set_local_pose_offset_pub[i].publish(origin_gps);
    }

    while(ros::ok())
    {     
        int control_command;
        std::cout << "Please input control command: 0 for XYZ_POS, 1 for One_Column, 2 for Triangle" << std::endl;
        std::cin >> control_command;
        if(control_command == 0)
        {
            std::cout << "Please input X [m]:" << std::endl;
            std::cin >> leader_pos[0];

            std::cout << "Please input Y [m]:" << std::endl;
            std::cin >> leader_pos[1];

            std::cout << "Please input Z [m]:" << std::endl;
            std::cin >> leader_pos[2];
        }else if(control_command == 1)
        {
            separation = getFormationSeparation(0, formation_size, agent_num);
        }else if(control_command == 2)
        {
            separation = getFormationSeparation(1, formation_size, agent_num);
        }   

        for(int i=0; i<agent_num; i++)
        {
            prometheus_msgs::UAVCommand uav_command;
            uav_command.header.stamp = ros::Time::now();
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
            uav_command.position_ref[0] = leader_pos[0] + separation(i,0);
            uav_command.position_ref[1] = leader_pos[1] + separation(i,1);
            uav_command.position_ref[2] = leader_pos[2] + separation(i,2);

            uav_command_pub[i].publish(uav_command); 
        }
    }
}