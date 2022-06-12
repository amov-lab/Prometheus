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

prometheus_msgs::GPSData origin_gps;

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

void uav1_state_cb(const prometheus_msgs::UAVState::ConstPtr &msg)
{
    origin_gps.latitude  = msg->latitude;
    origin_gps.longitude = msg->longitude;
    origin_gps.altitude  = msg->altitude;
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

    ros::Publisher uav_command_pub[agent_num];
    ros::Publisher set_local_pose_offset_pub[agent_num];
    ros::Subscriber uav1_state_sub = n.subscribe<prometheus_msgs::UAVState>("/uav1/prometheus/state", 1, uav1_state_cb);

    for(int i=0; i<agent_num; i++)
    {
        uav_command_pub[i] = n.advertise<prometheus_msgs::UAVCommand>("/uav" + std::to_string(i+1) + "/prometheus/command", 10);
        set_local_pose_offset_pub[i] = n.advertise<prometheus_msgs::GPSData>("/uav" + std::to_string(i+1) + "/prometheus/set_local_offset_pose", 10);
    }

    //此处做一个阻塞确保无人机全部正常启动后,能够将无人机原点坐标系统一到1号无人机所在位置
    std::cout << "Please input 1 to continue" << std::endl;
    int start_flag;
    std::cin >>start_flag;
    ros::spinOnce();
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