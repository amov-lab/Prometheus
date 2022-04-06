#include <ros/ros.h>
#include <prometheus_msgs/SwarmCommand.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "groud_station_node");
    ros::NodeHandle n("~");
    prometheus_msgs::SwarmCommand swarm_cmd;
    int swarm_num, location_source;
    float swarm_size;
    n.param<int>("swarm_num", swarm_num, 3);
    n.param<int>("location_source", location_source, 4);
    n.param<float>("swarm_size", swarm_size, 1.0);
    swarm_cmd.swarm_num = swarm_num;
    swarm_cmd.swarm_location_source = location_source;
    swarm_cmd.swarm_size = swarm_size;
    swarm_cmd.source = "groud_station_node";
    ros::Publisher swarm_cmd_pub = n.advertise<prometheus_msgs::SwarmCommand>("/prometheus/swarm_command", 10);
    int command;
    std::cout << "----------------------------------------------" << std::endl;

    while(ros::ok())
    {
        std::cout << "Please input swarm control command: 1 for Init, 2 for Start, 3 for Hold, 4 for Stop, 5 for Formation" << std::endl;
        std::cin >> command;
        switch (command)
        {
        case prometheus_msgs::SwarmCommand::Init:
            swarm_cmd.header.stamp = ros::Time::now();
            swarm_cmd.Swarm_CMD = prometheus_msgs::SwarmCommand::Init;
            swarm_cmd_pub.publish(swarm_cmd);
            break;
        
        case prometheus_msgs::SwarmCommand::Start:
            swarm_cmd.header.stamp = ros::Time::now();
            swarm_cmd.Swarm_CMD = prometheus_msgs::SwarmCommand::Start;
            swarm_cmd_pub.publish(swarm_cmd);
            break;
        
        case prometheus_msgs::SwarmCommand::Hold:
            swarm_cmd.header.stamp = ros::Time::now();
            swarm_cmd.Swarm_CMD = prometheus_msgs::SwarmCommand::Hold;
            swarm_cmd_pub.publish(swarm_cmd);
            break;

        case prometheus_msgs::SwarmCommand::Stop:
            swarm_cmd.header.stamp = ros::Time::now();
            swarm_cmd.Swarm_CMD = prometheus_msgs::SwarmCommand::Stop;
            swarm_cmd_pub.publish(swarm_cmd);
            break;
        
        case prometheus_msgs::SwarmCommand::Formation:
            swarm_cmd.header.stamp = ros::Time::now();
            swarm_cmd.Swarm_CMD = prometheus_msgs::SwarmCommand::Formation;
            int formation_flag;
            std::cout << "Please input formation, o for one_column, 1 for triangle, 2 for Square:" << std::endl;
            std::cin >> formation_flag;
            switch(formation_flag)
            {
                case 0:
                    swarm_cmd.swarm_shape = prometheus_msgs::SwarmCommand::One_column;
                    break;
                case 1:
                    swarm_cmd.swarm_shape = prometheus_msgs::SwarmCommand::Triangle;
                    break;
                case 2:
                    swarm_cmd.swarm_shape = prometheus_msgs::SwarmCommand::Square;
                    break;
            }
            std::cout << "Please input position x:" << std::endl;
            std::cin >> swarm_cmd.leader_pos[0];
            std::cout << "Please input position y:" << std::endl;
            std::cin >> swarm_cmd.leader_pos[1];
            std::cout << "Please input position z:" << std::endl;
            std::cin >> swarm_cmd.leader_pos[2];
            swarm_cmd_pub.publish(swarm_cmd);
            break;
        }
        std::cout << "----------------------------------------------" << std::endl;
    }
}
