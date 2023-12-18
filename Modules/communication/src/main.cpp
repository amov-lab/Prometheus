#include "communication_bridge.hpp"
#include "rviz_reduce_the_frequency.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_station_bridge");
    ros::NodeHandle nh("~");

    printf("\033[1;32m---->[ground_station_bridge] start running\n\033[0m");

    CommunicationBridge communication_bridge_(nh);

    ReduceTheFrequency reduce_the_frequency_(nh);

    ros::spin();

    return 0;
}
