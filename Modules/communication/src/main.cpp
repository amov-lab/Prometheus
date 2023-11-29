#include "communication_bridge.hpp"
#include "custom_data_segment.hpp"
#include "rviz_topic.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_station_bridge");
    ros::NodeHandle nh("~");

    printf("\033[1;32m---->[ground_station_bridge] start running\n\033[0m");

    CommunicationBridge communication_bridge_(nh);

    // GFKD gfkd(nh);

    ros::spin();

    return 0;
}
