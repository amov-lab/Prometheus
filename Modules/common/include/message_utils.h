/***************************************************************************************************************************
* message_utils.h
*
* Author: Jario
*
* Update Time: 2020.4.29
***************************************************************************************************************************/
#ifndef PROMETHEUS_MESSAGE_UTILS_H
#define PROMETHEUS_MESSAGE_UTILS_H


#include <string>
#include <prometheus_msgs/Message.h>


void pub_message(ros::Publisher& puber, int msg_type, std::string source_node, std::string msg_content)
{
    prometheus_msgs::Message exect_msg;
    exect_msg.header.stamp = ros::Time::now();
    exect_msg.message_type = msg_type;
    exect_msg.source_node = source_node;
    exect_msg.content = msg_content;
    puber.publish(exect_msg);
}


#endif
