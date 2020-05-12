#include "plan_manage/tools.h"
namespace dyn_planner
{
// void pub_msg(ros::Publisher & puber, std::string mmm, int type){
//     prometheus_msgs::Message exect_msg;
//     exect_msg.header.stamp = ros::Time::now();
//     exect_msg.message_type=type;
//     exect_msg.content = mmm;
//     puber.publish(exect_msg);
// }

ros::Publisher message_pub;
}