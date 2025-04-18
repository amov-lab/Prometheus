#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <plan_manage/ego_replan_fsm.h>
#include <prometheus_msgs/UAVControlState.h>
using namespace ego_planner;
// prometheus_msgs::UAVControlState rc_status;
// void rc_status_callback(const prometheus_msgs::UAVControlState::ConstPtr& msg){
//     // 获取遥控器控制状态
//     rc_status = *msg;
//     //std::cout << rc_status.control_state << std::endl;
// }

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ego_planner_node");
  ros::NodeHandle nh("~");
  // [订阅] 遥控器控制状态
  // ros::Subscriber rcstatus_sub = nh.subscribe<prometheus_msgs::UAVControlState>("/uav1/prometheus/control_state", 1,rc_status_callback);
  // bool start_ego_node = true;
  // while (start_ego_node)
  // {
  //   ros::spinOnce();
  //   if(rc_status.control_state!= 2){
  //     ROS_ERROR("Control mode should be COMMAND_CONTROL , please check the control mode of the drone.%i",rc_status.control_state);
  //   }else{
  //     start_ego_node = false;
  //     ROS_INFO("Control mode is COMMAND_CONTROL, start to run ego_planner_node.");
  //   }
  // }
  EGOReplanFSM rebo_replan;
  rebo_replan.init(nh);
  ros::spin();
  return 0;
}

