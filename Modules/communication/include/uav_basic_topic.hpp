#ifndef UAV_BASIC_TOPIC_HPP
#define UAV_BASIC_TOPIC_HPP

#include <ros/ros.h>
#include "communication.hpp"
#include "prometheus_msgs/UAVState.h"
#include "prometheus_msgs/TextInfo.h"
#include "prometheus_msgs/OffsetPose.h"
#include "prometheus_msgs/UAVCommand.h"
#include "prometheus_msgs/UAVSetup.h"
#include "prometheus_msgs/UAVControlState.h"

#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/AttitudeTarget.h"
// 吊舱,图像相关控制
#include "std_srvs/SetBool.h"
#include "prometheus_msgs/GimbalState.h"
#include "prometheus_msgs/Control.h"
#include "prometheus_msgs/GimbalControl.h"
// 自定义消息
#include "prometheus_msgs/CustomDataSegment.h"
#include "prometheus_msgs/BasicDataTypeAndValue.h"
#include "prometheus_msgs/ParamSettings.h"
#include "custom_data_segment.hpp"

#include "rviz_reduce_the_frequency.hpp"

#include "message_convert.hpp"

class UAVBasic
{
public:
    UAVBasic();

    UAVBasic(ros::NodeHandle &nh,int id,Communication *communication);

    ~UAVBasic();

    inline int getRobotId(){return robot_id;};

    void stateCb(const prometheus_msgs::UAVState::ConstPtr &msg);

    //【回调】uav反馈信息
    void textInfoCb(const prometheus_msgs::TextInfo::ConstPtr &msg);

    void controlStateCb(const prometheus_msgs::UAVControlState::ConstPtr &msg);

    void offsetPoseCb(const prometheus_msgs::OffsetPose::ConstPtr &msg);

    struct UAVState getUAVState();

    void uavCmdPub(struct UAVCommand uav_cmd);

    void uavCmdCb(const prometheus_msgs::UAVCommand::ConstPtr &msg);

    void uavSetupPub(struct UAVSetup uav_setup);

    void setTimeStamp(uint time);

    uint getTimeStamp();

    void uavTargetPub(struct WindowPosition window_position);

    void send(const ros::TimerEvent &time_event);

    void px4PosTargetCb(const mavros_msgs::PositionTarget::ConstPtr &msg);
    void px4AttTargetCb(const mavros_msgs::AttitudeTarget::ConstPtr &msg);

    void gimbalControlPub(struct GimbalControl gimbal_control);
    void gimbalServer(struct GimbalService gimbal_service);
    void gimbalStateCb(const prometheus_msgs::GimbalState::ConstPtr &msg);

    void customDataSegmentCb(const prometheus_msgs::CustomDataSegment::ConstPtr &msg);
    void customDataSegmentPub(struct CustomDataSegment_1 custom_data_segment);

    void gimbalControlPubTimer(const ros::TimerEvent &time_event);

    // MAVLINK 消息： SERIAL_CONTROL(126)的发布/接收 
    // 这里主要模拟一个QGC终端 MAVLINK CONSOLE 的实现
    void serialControlPub(const std::string &cmd);
    void serialControlCb(const mavros_msgs::Mavlink::ConstPtr &msg);

    void paramSettingsPub(struct ParamSettings param_settings, std::string prefix = "");

    void setGroundStationIP(std::string ip);

private:
    ros::Subscriber uav_state_sub_;

    // 反馈信息
    ros::Subscriber text_info_sub_;
    // 控制状态
    ros::Subscriber uav_control_state_sub_;
    ros::Publisher uav_cmd_pub_;
    ros::Publisher uav_setup_pub_;
    ros::Subscriber uav_cmd_sub_;
    ros::Publisher uav_target_pub_;
    ros::Publisher gimbal_control_pub_;
    ros::Subscriber px4_position_target_sub_;
    ros::Subscriber px4_attitude_target_sub_;

    ros::Subscriber gimbal_state_sub_;

    ros::Subscriber offset_pose_sub_;

    // 自定义消息
    ros::Subscriber custom_data_segment_sub_;
    ros::Publisher custom_data_segment_pub_;

    // SERIAL_CONTROL(126)
    ros::Subscriber serial_control_sub_;
    ros::Publisher serial_control_pub_;

    ros::Publisher param_settings_pub_;

    ros::ServiceClient gimbal_home_client_;
    ros::ServiceClient gimbal_take_client_;
    ros::ServiceClient local_take_client_;
    ros::ServiceClient gimbal_record_client_;
    ros::ServiceClient local_record_client_;

    int current_mode_;

    int robot_id;

    struct UAVState uav_state_;
    struct TextInfo text_info_;
    struct UAVControlState uav_control_state_;
    struct UAVCommand uav_command_;
    struct CustomDataSegment_1 custom_data_segment_;

    Communication *communication_ = NULL;

    std::string multicast_udp_ip;
    std::string ground_station_ip;

    uint time_stamp_ = 0;

    // 只控制 uav_state、uav_command 和 uav_control_state 的发送频率
    ros::Timer send_timer;
    int send_hz;
    // 下列变量仅在发送定时器中有效，为判断当前数据是否刷新
    bool uav_state_ready = false;
    bool uav_control_state_ready = false;
    bool uav_command_ready = false;
    bool custom_data_segment_ready = false;

    // 吊舱类型 
    int gimbal_type = 0;
    // 吊舱控制 定时器持续发送
    ros::Timer gimbal_control_pub_timer;
    prometheus_msgs::GimbalControl gimbal_control_;
    prometheus_msgs::OffsetPose offset_pose_;

    // 
    ReduceTheFrequency *reduce_the_frequency_;
};

#endif