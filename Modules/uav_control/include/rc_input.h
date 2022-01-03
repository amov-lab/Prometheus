#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>

class RC_Input
{
    public:
        RC_Input();
        void handle_rc_data(mavros_msgs::RCInConstPtr pMsg);
        
        double ch[4];           // 1-4通道数值: roll pitch yaw thrust 
        double channel_5;       // 通道5
        double channel_6;       // 通道6
        double channel_7;       // 通道7
        double channel_8;       // 通道8
        double last_channel_5;
        double last_channel_6;
        double last_channel_7;
        double last_channel_8;
        bool have_init_channel_5{false};
        bool have_init_channel_6{false};
        bool have_init_channel_7{false};
        bool have_init_channel_8{false};

        mavros_msgs::RCIn msg;
        ros::Time rcv_stamp;            // 收到遥控器消息的时间
        
        bool enter_hover_control;
        bool in_hover_control;
        bool enter_command_control;
        bool in_command_control;
        
        bool toggle_reboot;
        bool toggle_land;

    private:

        static constexpr double channel_5_threshold_value = 0.75;
        static constexpr double channel_6_threshold_value = 0.75;
        static constexpr double channel_7_threshold_value = 0.5;    // 0.5 0.75的区别是？
        static constexpr double channel_8_threshold_value = 0.5;
        static constexpr double DEAD_ZONE = 0.1;                    // 死区
        
        void check_validity();
};

RC_Input::RC_Input()
{
  rcv_stamp = ros::Time(0);

  last_channel_5 = -1.0;
  last_channel_6 = -1.0;
  last_channel_7 = -1.0;
  last_channel_8 = -1.0;

  in_command_control = false;
  enter_command_control = false;
  in_hover_control = false;
  enter_hover_control = false;
  toggle_reboot = false;
  toggle_land = false;
}

void RC_Input::handle_rc_data(mavros_msgs::RCInConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    // 1-4通道数值
    for (int i = 0; i < 4; i++)
    {
        // 映射至[-1,1]区间
        ch[i] = ((double)msg.channels[i] - 1500.0) / 500.0;
        if (ch[i] > DEAD_ZONE)
            ch[i] = (ch[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
        else if (ch[i] < -DEAD_ZONE)
            ch[i] = (ch[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
        else
            ch[i] = 0.0;
    }

    // 5通道，映射至[0,1]区间？
    channel_5 = ((double)msg.channels[4] - 1000.0) / 1000.0;
    // 6通道，映射至[0,1]区间
    channel_6 = ((double)msg.channels[5] - 1000.0) / 1000.0;
    // 7通道，映射至[0,1]区间
    channel_7 = ((double)msg.channels[6] - 1000.0) / 1000.0;
    // 8通道，映射至[0,1]区间
    channel_8 = ((double)msg.channels[7] - 1000.0) / 1000.0;

    // 检验有效性
    check_validity();

    // 初始化
    if (!have_init_channel_5)
    {
        have_init_channel_5 = true;
        last_channel_5 = channel_5;
    }
    if (!have_init_channel_6)
    {
        have_init_channel_6 = true;
        last_channel_6 = channel_6;
    }
    if (!have_init_channel_7)
    {
        have_init_channel_7 = true;
        last_channel_7 = channel_7;
    }
    if (!have_init_channel_8)
    {
        have_init_channel_8 = true;
        last_channel_8 = channel_8;
    }

    // 判断通道5 - 上一时刻小于threshold_value，本时刻大于threshold_value，则判断进行一次切换
    if (last_channel_5 < channel_5_threshold_value && channel_5 > channel_5_threshold_value)
        enter_hover_control = true;
    else
        enter_hover_control = false;

    // 本时刻大于threshold_value，则认定当前在该模式（即必须将摇杆保持在该状态，不能随意切换）
    if (channel_5 > channel_5_threshold_value)
        in_hover_control = true;
    else
        in_hover_control = false;

    // 判断通道6 - 必须先进入hover_control，才能判断是否进入command_control
    if (in_hover_control)
    {
        if (last_channel_6 < channel_6_threshold_value && channel_6 > channel_6_threshold_value)
            enter_command_control = true;
        else 
            enter_command_control = false;

        if (channel_6 > channel_6_threshold_value)
            in_command_control = true;
        else
            in_command_control = false;
    }

    // 判断通道7 - 必须hover_control或者command_control，才可自动降落
    if (in_hover_control || in_command_control)
    {
        if (last_channel_7 < channel_7_threshold_value && channel_7 > channel_7_threshold_value)
            toggle_land = true;
        else
            toggle_land = false;
    }
    else
        toggle_land = false;

    // 判断通道8 - 必须非hover_control、非command_control，才可重启
    if (!in_hover_control && !in_command_control)
    {
        if (last_channel_8 < channel_8_threshold_value && channel_8 > channel_8_threshold_value)
            toggle_reboot = true;
        else
            toggle_reboot = false;
    }
    else
        toggle_reboot = false;

    last_channel_5 = channel_5;
    last_channel_6 = channel_6;
    last_channel_7 = channel_7;
    last_channel_8 = channel_8;
}

void RC_Input::check_validity()
{
    if (channel_5 >= -1.1 && channel_5 <= 1.1 && 
        channel_6 >= -1.1 && channel_6 <= 1.1 && 
        channel_7 >= -1.1 && channel_7 <= 1.1 && 
        channel_8 >= -1.1 && channel_8 <= 1.1)
    {
        // pass
    }
    else
    {
        ROS_ERROR("RC data validity check fail. channel_5=%f, channel_6=%f, channel_8=%f", channel_5, channel_6, channel_8);
    }
}
#endif