#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include "printf_utils.h"

class RC_Input
{
public:
    RC_Input(){};
    void init();
    void handle_rc_data(mavros_msgs::RCInConstPtr pMsg);
    void printf_info();

    double ch[4];     // 1-4通道数值: roll pitch yaw thrust
    double channel_5; // 通道5（三段） - 切换RC_POS_CONTROL
    double channel_6; // 通道6（三段） - 切换COMMAND_CONTROL
    double channel_7; // 通道7（二段） - 切换LAND_CONTROL
    double channel_8; // 通道8（二段） - 重启飞控
    double channel_9; // 通道9（二段） - 解锁
    double last_channel_5;
    double last_channel_6;
    double last_channel_7;
    double last_channel_8;
    bool have_init_channel_5{false};
    bool have_init_channel_6{false};
    bool have_init_channel_7{false};
    bool have_init_channel_8{false};

    mavros_msgs::RCIn msg;
    ros::Time rcv_stamp; // 收到遥控器消息的时间

    bool enter_init;
    bool in_init;
    bool enter_rc_pos_control;
    bool in_rc_pos_control;
    bool enter_command_control;
    bool in_command_control;

    bool toggle_arm, toggle_disarm;
    bool toggle_kill;
    bool toggle_reboot;
    bool toggle_land;

private:
    static constexpr double channel_5_threshold_value_1 = 0.25;
    static constexpr double channel_5_threshold_value_2 = 0.75;
    static constexpr double channel_6_threshold_value_1 = 0.25;
    static constexpr double channel_6_threshold_value_2 = 0.75;
    static constexpr double channel_7_threshold_value_1 = 0.25;
    static constexpr double channel_8_threshold_value = 0.75;
    static constexpr double DEAD_ZONE = 0.05;                 // 死区

    void check_validity();
};

void RC_Input::init()
{
    rcv_stamp = ros::Time(0);

    last_channel_5 = -1.0;
    last_channel_6 = -1.0;
    last_channel_7 = -1.0;
    last_channel_8 = -1.0;

    toggle_arm = false;
    toggle_disarm = false;
    toggle_kill = false;
    toggle_reboot = false;
    toggle_land = false;

    enter_init = false;
    in_init = false;
    in_command_control = false;
    enter_command_control = false;
    in_rc_pos_control = false;
    enter_rc_pos_control = false;
}

void RC_Input::printf_info()
{
    cout << GREEN << ">>>>>>>>>>>>>>>>>> RC INFO  <<<<<<<<<<<<<<<<<<" << TAIL << endl;
    //固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为n位
    cout << setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout << GREEN << "ch1 : " << ch[0] << TAIL << endl;
    cout << GREEN << "ch2 : " << ch[1] << TAIL << endl;
    cout << GREEN << "ch3 : " << ch[2] << TAIL << endl;
    cout << GREEN << "ch4 : " << ch[3] << TAIL << endl;
    cout << GREEN << "ch5 : " << channel_5 << "  last_ch5 : " << last_channel_5 << TAIL << endl;
    cout << GREEN << "ch6 : " << channel_6 << "  last_ch6 : " << last_channel_6 << TAIL << endl;
    cout << GREEN << "ch7 : " << channel_7 << "  last_ch7 : " << last_channel_7 << TAIL << endl;
    cout << GREEN << "ch8 : " << channel_8 << "  last_ch8 : " << last_channel_8 << TAIL << endl;
    cout << GREEN << "toggle_arm : " << toggle_arm << TAIL << endl;
    cout << GREEN << "toggle_disarm : " << toggle_disarm << TAIL << endl;
    cout << GREEN << "toggle_kill : " << toggle_kill << TAIL << endl;
    cout << GREEN << "toggle_land : " << toggle_land << TAIL << endl;
    cout << GREEN << "toggle_reboot : " << toggle_reboot << TAIL << endl;
    cout << GREEN << "enter_init : " << enter_init << TAIL << endl;
    cout << GREEN << "in_init : " << in_init << TAIL << endl;
    cout << GREEN << "enter_rc_pos_control : " << enter_rc_pos_control << TAIL << endl;
    cout << GREEN << "in_rc_pos_control : " << in_rc_pos_control << TAIL << endl;
    cout << GREEN << "enter_command_control : " << enter_command_control << TAIL << endl;
    cout << GREEN << "in_command_control : " << in_command_control << TAIL << endl; 
}

void RC_Input::handle_rc_data(mavros_msgs::RCInConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    if( msg.channels[0]< 1100 && msg.channels[1]<1100 && msg.channels[2]< 1100 && msg.channels[3]>1900)
    {
        toggle_reboot = true;
        return;
    }else
    {
        toggle_reboot = false;
    }

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

    // 5通道，映射至[0,1]区间
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

    // 判断通道5 - channel_5_threshold_value_1 （0.25） channel_5_threshold_value_2 (0.75)
    // 目前真机中会出现处于中间位置会存在1500的输出值,相当于三段档杆,因此逻辑处理不能以两段档杆来处理
    if (last_channel_5 < channel_5_threshold_value_2 && channel_5 > channel_5_threshold_value_2)
    {
        // 由上往下拨一次
        toggle_arm = true;
        toggle_disarm = false;
    }
    else if (last_channel_5 > channel_5_threshold_value_1 && channel_5 < channel_5_threshold_value_1)
    {
        // 由下往上拨一次
        toggle_arm = false;
        toggle_disarm = true;
    }else
    {
        toggle_arm = false;
        toggle_disarm = false;
    }

    // 判断通道6 - channel_6_threshold_value_1 （0.25） channel_6_threshold_value_2 （0.75）
    if (channel_6 != last_channel_6 && channel_6 <= channel_6_threshold_value_1)
    {
        enter_init = true;
        enter_rc_pos_control = false;
        enter_command_control = false;
    }
    else if (channel_6 != last_channel_6 && (channel_6 > channel_6_threshold_value_1 && channel_6 < channel_6_threshold_value_2))
    {
        enter_init = false;
        enter_rc_pos_control = true;
        enter_command_control = false;
    }else if (channel_6 != last_channel_6 && channel_6 >= channel_6_threshold_value_2)
    {
        enter_init = false;
        enter_rc_pos_control = false;   
        enter_command_control = true;
    }

    // 判断通道7 - channel_7_threshold_value_1 （0.25）
    if (last_channel_7 < channel_7_threshold_value_1 && channel_7 > channel_7_threshold_value_1)
    {
        toggle_kill = true;
    }else
    {
        toggle_kill = false;
    }

    // 判断通道8 - channel_8_threshold_value （0.75） 
    // 目前真机中会出现处于中间位置会存在1500的输出值,阈值设置为0.5会出现异常
    if (last_channel_8 < channel_8_threshold_value && channel_8 > channel_8_threshold_value)
    {
        toggle_land = true;
    }else
    {
        toggle_land = false;
    }

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
