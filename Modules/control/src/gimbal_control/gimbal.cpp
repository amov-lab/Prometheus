#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include "prometheus_msgs/GimbalCtl.h"

#define Q10F_UP 1
#define Q10F_DOWN 2
#define Q10F_LEFT 3
#define Q10F_RIGHT 4
#define Q10F_STOP 5


//全局变量
serial::Serial serial_Q10f; //声明串口

uint8_t Q10FTxBuffer[256];
uint8_t Q10FTxnum = 0;

//按角度
uint8_t UpAng[9] = {0x81, 0x01, 0x0A, 0x01, 0x00, 0x00, 0x03, 0x01, 0xFF};    //81 01 0A 01 00 32 03 01 FF  俯仰角往上50度
uint8_t DownAng[9] = {0x81, 0x01, 0x0A, 0x01, 0x00, 0x00, 0x03, 0x02, 0xFF};  //81 01 0A 01 00 32 03 02 FF  俯仰角往下50度
uint8_t RightAng[9] = {0x81, 0x01, 0x0A, 0x01, 0x00, 0x00, 0x01, 0x03, 0xFF}; //81 01 0A 01 00 32 02 03 FF  偏航角往右50度
uint8_t LeftAng[9] = {0x81, 0x01, 0x0A, 0x01, 0x00, 0x00, 0x02, 0x03, 0xFF};  //81 01 0A 01 00 32 01 03 FF  偏航角往左50度
uint8_t StopAng[9] = {0x81, 0x01, 0x0A, 0x01, 0x00, 0x00, 0x03, 0x03, 0xFF};  //81 01 0A 01 00 32 03 03 FF  回到home

//按速度
//TODT
uint8_t UpSpeed[9] = {0x81, 0x01, 0x06, 0x01, 0x00, 0x10, 0x03, 0x01, 0xFF};    //81 01 0A 01 00 32 03 01 FF  俯仰角往上50度
uint8_t DownSpeed[9] = {0x81, 0x01, 0x06, 0x01, 0x00, 0x10, 0x03, 0x02, 0xFF};  //81 01 0A 01 00 32 03 02 FF  俯仰角往下50度
uint8_t RightSpeed[9] = {0x81, 0x01, 0x06, 0x01, 0x10, 0x00, 0x02, 0x03, 0xFF}; //81 01 0A 01 00 32 02 03 FF  偏航角往右50度
uint8_t LeftSpeed[9] = {0x81, 0x01, 0x06, 0x01, 0x10, 0x00, 0x03, 0x01, 0xFF};  //81 01 0A 01 00 32 01 03 FF  偏航角往左50度
uint8_t StopSpeed[9] = {0x81, 0x01, 0x06, 0x01, 0x00, 0x00, 0x03, 0x03, 0xFF};  //81 01 0A 01 00 32 03 03 FF  回到home

//相机变倍
uint8_t ZoomOut[7] = {0xff, 0x01, 0x00, 0x40, 0x00, 0x00, 0x41};
uint8_t ZoomIn[7] = {0xff, 0x01, 0x00, 0x20, 0x00, 0x00, 0x21};
uint8_t ZoomStop[6] = {0x81, 0x01, 0x04, 0x07, 0x00, 0xff};

//相机变倍的VISCA协议
uint8_t ZoomTable[30][9] = {
    {0x81, 0x01, 0x04, 0x47, 0x00, 0x00, 0x00, 0x00, 0xFF}, //1  00 00 00 00
    {0x81, 0x01, 0x04, 0x47, 0x01, 0x07, 0x0f, 0x0f, 0xFF}, //   01 07 0f 0f
    {0x81, 0x01, 0x04, 0x47, 0x02, 0x02, 0x02, 0x02, 0xFF}, //   02 02 02 02
    {0x81, 0x01, 0x04, 0x47, 0x02, 0x08, 0x03, 0x04, 0xFF}, //4  02 08 03 04
    {0x81, 0x01, 0x04, 0x47, 0x02, 0x0c, 0x09, 0x00, 0xFF}, //   02 0c 09 00
    {0x81, 0x01, 0x04, 0x47, 0x02, 0x0f, 0x0d, 0x08, 0xFF}, //   02 0f 0d 08
    {0x81, 0x01, 0x04, 0x47, 0x03, 0x02, 0x07, 0x0d, 0xFF}, //7  03 02 07 0d
    {0x81, 0x01, 0x04, 0x47, 0x03, 0x04, 0x0b, 0x0d, 0xFF}, //   03 04 0b 0d
    {0x81, 0x01, 0x04, 0x47, 0x03, 0x06, 0x0b, 0x03, 0xFF}, //   03 06 0b 03
    {0x81, 0x01, 0x04, 0x47, 0x03, 0x08, 0x05, 0x00, 0xFF}, //10 03 08 05 00
    {0x81, 0x01, 0x04, 0x47, 0x03, 0x09, 0x0a, 0x03, 0xFF}, //   03 09 0a 03
    {0x81, 0x01, 0x04, 0x47, 0x03, 0x0a, 0x0b, 0x07, 0xFF}, //   03 0a 0b 07
    {0x81, 0x01, 0x04, 0x47, 0x03, 0x0b, 0x0a, 0x05, 0xFF}, //13 03 0b 0a 05
    {0x81, 0x01, 0x04, 0x47, 0x03, 0x0c, 0x06, 0x0d, 0xFF}, //   03 0c 06 0d
    {0x81, 0x01, 0x04, 0x47, 0x03, 0x0d, 0x00, 0x04, 0xFF}, //   03 0d 00 04
    {0x81, 0x01, 0x04, 0x47, 0x03, 0x0d, 0x08, 0x01, 0xFF}, //16 03 0d 08 01
    {0x81, 0x01, 0x04, 0x47, 0x03, 0x0d, 0x0f, 0x02, 0xFF}, //   03 0d 0f 02
    {0x81, 0x01, 0x04, 0x47, 0x03, 0x0e, 0x04, 0x0a, 0xFF}, //   03 0e 04 0a
    {0x81, 0x01, 0x04, 0x47, 0x03, 0x0e, 0x09, 0x05, 0xFF}, //19 03 0e 09 05
    {0x81, 0x01, 0x04, 0x47, 0x03, 0x0e, 0x0e, 0x00, 0xFF}, //   03 0e 0e 00
    {0x81, 0x01, 0x04, 0x47, 0x03, 0x0f, 0x00, 0x06, 0xFF}, //   03 0f 00 06
    {0x81, 0x01, 0x04, 0x47, 0x03, 0x0f, 0x03, 0x08, 0xFF}, //22 03 0f 03 08
    {0x81, 0x01, 0x04, 0x47, 0x03, 0x0f, 0x05, 0x0e, 0xFF}, //   03 0f 05 0e
    {0x81, 0x01, 0x04, 0x47, 0x03, 0x0f, 0x08, 0x03, 0xFF}, //   03 0f 08 03
    {0x81, 0x01, 0x04, 0x47, 0x03, 0x0f, 0x0a, 0x09, 0xFF}, //25 03 0f 0a 09
    {0x81, 0x01, 0x04, 0x47, 0x03, 0x0f, 0x0c, 0x0e, 0xFF}, //   03 0f 0c 0e
    {0x81, 0x01, 0x04, 0x47, 0x03, 0x0f, 0x0e, 0x07, 0xFF}, //   03 0f 0e 07
    {0x81, 0x01, 0x04, 0x47, 0x04, 0x00, 0x00, 0x00, 0xFF}, //28 04 00 00 00
    {0x81, 0x01, 0x04, 0x47, 0x04, 0x00, 0x00, 0x00, 0xFF}, //   04 00 00 00
    {0x81, 0x01, 0x04, 0x47, 0x04, 0x00, 0x00, 0x00, 0xFF}, //30 04 00 00 00
};

//相机手动聚集
uint8_t focusOut[7] = {0xFF, 0x01, 0x00, 0x80, 0x00, 0x00, 0x81};  //获得焦点触发事件
uint8_t focusIn[7] = {0xFF, 0x01, 0x01, 0x00, 0x00, 0x00, 0x02};   //失去焦点触发事件
uint8_t focusStop[7] = {0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01}; //停止事件

//相机模式切换及拍照
uint8_t cameraModeChange[7] = {0xff, 0x01, 0x00, 0x07, 0x00, 0x67, 0x6f}; //切换拍照模式和录像模式
uint8_t takePicture[7] = {0xff, 0x01, 0x00, 0x07, 0x00, 0x55, 0x5d};      //在拍照模式下拍照，在录像模式下录像

//一键回中
uint8_t homePosition[9] = {0x81, 0x01, 0x0a, 0x01, 0x00, 0x00, 0x03, 0x03, 0xff}; //角度控制模式恢复到home点

//航向跟随控制
uint8_t cmd_follow_yaw_disable[11] = {0x3e, 0x1f, 0x06, 0x25, 0x01, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x20};
uint8_t cmd_follow_yaw_enable[11] = {0x3e, 0x1f, 0x06, 0x25, 0x01, 0x1f, 0x01, 0x00, 0x00, 0x00, 0x21};

//读取吊舱角度信息
uint8_t getAng[5] = {0x3e, 0x3d, 0x00, 0x3d, 0x00}; //获取相机的角度信息

bool Q10FInit(void)
{
    Q10FTxnum = 0;
    return 0;
}

uint8_t Check8Bit(uint8_t Input[], uint8_t num)
{
    uint8_t sum = 0;
    for (int i = 0; i < num; i++)
    {
        sum += Input[i];
    }
    return sum;
}

//云台姿态控制，角度控制
bool Q10FRotateSpeed(uint8_t type, uint8_t angle)
{
    if (angle > 180)
        angle = 180;
    if (type == Q10F_UP)
    {
        UpAng[5] = angle;
        serial_Q10f.write(UpAng, 9);
    }
    else if (type == Q10F_DOWN)
    {
        DownAng[5] = angle;
        serial_Q10f.write(DownAng, 9);
    }
    else if (type == Q10F_LEFT)
    {
        LeftAng[5] = angle;
        serial_Q10f.write(LeftAng, 9);
    }
    else if (type == Q10F_RIGHT)
    {
        RightAng[5] = angle;
        serial_Q10f.write(RightAng, 9);
    }
    else if (type == Q10F_STOP)
    {
        serial_Q10f.write(StopAng, 9);
    }
    else
    {
        return false;
    }
    // delay_us(2);
    return true;
}

//输入为Q10F相机的倍数，正常放大倍数为1~30倍
bool Q10FZoomTableCtrl(uint8_t multiple)
{
    if (multiple < 1)
        multiple = 1;
    if (multiple > 30)
        multiple = 30;
    multiple = multiple - 1;

    serial_Q10f.write(ZoomTable[multiple], 9);
    // delay_us(2);
    return true;
}

//放大倍数
bool Q10FCameraZoomIn(void)
{
    serial_Q10f.write(ZoomIn, 7);
    return true;
}

//缩小倍数
bool Q10FCameraZoomOut(void)
{
    serial_Q10f.write(ZoomOut, 7);
    return true;
}

//保持当前倍数
bool Q10FCameraZoomStop(void)
{
    serial_Q10f.write(ZoomStop, 6); //有问题注意
}

//模式切换
bool Q10FCameraModeChange(void)
{
    serial_Q10f.write(cameraModeChange, 7);
    return true;
}

//在照相模式中照相，在录制模式中开始/停止录制
bool Q10FTakePicture(void)
{
    serial_Q10f.write(takePicture, 7);
    return true;
}

//获得焦点
bool Q10FfocusOut(void)
{
    serial_Q10f.write(focusOut, 7);
    return true;
}

//失去焦点
bool Q10FfocusIn(void)
{
    serial_Q10f.write(focusIn, 7);
    return true;
}

//停止聚焦
bool Q10FfocusStop(void)
{
    serial_Q10f.write(focusStop, 7);
    return true;
}

//home点位置
bool Q10FHomePosition(void)
{
    serial_Q10f.write(homePosition, 9);
    return true;
}

//读取吊舱的角度信息
bool Q10FGetAng(void)
{
    serial_Q10f.write(getAng, 5);
    return true;
}

//启用航向跟随
bool Q10FHYawFollowEnable(void)
{
    serial_Q10f.write(cmd_follow_yaw_enable, 11);
    return true;
}

//取消航向跟随
bool Q10FHYawFollowDisable(void)
{
    serial_Q10f.write(cmd_follow_yaw_disable, 11);
    return true;
}

int q10f_pitch = 0;
int q10f_yaw = 0;
int q10f_zoom = 1;
int q10f_focus = 0;
int q10f_home = 0;
int q10f_TakePicture = 0;
int q10f_cameraModeChange = 0;
int q10f_yawfollow = 0;
int stop_zoom_flag = 0;
int stop_focus_flag = 0;
int take_picutre_2s_flag = 0;

//打印当前的控制状态
void display_q10f_status(void)
{
    std::cout << "pitch:" << q10f_pitch << "\tyaw:" << q10f_yaw << "\tzoom:" << q10f_zoom << "\tfocus:" << q10f_focus;
    std::cout << "\thome:" << q10f_home << "takePic:" << q10f_TakePicture << "modeC:" << q10f_cameraModeChange << "yawf:" << q10f_yawfollow << "\r\n";
}

//打印当前的实时数据状态

//
void chatterCallback(const prometheus_msgs::GimbalCtl::ConstPtr &msg)
{
    uint8_t temp;
    q10f_home = msg->home;
    q10f_TakePicture = msg->TakePicture;
    q10f_cameraModeChange = msg->cameraModeChange;

    //俯仰控制
    if (msg->pitch != 0)
    {
        q10f_pitch += msg->pitch;

        if (q10f_pitch > 60)
        {
            q10f_pitch = 60;
        }
        if (q10f_pitch < -90)
        {
            q10f_pitch = -90;
        }

        if (q10f_pitch > 0)
        {
            temp = (uint8_t)q10f_pitch;
            Q10FRotateSpeed(Q10F_UP, temp);
        }
        else
        {
            temp = (uint8_t)(-q10f_pitch);
            Q10FRotateSpeed(Q10F_DOWN, temp);
        }
        display_q10f_status();
        return;
    }

    //航向控制
    if (msg->yaw != 0)
    {
        q10f_yaw += msg->yaw;
        if (q10f_yaw > 140)
        {
            q10f_yaw = 140;
        }
        if (q10f_yaw < -140)
        {
            q10f_yaw = -140;
        }
        if (q10f_yaw > 0)
        {
            temp = (uint8_t)q10f_yaw;
            Q10FRotateSpeed(Q10F_RIGHT, temp);
        }
        else
        {
            temp = (uint8_t)(-q10f_yaw);
            Q10FRotateSpeed(Q10F_LEFT, temp);
        }
        display_q10f_status();
        return;
    }

    //变倍控制
    if (msg->zoom != 0)
    {
        q10f_zoom += msg->zoom;

        if (q10f_zoom > 10)
        {
            q10f_zoom = 10;
        }
        if (q10f_zoom < 1)
        {
            q10f_zoom = 1;
        }
        Q10FZoomTableCtrl(q10f_zoom);

        display_q10f_status();
        return;
    }

    //变焦控制
    if (msg->focus != 0)
    {
        q10f_focus += msg->focus;
        if (q10f_focus > 10)
        {
            q10f_focus = 10;
        }
        if (q10f_focus < 0)
        {
            q10f_focus = 0;
        }
        if (msg->focus > 0)
        {
            Q10FfocusIn();
        }
        else
        {
            Q10FfocusOut();
        }
        stop_focus_flag = 20; //20*10ms
        display_q10f_status();
        return;
    }

    //拍照录像模式的切换
    if (msg->cameraModeChange != 0)
    {
        Q10FCameraModeChange();
        display_q10f_status();
        return;
    }

    //回到home位置
    if (msg->home != 0)
    {
        q10f_pitch = 0;
        q10f_yaw = 0;
        Q10FHomePosition();
        display_q10f_status();
        return;
    }

    //航向跟随
    if (msg->yawfollow != 0)
    {
        if (q10f_yawfollow == 1)
        {
            Q10FHYawFollowDisable();
            q10f_yawfollow = 0;
        }
        else
        {
            Q10FHYawFollowEnable();
            q10f_yawfollow = 1;
        }
    }
    display_q10f_status();
}

//主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gimbal");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("GimbalCtl", 1000, chatterCallback);

    try
    {
        //串口设置
        serial_Q10f.setPort("/dev/ttyUSB0");
        serial_Q10f.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial_Q10f.setTimeout(to);
        serial_Q10f.open();
    }
    catch (const std::exception &e)
    {
        // std::cerr << e.what() << '\n';
        ROS_ERROR_STREAM("Unable to open Serial Port !");
        return -1;
    }
    if (serial_Q10f.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }
    Q10FHomePosition(); //初始化相机到homePosition
    Q10FGetAng();       //相机启动之后实时读取相机角度信息

    //设置循环的频率 100HZ 10ms 要求循环频率大于数据接受频率
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        if (stop_focus_flag > 0)
        {
            stop_focus_flag--;
            if (stop_focus_flag == 0)
                Q10FfocusStop();
        }
        static int loop_2s = 0;
        loop_2s++;
        if (loop_2s >= 200) //200*10ms = 2s
        {
            loop_2s = 0;
            if (take_picutre_2s_flag == 1) //拍照开关打开
            {
                Q10FTakePicture();
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
