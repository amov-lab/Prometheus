//gimbal tracking control & serial communication---advanced by Xuancen Liu -----------------------------------------
//2019.9.18 at Hunan Changsha.
// email: buaalxc@163.com
//wechat: liuxuancen003

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include "prometheus_msgs/GimbalTrackError.h"
#include "prometheus_msgs/gimbal.h"
#include <iomanip>
#include <fstream>
#include <thread>
#include <std_srvs/SetBool.h>

using namespace std;
serial::Serial ser;
prometheus_msgs::GimbalTrackError temp;
prometheus_msgs::gimbal gimbaldata;
unsigned char command[20];

float flag_target;
short int yaw_control_last, pitch_control_last;
bool is_land = false;

float imu_yaw_vel, imu_pitch_vel;
float get_ros_time(ros::Time begin);
//获取当前时间 单位：秒
float get_ros_time(ros::Time begin)
{
  ros::Time time_now = ros::Time::now();
  float currTimeSec = time_now.sec - begin.sec;
  float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
  return (currTimeSec + currTimenSec);
}

void pos_diff_callback(const prometheus_msgs::GimbalTrackError::ConstPtr &msg)
{

  // float pKp = 2.6, yKp = 2.6;
  // NOTE PID算法
  float pKp = 2, yKp = 2;
  float pKd = 0.2, yKd = 0.2;
  float pKI = 0.0, yKI = 0.0;

  unsigned char xl, xh, yl, yh;
  short int yaw_control, pitch_control;
  // unsigned char command[20];
  int tmp = 0;

  temp.x = msg->x; //* std::abs(msg->x);
  temp.y = msg->y; //* std::abs(msg->y);
  temp.velx = msg->velx;
  temp.vely = msg->vely;
  temp.Ix = msg->Ix;
  temp.Iy = msg->Iy;

  yaw_control = (short int)(yKp * temp.x + yKd * (temp.velx) + yKI * temp.Ix);
  yaw_control_last = yaw_control;
  if (yaw_control - 500.0 > 0.001)
  {
    yaw_control = 500;
  }
  if (yaw_control + 500.0 < -0.001)
  {
    yaw_control = -500;
  }
  // if (yaw_control > 0.001)
  // {
  //   yaw_control = yaw_control + 50.0;
  // }
  // if (yaw_control < 0.001)
  // {
  //   yaw_control = yaw_control - 50.0;
  // }
  // xh = (yaw_control & 0xff00) >> 8;
  // xl = (yaw_control & 0x00ff);

  pitch_control = (short int)(pKp * temp.y + pKd * (temp.vely) + pKI * temp.Iy);
  pitch_control_last = pitch_control;
  if (pitch_control - 500.0 > 0.001)
  {
    pitch_control = 500;
  }
  if (pitch_control + 500.0 < -0.001)
  {
    pitch_control = -500;
  }
  // if (pitch_control > 0.001)
  // {
  //   pitch_control = pitch_control + 50.0;
  // }
  // if (pitch_control < 0.001)
  // {
  //   pitch_control = pitch_control - 50.0;
  // }
  yh = (pitch_control & 0xff00) >> 8;
  yl = (pitch_control & 0x00ff);

  command[0] = 0xFF;
  command[1] = 0x01;
  command[2] = 0x0F;
  command[3] = 0x10;
  command[5] = 0x01;
  // Roll
  command[9] = 0x00;
  command[10] = 0x00;
  // Pitch
  command[13] = 0x00;
  command[14] = 0x00;
  // Yaw
  command[17] = 0x00;
  command[18] = 0x00;
  command[19] = 0;
  if (!msg->detected)
  {
    command[4] = 0x01;
    command[5] = 0x01;
    command[6] = 0x01;
    command[7] = 0x00;
    command[8] = 0x00;
    command[9] = 0x00;
    command[10] = 0x00;
    command[11] = 0x00;
    command[12] = 0x00;
    command[13] = 0x00;
    command[14] = 0x00;
    command[15] = 0x00;
    command[16] = 0x00;
    command[17] = 0x00;
    command[18] = 0x00;
  }
  else
  {
    if (is_land)
    {
      xh = (-yaw_control & 0xff00) >> 8;
      xl = (-yaw_control & 0x00ff);
      // 控制模式
      command[4] = 0x01;
      command[6] = 0x00;
      // roll
      command[7] = xl;
      command[8] = xh;
      // yaw
      command[15] = 0x00;
      command[16] = 0x00;
    }
    else
    {
      xh = (yaw_control & 0xff00) >> 8;
      xl = (yaw_control & 0x00ff);
      // 控制模式
      command[4] = 0x00;
      command[6] = 0x01;
      // roll
      command[7] = 0x00;
      command[8] = 0x00;
      // yaw
      command[15] = xl;
      command[16] = xh;
    }
    command[11] = yl;
    command[12] = yh;
  }
  for (int i = 4; i < 19; i++)
  {
    tmp += (int)command[i];
  }
  command[19] = tmp % 256;
  // for (int i = 0; i < 20; i++)
  // std::cout << std::hex << (int)command[i] << ' ';
  // std::cout << std::endl;
  // command = {0xff, 0x01, 0x0f, 0x10, 0x00, 0x05, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0x07, 0x00, 0x00, 0xE8, 0x03, 0xcc};

  /*
  if (!msg->detected)
  {
    //cout<<"============000000000000000000"<<endl;
    command[0] = 0x55;
    command[1] = 0x01;
    command[2] = 0x00;
    command[3] = 0x00;
    command[4] = 0x00;
    command[5] = 0x00;
    command[6] = 0x02;
    command[7] = 0x58;
  }
  else
  {
    //cout<<"============11111111111111111"<<endl;
    // TODO 无法使用
    if (is_land)
    {
      command[0] = 0x55;
      command[1] = 0x01;
      command[2] = 0;
      command[3] = 0;
    }
    else
    {
      command[0] = 0x55;
      command[1] = 0x01;
      command[2] = xl;
      command[3] = xh;
    }
    command[4] = yl;
    command[5] = yh;
    command[6] = 0x02;
    command[7] = (command[0] + command[1] + command[2] + command[3] + command[4] + command[5] + command[6]) % 256;
  }
  */
}

bool change_mode(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  is_land = req.data;
  std::cout << (is_land ? "into Land Mode" : "Exit Land Mode") << std::endl;
  return true;
}
//日志】数据保存
// ofstream file;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

int main(int argc, char **argv)
{

  //初始化节点
  ros::init(argc, argv, "Gimbal_Tracking");
  //声明节点句柄
  ros::NodeHandle nh;
  static int p;
  float dt;
  float imu_camera[3], imu_camera_last[3], imu_camera_vel[3];
  float last_time;
  float RC_target_camera[3], stator_rel_camera_last[3], stator_rel_camera_vel[3];
  float stator_rel_camera[3];

  /*
  注意，里面有iOS：：app，表示打开文件后，在写入的文件不会覆盖原文件中的内容，也就是原来文件中的数据会得到保存。
  file.open("/root/tmp/track_ws/serial_node.txt", ios::binary | ios::app | ios::in | ios::out);

  file << "cur_time"
       << "  "
       << "dt"
       << "  "
       << "imu_camera[0]"
       << "  "
       << "imu_camera[1]"
       << "  "
       << "imu_camera[2]"
       << "  "
       << "RC_target_camera[0]"
       << "  "
       << "RC_target_camera[1]"
       << "  "
       << "RC_target_camera[2]"
       << "  "
       << "stator_rel_camera[0]"
       << "  "
       << "stator_rel_camera[1]"
       << "  "
       << "stator_rel_camera[2]"
       << "  "
       << "imu_camera_vel[0]"
       << "  "
       << "imu_camera_vel[1]"
       << "  "
       << "imu_camera_vel[2]"
       << "  "
       << "stator_rel_camera_vel[0]"
       << "  "
       << "stator_rel_camera_vel[1]"
       << "  "
       << "stator_rel_camera_vel[2]"
       << "  "
       << "yaw_control_last"
       << "  "
       << "pitch_control_last"
       << "\n";
       */

  //    float pitch_imu;
  //    float pitch_rc;
  //    float pitch_stator_rel;
  std_msgs::UInt8MultiArray r_buffer;
  //订阅主题，并配置回调函数
  // 向串口写入获取数据指令
  // ros::Subscriber write_sub = nh.subscribe<pr::Cloud_platform>("write", 10, write_callback);
  // 控制吊舱中心对准
  ros::Subscriber pos_diff_sub = nh.subscribe<prometheus_msgs::GimbalTrackError>("/prometheus/object_detection/circelx_error", 1, pos_diff_callback);
  // 降落模式控制, 摄像头朝下, 使用roll, pitch控制, 防止子在yaw, pitch控制时的万向锁
  ros::ServiceServer server = nh.advertiseService("/prometheus/gimbal/is_land", change_mode);
  //发布主题: 吊舱欧拉角
  ros::Publisher read_pub = nh.advertise<prometheus_msgs::gimbal>("/readgimbal", 10);
  ros::Time begin_time = ros::Time::now();

  try
  {

    ser.setPort("/dev/ttyUSB0");
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  }
  catch (serial::IOException &e)
  {
    ROS_ERROR_STREAM("Unable to open port ");
    return -1;
  }

  if (ser.isOpen())
  {
    ROS_INFO_STREAM("Serial Port initialized");
  }
  else
  {
    return -1;
  }
  //指定循环的频率
  ros::Rate loop_rate(28);

  unsigned char info[5] = {0x3e, 0x3d, 0x00, 0x3d, 0x00};
  unsigned char header[4]{0x3e, 0x3d, 0x36, 0x73};

  ser.write(info, 5);
  while (ros::ok())
  {
    //cout<<"serial is not avaliable"<<endl;
    // ser.available -- Return the number of characters in the buffer

    if (ser.available())
    {
      float cur_time = get_ros_time(begin_time);
      dt = cur_time - last_time;
      last_time = cur_time;
      ser.write(command, 20);
      ser.write(info, 5);
      p = ser.available();
      std_msgs::UInt8MultiArray serial_data;

      ser.read(serial_data.data, p);
      r_buffer.data.resize(59 + 1);
      r_buffer.data[0] = p;

      // 计算机header所在位置
      if (p < 59)
        continue;
      int bias = 0;
      for (int j = 0; j < p - 55;)
      {
        bool find = true;
        for (int i = 0; i < 4; i++)
        {
          if (serial_data.data[j + i] != header[i])
          {
            j += i + 1;
            find = false;
            break;
          }
        }
        if (find == true)
        {
          bias = j;
          break;
        }
      }

      // 校验和验证
      int check_sum = 0;
      for (int i = bias + 4; i < bias + 57; i++)
      {
        check_sum += serial_data.data[i];
      }
      if (check_sum % 256 != serial_data.data[bias + 58])
        continue;

      // 解析数据
      for (int i = bias; i < bias + 59; i++)
      {
        r_buffer.data[i + 1 - bias] = serial_data.data[i];

        if (r_buffer.data[6] > 40)
        {
          imu_camera[0] = -0.02197 * (256 * (256 - r_buffer.data[6]) - r_buffer.data[5]);
        }
        else if (r_buffer.data[6] < 40)
        {
          imu_camera[0] = 0.02197 * (256 * r_buffer.data[6] + r_buffer.data[5]);
        }

        imu_camera_vel[0] = (imu_camera[0] - imu_camera_last[0]) / dt;
        imu_camera_last[0] = imu_camera[0];

        //printf("r_buffer.data[6] is %d \n",r_buffer.data[6]);
        // printf("r_buffer.data[5] is %d \n",r_buffer.data[5]);
        // printf("imu_camera[0] is %f \n",imu_camera[0]);

        if (r_buffer.data[8] > 40)
        {
          RC_target_camera[0] = -0.02197 * (256 * (256 - r_buffer.data[8]) - r_buffer.data[7]);
        }
        else if (r_buffer.data[8] < 40)
        {
          RC_target_camera[0] = 0.02197 * (256 * r_buffer.data[8] + r_buffer.data[7]);
        }

        if (r_buffer.data[10] > 40)
        {
          stator_rel_camera[0] = -0.02197 * (256 * (256 - r_buffer.data[10]) - r_buffer.data[9]);
        }
        else if (r_buffer.data[10] < 40)
        {
          stator_rel_camera[0] = 0.02197 * (256 * r_buffer.data[10] + r_buffer.data[9]);
        }
        //printf("stator_rel_camera[0] is %d \n",(stator_rel_camera[0]));

        //----------------------------------------俯仰PITCH-------------------------------------------
        if (r_buffer.data[24] > 40)
        {
          imu_camera[1] = -0.02197 * (256 * (256 - r_buffer.data[24]) - r_buffer.data[23]);
        }
        else if (r_buffer.data[24] < 40)
        {
          imu_camera[1] = 0.02197 * (256 * r_buffer.data[24] + r_buffer.data[23]);
        }

        imu_camera_vel[1] = (imu_camera[1] - imu_camera_last[1]) / dt;
        imu_camera_last[1] = imu_camera[1];

        if (r_buffer.data[26] > 40)
        {
          RC_target_camera[1] = -0.02197 * (256 * (256 - r_buffer.data[26]) - r_buffer.data[25]);
        }
        else if (r_buffer.data[26] < 40)
        {
          RC_target_camera[1] = 0.02197 * (256 * r_buffer.data[26] + r_buffer.data[25]);
        }

        if (r_buffer.data[28] > 40)
        {
          stator_rel_camera[1] = -0.02197 * (256 * (256 - r_buffer.data[28]) - r_buffer.data[27]);
        }
        else if (r_buffer.data[28] < 40)
        {
          stator_rel_camera[1] = 0.02197 * (256 * r_buffer.data[28] + r_buffer.data[27]);
        }

        //----------------------------------------偏航YAW-------------------------------------------

        if (r_buffer.data[42] > 40)
        {
          imu_camera[2] = -0.02197 * (256 * (256 - r_buffer.data[42]) - r_buffer.data[41]);
        }
        else if (r_buffer.data[42] < 40)
        {
          imu_camera[2] = 0.02197 * (256 * r_buffer.data[42] + r_buffer.data[41]);
        }

        imu_camera_vel[2] = (imu_camera[2] - imu_camera_last[2]) / dt;
        imu_camera_last[2] = imu_camera[2];

        //printf("imu_camera[1] is %d \n",(imu_camera[1]));

        if (r_buffer.data[44] > 40)
        {
          RC_target_camera[2] = -0.02197 * (256 * (256 - r_buffer.data[44]) - r_buffer.data[43]);
        }
        else if (r_buffer.data[44] < 40)
        {
          RC_target_camera[2] = 0.02197 * (256 * r_buffer.data[44] + r_buffer.data[43]);
        }
        //printf("RC_target_camera[1] is %d \n",(RC_target_camera[1]));

        if (r_buffer.data[46] > 40)
        {
          stator_rel_camera[2] = -0.02197 * (256 * (256 - r_buffer.data[46]) - r_buffer.data[45]);
        }
        else if (r_buffer.data[46] < 40)
        {
          stator_rel_camera[2] = 0.02197 * (256 * r_buffer.data[46] + r_buffer.data[45]);
        }
        //printf("stator_rel_camera[1] is %d \n",(stator_rel_camera[1]));
      }

      imu_camera_vel[0] = (imu_camera[0] - imu_camera_last[0]) / dt;
      imu_camera_last[0] = imu_camera[0];

      stator_rel_camera_vel[0] = (stator_rel_camera[0] - stator_rel_camera_last[0]) / dt;
      stator_rel_camera_last[0] = stator_rel_camera[0];

      imu_camera_vel[1] = (imu_camera[1] - imu_camera_last[1]) / dt;
      imu_camera_last[1] = imu_camera[1];

      stator_rel_camera_vel[1] = (stator_rel_camera[1] - stator_rel_camera_last[1]) / dt;
      stator_rel_camera_last[1] = stator_rel_camera[1];

      imu_camera_vel[2] = (imu_camera[2] - imu_camera_last[2]) / dt;
      imu_camera_last[2] = imu_camera[2];

      stator_rel_camera_vel[2] = (stator_rel_camera[2] - stator_rel_camera_last[2]) / dt;
      stator_rel_camera_last[2] = stator_rel_camera[2];

      gimbaldata.imu0 = imu_camera[0];
      gimbaldata.imu1 = imu_camera[1];
      gimbaldata.imu2 = imu_camera[2];
      gimbaldata.rc0 = RC_target_camera[0];
      gimbaldata.rc1 = RC_target_camera[1];
      gimbaldata.rc2 = RC_target_camera[2];
      gimbaldata.rel0 = stator_rel_camera[0];
      gimbaldata.rel1 = stator_rel_camera[1];
      gimbaldata.rel2 = stator_rel_camera[2];
      gimbaldata.imuvel0 = imu_camera_vel[0];
      gimbaldata.imuvel1 = imu_camera_vel[1];
      gimbaldata.imuvel2 = imu_camera_vel[2];
      gimbaldata.relvel0 = stator_rel_camera_vel[0];
      gimbaldata.relvel1 = stator_rel_camera_vel[1];
      gimbaldata.relvel2 = stator_rel_camera_vel[2];

      std::cout << "-----------------imu_camera----------------------" << endl;
      std::cout << "rel_roll:  " << setprecision(6) << stator_rel_camera[0] << std::endl;
      std::cout << "rel_pitch: " << setprecision(6) << stator_rel_camera[1] << std::endl;
      std::cout << "rel_yaw:   " << setprecision(6) << stator_rel_camera[2] << std::endl;
      std::cout << "relvel0:   " << setprecision(6) << gimbaldata.relvel0 << std::endl;
      std::cout << "relvel1:   " << setprecision(6) << gimbaldata.relvel1 << std::endl;
      std::cout << "relvel2:   " << setprecision(6) << gimbaldata.relvel2 << std::endl;

      // file << cur_time << "  " << dt << "  " << imu_camera[0] << "  " << imu_camera[1] << "  " << imu_camera[2] << "  " << RC_target_camera[0] << "  " << RC_target_camera[1] << "  " << RC_target_camera[2] << "  " << stator_rel_camera[0] << "  " << stator_rel_camera[1] << "  " << stator_rel_camera[2] << "  " << imu_camera_vel[0] << "  " << imu_camera_vel[1] << "  " << imu_camera_vel[2] << "  " << stator_rel_camera_vel[0] << "  " << stator_rel_camera_vel[1] << "  " << stator_rel_camera_vel[2] << "  " << yaw_control_last << "  " << pitch_control_last << "\n";
      // 此处数据需要完完成分类、创建msg.h、publish出去
      read_pub.publish(gimbaldata);
    }
    //处理ROS的信息，比如订阅消息,并调用回调函数
    ros::spinOnce();
    loop_rate.sleep();
  }
  // file.close(); //关闭文件，保存文件。
}
