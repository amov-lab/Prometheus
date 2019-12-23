/***************************************************************************************************************************
 * collision_avoidance.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2019.4.17
 *
 * 说明: 避障程序
 *
 *
 * 初版，需要遵循固定场景，具体场景设置请参看教程文件
 * 具体功能待完善
***************************************************************************************************************************/
//ROS 头文件
#include <ros/ros.h>
#include <command_to_mavros.h>
//topic 头文件
#include <iostream>
#include <prometheus_msgs/ControlCommand.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <prometheus_msgs/DroneState.h>

/*
 * 主要功能:
 * 1.获取激光雷达数据
 * 2.根据距离判断是否启用避障策略
 * 3.如启用避障策略,产生控制指令
 *
 */

using namespace std;
 
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define RAD2DEG(x) ((x)*180./M_PI)
//--------------------------------------------输入--------------------------------------------------
sensor_msgs::LaserScan Laser;                                   //激光雷达点云数据
geometry_msgs::Pose pos_drone;                                  //无人机当前位置
float target_x;                                                 //期望位置_x
float target_y;                                                 //期望位置_y

int range_min;                                                //激光雷达探测范围 最小角度
int range_max;                                                //激光雷达探测范围 最大角度

//--------------------------------------------算法相关--------------------------------------------------
float R_outside,R_inside;                                       //安全半径 [避障算法相关参数]
float p_R;                                                      //大圈比例参数
float p_r;                                                      //小圈比例参数

float distance_c,angle_c;                                       //最近障碍物距离 角度
float distance_cx,distance_cy;                                  //最近障碍物距离XY
float vel_collision[2];                                         //躲避障碍部分速度
float vel_collision_max;                                        //躲避障碍部分速度限幅

float p_xy;                                                     //追踪部分位置环P
float vel_track[2];                                             //追踪部分速度
float vel_track_max;                                            //追踪部分速度限幅
int flag_land;                                                  //降落标志位
//--------------------------------------------输出--------------------------------------------------
std_msgs::Bool flag_collision_avoidance;                       //是否进入避障模式标志位
float vel_sp_body[2];                                           //总速度
float vel_sp_max;                                               //总速度限幅
prometheus_msgs::ControlCommand Command_Now;                               //发送给position_control.cpp的命令
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void cal_min_distance();
float satfunc(float data, float Max);
void printf();                                                                       //打印函数
void printf_param();                                                                 //打印各项参数以供检查
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//接收雷达的数据，并做相应处理,然后计算前后左右四向最小距离
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    Laser = *scan;

    int count;    //count = 359 or 358
    count = Laser.scan_time / Laser.time_increment;

    //-179°到180°
    //cout << "Angle_range : "<< RAD2DEG(Laser.angle_min) << " to " << RAD2DEG(Laser.angle_max) <<endl;

    //剔除inf的情况
    for(int i = 0; i <= count; i++)
    {
        //判断是否为inf
        int a = isinf(Laser.ranges[i]);

        //如果为inf，则赋值上一角度的值
        if(a == 1)
        {
            if(i == 0)
            {
                Laser.ranges[i] = Laser.ranges[count];
            }
            else
            {
                Laser.ranges[i] = Laser.ranges[i-1];
            }
        }
    }

    //计算前后左右四向最小距离
    cal_min_distance();

}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    prometheus_msgs::DroneState state;
    pos_drone.position.x = state.position[0];
    pos_drone.position.y = state.position[1];
    pos_drone.position.z = state.position[2];
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_avoidance");
    ros::NodeHandle nh("~");

    // 频率 [20Hz]
    ros::Rate rate(20.0);

    //【订阅】Lidar数据
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_cb);

    //【订阅】无人机当前状态
    // 本话题来自根据需求自定px4_pos_estimator.cpp
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus_msgs/drone_state", 10, drone_state_cb);

    // 【发布】发送给position_control.cpp的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus_msgs/control_command", 10);

    //读取参数表中的参数
    nh.param<float>("target_x", target_x, 1.0);
    nh.param<float>("target_y", target_y, 1.0);

    nh.param<float>("R_outside", R_outside, 2);
    nh.param<float>("R_inside", R_inside, 1);

    nh.param<float>("p_xy", p_xy, 0.5);

    nh.param<float>("vel_track_max", vel_track_max, 0.5);

    nh.param<float>("p_R", p_R, 0.0);
    nh.param<float>("p_r", p_r, 0.0);

    nh.param<float>("vel_collision_max", vel_collision_max, 0.0);
    nh.param<float>("vel_sp_max", vel_sp_max, 0.0);

    nh.param<int>("range_min", range_min, 0.0);
    nh.param<int>("range_max", range_max, 0.0);


    //打印现实检查参数
    printf_param();

    int check_flag;
    //输入1,继续，其他，退出程序
    cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
    cin >> check_flag;

    if(check_flag != 1)
    {
        return -1;
    }

    //初值
    vel_track[0]= 0;
    vel_track[1]= 0;

    vel_collision[0]= 0;
    vel_collision[1]= 0;

    vel_sp_body[0]= 0;
    vel_sp_body[1]= 0;

    //四向最小距离 初值
    flag_land = 0;

    //输出指令初始化
    int comid = 1;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        //回调一次 更新传感器状态
        //1. 更新雷达点云数据，存储在Laser中,并计算四向最小距离
        ros::spinOnce();

        //2. 根据最小距离判断：是否启用避障策略
        if (distance_c >= R_outside )
        {
            flag_collision_avoidance.data = false;
        }
        else
        {
            flag_collision_avoidance.data = true;
        }

        //3. 计算追踪速度
        vel_track[0] = p_xy * (target_x - pos_drone.position.x);
        vel_track[1] = p_xy * (target_y - pos_drone.position.y);

        //速度限幅
        for (int i = 0; i < 2; i++)
        {
            vel_track[i] = satfunc(vel_track[i],vel_track_max);
        }
        vel_collision[0]= 0;
        vel_collision[1]= 0;

        //4. 避障策略
        if(flag_collision_avoidance.data == true)
        {
            distance_cx = distance_c * cos(angle_c/180*3.1415926);
            distance_cy = distance_c * sin(angle_c/180*3.1415926);

            distance_cx = - distance_cx;

            float F_c;

            F_c = 0;

            if(distance_c > R_outside)
            {
                //对速度不做限制
                vel_collision[0] = vel_collision[0] + 0;
                vel_collision[1] = vel_collision[1] + 0;
                cout << " Forward Outside "<<endl;
            }

            //小幅度抑制移动速度
            if(distance_c > R_inside && distance_c <= R_outside)
            {
                F_c = p_R * (R_outside - distance_c);

            }

            //大幅度抑制移动速度
            if(distance_c <= R_inside )
            {
                F_c = p_R * (R_outside - R_inside) + p_r * (R_inside - distance_c);
            }

            if(distance_cx > 0)
            {
                vel_collision[0] = vel_collision[0] - F_c * distance_cx /distance_c;
            }else{
                vel_collision[0] = vel_collision[0] - F_c * distance_cx /distance_c;
            }

            if(distance_cy > 0)
            {
                vel_collision[1] = vel_collision[1] - F_c * distance_cy / distance_c;
            }else{
                vel_collision[1] = vel_collision[1] - F_c * distance_cy /distance_c;
            }


            //避障速度限幅
            for (int i = 0; i < 2; i++)
            {
                vel_collision[i] = satfunc(vel_collision[i],vel_collision_max);
            }
        }

        vel_sp_body[0] = vel_track[0] + vel_collision[0];
        vel_sp_body[1] = vel_track[1] + vel_collision[1];

        for (int i = 0; i < 2; i++)
        {
            vel_sp_body[i] = satfunc(vel_sp_body[i],vel_sp_max);
        }

        //5. 发布Command指令给position_controller.cpp
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = command_to_mavros::Move_Body;     //机体系下移动
        Command_Now.Command_ID = comid;
        comid++;
        Command_Now.Reference_State.Sub_mode  = command_to_mavros::XY_VEL_Z_POS; // xy 速度控制模式 z 位置控制模式
        Command_Now.Reference_State.velocity_ref[0] =  vel_sp_body[0];
        Command_Now.Reference_State.velocity_ref[1] =  - vel_sp_body[1];  //ENU frame
        Command_Now.Reference_State.position_ref[2] =  0;
        Command_Now.Reference_State.yaw_ref = 0 ;

        float abs_distance;
        abs_distance = sqrt((pos_drone.position.x - target_x) * (pos_drone.position.x - target_x) + (pos_drone.position.y - target_y) * (pos_drone.position.y - target_y));

        if(abs_distance < 0.3 || flag_land == 1)
        {
            Command_Now.Mode = 3;     //Land
            flag_land = 1;
        }

        command_pub.publish(Command_Now);

        //打印
        printf();

        rate.sleep();

    }

    return 0;

}


//计算前后左右四向最小距离
void cal_min_distance()
{

    distance_c = Laser.ranges[range_min];
    angle_c = 0;
    for (int i = range_min; i <= range_max; i++)
    {
        if(Laser.ranges[i] < distance_c)
        {
            distance_c = Laser.ranges[i];
            angle_c = i;
        }
    }


}


//饱和函数
float satfunc(float data, float Max)
{
    if(abs(data)>Max)
    {
        return ( data > 0 ) ? Max : -Max;
    }
    else
    {
        return data;
    }
}


void printf()
{

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>collision_avoidance<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "Minimun_distance : "<<endl;
    cout << "Distance : " << distance_c << " [m] "<<endl;
    cout << "Angle :    " << angle_c    << " [du] "<<endl;
    cout << "distance_cx :    " << distance_cx    << " [m] "<<endl;
    cout << "distance_cy :    " << distance_cy    << " [m] "<<endl;


    if(flag_collision_avoidance.data == true)
    {
        cout << "Collision avoidance Enabled "<<endl;
    }
    else
    {
        cout << "Collision avoidance Disabled "<<endl;
    }

    cout << "vel_track_x : " << vel_track[0] << " [m/s] "<<endl;
    cout << "vel_track_y : " << vel_track[1] << " [m/s] "<<endl;

    cout << "vel_collision_x : " << vel_collision[0] << " [m/s] "<<endl;
    cout << "vel_collision_y : " << vel_collision[1] << " [m/s] "<<endl;

    cout << "vel_sp_x : " << vel_sp_body[0] << " [m/s] "<<endl;
    cout << "vel_sp_y : " << vel_sp_body[1] << " [m/s] "<<endl;

}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "target_x : "<< target_x << endl;
    cout << "target_y : "<< target_y << endl;

    cout << "R_outside : "<< R_outside << endl;
    cout << "R_inside : "<< R_inside << endl;

    cout << "p_xy : "<< p_xy << endl;
    cout << "vel_track_max : "<< vel_track_max << endl;

    cout << "p_R : "<< p_R << endl;
    cout << "p_r : "<< p_r << endl;

    cout << "vel_collision_max : "<< vel_collision_max << endl;

    cout << "vel_sp_max : "<< vel_sp_max << endl;
    cout << "range_min : "<< range_min << endl;
    cout << "range_max : "<< range_max << endl;


}


