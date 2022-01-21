//ros头文件
#include <ros/ros.h>
#include <iostream>
#include <mission_utils.h>

using namespace std;

#define START_POINT_X -6.5
#define START_POINT_Y 0.0
#define START_POINT_Z 1.7
#define START_POINT_YAW 0.0

#define CIRCLE_POINT_X -3.5
#define CIRCLE_POINT_Y 0.0
#define CIRCLE_POINT_Z 1.8
#define CIRCLE_POINT_YAW 0.0

#define PILLAR_POINT_X 1.5
#define PILLAR_POINT_Y 0.0
#define PILLAR_POINT_Z 1.8
#define PILLAR_POINT_YAW 0.0

#define CORRIDOR_POINT_X 9.5
#define CORRIDOR_POINT_Y -1.0
#define CORRIDOR_POINT_Z 1.8
#define CORRIDOR_POINT_YAW 0.0

#define NUM_POINT_X 17
#define NUM_POINT_Y 0.0
#define NUM_POINT_Z 1.8
#define NUM_POINT_YAW 0.0
#define FOLLOWING_VEL 0.5
#define FOLLOWING_KP 2.0

#define LAND_POINT_X 23
#define LAND_POINT_Y 0.0
#define LAND_POINT_Z 1.8
#define LAND_POINT_YAW 0.0
#define LANDPAD_HEIGHT 0.0

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
Eigen::Vector3f start_point;
//椭圆穿越
Detection_result ellipse_det;
//避障任务
geometry_msgs::Point desired_vel;  
int flag_get_cmd = 0;
int flag_get_cmd_a = 0;
//走廊穿越
struct global_planner
{
    // 规划路径
    nav_msgs::Path path_cmd;
    int Num_total_wp;
    int wp_id;  
    int start_id;
}A_star;

//数字识别+颜色巡线
int flag_detection;
float error_body_y;
float yaw_sp;
//自主降落
Detection_result landpad_det;
Eigen::Vector3f pos_des_prev;

float kpx_land,kpy_land,kpz_land;                                                 //控制参数 - 比例参数

//无人机状态
prometheus_msgs::DroneState _DroneState;                                   //无人机状态量
Eigen::Matrix3f R_Body_to_ENU;

prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
ros::Publisher command_pub,goal_pub;
ros::Publisher local_planner_switch_pub,global_planner_switch_pub,circle_switch_pub, num_det_switch_pub, color_det_switch_pub, pad_det_switch_pub;

std_msgs::Bool switch_on;
std_msgs::Bool switch_off;

// 状态机
int State_Machine = 0;
float kpx_circle_track,kpy_circle_track,kpz_circle_track;                   //控制参数 - 比例参数
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void A_star_planner();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void ellipse_det_cb(const prometheus_msgs::DetectionInfo::ConstPtr& msg)
{
    ellipse_det.object_name = "circle";
    ellipse_det.Detection_info = *msg;
    ellipse_det.pos_body_frame[0] =   ellipse_det.Detection_info.position[2] + FRONT_CAMERA_OFFSET_X;
    ellipse_det.pos_body_frame[1] = - ellipse_det.Detection_info.position[0] + FRONT_CAMERA_OFFSET_Y;
    ellipse_det.pos_body_frame[2] = - ellipse_det.Detection_info.position[1] + FRONT_CAMERA_OFFSET_Z;

    ellipse_det.pos_body_enu_frame = R_Body_to_ENU * ellipse_det.pos_body_frame;

    if(ellipse_det.Detection_info.detected)
    {
        ellipse_det.num_regain++;
        ellipse_det.num_lost = 0;
    }else
    {
        ellipse_det.num_regain = 0;
        ellipse_det.num_lost++;
    }

    // 当连续一段时间无法检测到目标时，认定目标丢失
    if(ellipse_det.num_lost > VISION_THRES)
    {
        ellipse_det.is_detected = false;
    }

    // 当连续一段时间检测到目标时，认定目标得到
    if(ellipse_det.num_regain > VISION_THRES)
    {
        ellipse_det.is_detected = true;
    }
}
void local_planner_cmd_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    flag_get_cmd = 1;
    desired_vel = *msg;
}
void global_planner_cmd_cb(const nav_msgs::Path::ConstPtr& msg)
{
    flag_get_cmd_a = flag_get_cmd_a + 1;
    A_star.path_cmd = *msg;
    A_star.Num_total_wp = A_star.path_cmd.poses.size();

    //选择与当前无人机所在位置最近的点,并从该点开始追踪
    A_star.start_id = 0;
    float distance_to_wp_min = abs(A_star.path_cmd.poses[0].pose.position.x - _DroneState.position[0])
                                + abs(A_star.path_cmd.poses[0].pose.position.y - _DroneState.position[1]);
    for (int j=1;j<A_star.Num_total_wp;j++)
    {
        float distance_to_wp = abs(A_star.path_cmd.poses[j].pose.position.x - _DroneState.position[0])
                                + abs(A_star.path_cmd.poses[j].pose.position.y - _DroneState.position[1]);
        if(distance_to_wp < distance_to_wp_min)
        {
            distance_to_wp_min = distance_to_wp;
            A_star.start_id = j;
        }
    }

    //这里增大开始路径点是为了解决当得到新路径时,无人机会回头的问题
    A_star.wp_id = A_star.start_id + 1;
    if(A_star.Num_total_wp - A_star.start_id > 8)
    {
        A_star.wp_id = A_star.start_id + 7;
    }
}
void color_line_cb(const geometry_msgs::Pose::ConstPtr &msg)
{
    error_body_y = - tan(msg->position.x) * NUM_POINT_Z;
    flag_detection = msg->position.y;

    float x1 = msg->orientation.w;
    float y1 = msg->orientation.x;
    float x2 = msg->orientation.y;
    float y2 = msg->orientation.z;

    float next_desired_yaw = - atan2(y2 - y1, x2 - x1);

    yaw_sp = (0.7*yaw_sp + 0.3*next_desired_yaw);
}
void landpad_det_cb(const prometheus_msgs::DetectionInfo::ConstPtr &msg)
{
    landpad_det.object_name = "landpad";
    landpad_det.Detection_info = *msg;
    landpad_det.pos_body_frame[0] = - landpad_det.Detection_info.position[1] + DOWN_CAMERA_OFFSET_X;
    landpad_det.pos_body_frame[1] = - landpad_det.Detection_info.position[0] + DOWN_CAMERA_OFFSET_Y;
    landpad_det.pos_body_frame[2] = - landpad_det.Detection_info.position[2] + DOWN_CAMERA_OFFSET_Z;

    landpad_det.pos_body_enu_frame = R_Body_to_ENU * landpad_det.pos_body_frame;

    //若已知降落板高度，则无需使用深度信息。
    landpad_det.pos_body_enu_frame[2] = LANDPAD_HEIGHT - _DroneState.position[2];

    landpad_det.pos_enu_frame[0] = _DroneState.position[0] + landpad_det.pos_body_enu_frame[0];
    landpad_det.pos_enu_frame[1] = _DroneState.position[1] + landpad_det.pos_body_enu_frame[1];
    landpad_det.pos_enu_frame[2] = _DroneState.position[2] + landpad_det.pos_body_enu_frame[2];
    // landpad_det.att_enu_frame[2] = _DroneState.attitude[2] + Detection_raw.attitude[2];
    landpad_det.att_enu_frame[2] = 0.0;

    if(landpad_det.Detection_info.detected)
    {
        landpad_det.num_regain++;
        landpad_det.num_lost = 0;
    }else
    {
        landpad_det.num_regain = 0;
        landpad_det.num_lost++;
    }

    // 当连续一段时间无法检测到目标时，认定目标丢失
    if(landpad_det.num_lost > VISION_THRES)
    {
        landpad_det.is_detected = false;
    }

    // 当连续一段时间检测到目标时，认定目标得到
    if(landpad_det.num_regain > VISION_THRES)
    {
        landpad_det.is_detected = true;
    }

}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;
    R_Body_to_ENU = get_rotation_matrix(_DroneState.attitude[0], _DroneState.attitude[1], _DroneState.attitude[2]);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "indoor_competition");
    ros::NodeHandle nh("~");
    
    //【订阅】椭圆识别结果,用于形状穿越
    ros::Subscriber ellipse_det_sub = nh.subscribe<prometheus_msgs::DetectionInfo>("/prometheus/object_detection/ellipse_det", 10, ellipse_det_cb);

    ros::Subscriber landpad_det_sub = nh.subscribe<prometheus_msgs::DetectionInfo>("/prometheus/object_detection/landpad_det", 10, landpad_det_cb);

    //ros::Subscriber num_det_sub = nh.subscribe<prometheus_msgs::MultiDetectionInfo>("/prometheus/object_detection/num_det", 10, num_det_cb);

    ros::Subscriber color_line_sub = nh.subscribe<geometry_msgs::Pose>("/prometheus/object_detection/color_line_angle", 10, color_line_cb);

    //【订阅】局部路径规划结果,用于避开障碍物 柱子
    ros::Subscriber local_planner_sub  =  nh.subscribe<geometry_msgs::Point>("/prometheus/local_planner/desired_vel", 50, local_planner_cmd_cb);

    //【订阅】全局路径规划结果,用于避开障碍物 走廊
    ros::Subscriber global_planner_sub =   nh.subscribe<nav_msgs::Path>("/prometheus/global_planner/path_cmd", 50, global_planner_cmd_cb);

    //【订阅】无人机当前状态
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);
    
    // 【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/prometheus/planning/goal", 10);

    // 为了避免同时运行过多程序导致电脑奔溃，设置程序运行/休眠 开关，但这个功能尚未完全启用
    local_planner_switch_pub = nh.advertise<std_msgs::Bool>("/prometheus/switch/local_planner", 10);
    global_planner_switch_pub = nh.advertise<std_msgs::Bool>("/prometheus/switch/global_planner", 10);
    circle_switch_pub = nh.advertise<std_msgs::Bool>("/prometheus/switch/circle_crossing", 10);
    num_det_switch_pub = nh.advertise<std_msgs::Bool>("/prometheus/switch/num_det", 10);
    color_det_switch_pub = nh.advertise<std_msgs::Bool>("/prometheus/switch/color_det", 10);
    pad_det_switch_pub = nh.advertise<std_msgs::Bool>("/prometheus/switch/pad_det", 10);

    switch_on.data = true;
    switch_off.data = false;
    //关闭所有节点
    local_planner_switch_pub.publish(switch_off);
    global_planner_switch_pub.publish(switch_off);
    circle_switch_pub.publish(switch_off);
    num_det_switch_pub.publish(switch_off);
    color_det_switch_pub.publish(switch_off);
    pad_det_switch_pub.publish(switch_off);

    nh.param<float>("kpx_circle_track", kpx_circle_track, 0.1);
    nh.param<float>("kpy_circle_track", kpy_circle_track, 0.1);
    nh.param<float>("kpz_circle_track", kpz_circle_track, 0.1);

    nh.param<float>("kpx_land", kpx_land, 0.1);
    nh.param<float>("kpy_land", kpy_land, 0.1);
    nh.param<float>("kpz_land", kpz_land, 0.1);


    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    // Waiting for input
    int start_flag = 0;
    while(start_flag == 0)
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Indoor Competition<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please enter 1 to start the mission..."<<endl;
        cin >> start_flag;
    }

    // 阶段1: 解锁并起飞
    Command_Now.Command_ID = 1;
    while( State_Machine == 0)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.Reference_State.yaw_ref = 999;
        command_pub.publish(Command_Now);   
        cout << "Switch to OFFBOARD and arm ..."<<endl;
        ros::Duration(3.0).sleep();
        
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.position_ref[0]     = START_POINT_X;
        Command_Now.Reference_State.position_ref[1]     = START_POINT_Y;
        Command_Now.Reference_State.position_ref[2]     = START_POINT_Z;
        Command_Now.Reference_State.yaw_ref             = START_POINT_YAW;
        command_pub.publish(Command_Now);
        cout << "Takeoff ..."<<endl;
        ros::Duration(2.0).sleep();
        ros::spinOnce();

        float dis = cal_distance(Eigen::Vector3f(_DroneState.position[0],_DroneState.position[1],_DroneState.position[2]),
                     Eigen::Vector3f(START_POINT_X, START_POINT_Y, START_POINT_Z));
        ros::spinOnce();
        if(dis < DIS_THRES)
        {
            State_Machine = 1;
        }
    }

    //阶段2: 过门,准备穿圆
    while(State_Machine == 1)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.position_ref[0]     = CIRCLE_POINT_X;
        Command_Now.Reference_State.position_ref[1]     = CIRCLE_POINT_Y;
        Command_Now.Reference_State.position_ref[2]     = CIRCLE_POINT_Z;
        Command_Now.Reference_State.yaw_ref             = CIRCLE_POINT_YAW;
        command_pub.publish(Command_Now);
        cout << "Moving to CIRCLE_POINT ..."<<endl;
        ros::Duration(2.0).sleep();

        ros::spinOnce();

        float dis = cal_distance(Eigen::Vector3f(_DroneState.position[0],_DroneState.position[1],_DroneState.position[2]),
                     Eigen::Vector3f(CIRCLE_POINT_X, CIRCLE_POINT_Y, CIRCLE_POINT_Z));

        if(ellipse_det.is_detected && dis < DIS_THRES)
        {
            State_Machine = 2;
        }
    }

    //阶段3: 传圆
    circle_switch_pub.publish(switch_on);
    
    while(State_Machine == 2)
    {
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_VEL;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.position_ref[0]     = 0;
        Command_Now.Reference_State.position_ref[1]     = 0;
        Command_Now.Reference_State.position_ref[2]     = 0;
        Command_Now.Reference_State.velocity_ref[0]     = kpx_circle_track * ellipse_det.pos_body_enu_frame[0];
        Command_Now.Reference_State.velocity_ref[1]     = kpy_circle_track * ellipse_det.pos_body_enu_frame[1];
        Command_Now.Reference_State.velocity_ref[2]     = kpz_circle_track * ellipse_det.pos_body_enu_frame[2];
        Command_Now.Reference_State.yaw_ref             = 0;

        command_pub.publish(Command_Now);   
        cout << "Tracking the cricle ..."<<endl;

        printf_detection_result(ellipse_det);
        ros::spinOnce();
        ros::Duration(0.05).sleep();

        if(abs(ellipse_det.pos_body_enu_frame[0]) < 1)
        {
            Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
            Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
            Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
            Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::BODY_FRAME;
            Command_Now.Reference_State.position_ref[0]     = 2.0;
            Command_Now.Reference_State.position_ref[1]     = 0;
            Command_Now.Reference_State.position_ref[2]     = 0;
            Command_Now.Reference_State.yaw_ref             = 0;
            command_pub.publish(Command_Now);   

            ros::Duration(5.0).sleep();

            State_Machine = 3;
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
            Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
            Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
            Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
            Command_Now.Reference_State.position_ref[0]     = PILLAR_POINT_X;
            Command_Now.Reference_State.position_ref[1]     = PILLAR_POINT_Y;
            Command_Now.Reference_State.position_ref[2]     = PILLAR_POINT_Z;
            Command_Now.Reference_State.yaw_ref             = PILLAR_POINT_YAW;
            command_pub.publish(Command_Now);
            cout << "Moving to PILLAR_POINT ..."<<endl;
        }
    }
    circle_switch_pub.publish(switch_off);

    //发布目标
    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = CORRIDOR_POINT_X;
    goal.pose.position.y = CORRIDOR_POINT_Y;
    goal.pose.position.z = CORRIDOR_POINT_Z;

    while(flag_get_cmd == 0)
    {
        goal_pub.publish(goal);
        local_planner_switch_pub.publish(switch_on);
        cout << "Turn on the local planner and pub the goal point ..."<<endl;
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    
    while(State_Machine == 3)
    {
        // 高度改为定高飞行
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XY_VEL_Z_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.velocity_ref[0]     = desired_vel.x;
        Command_Now.Reference_State.velocity_ref[1]     = desired_vel.y;
        Command_Now.Reference_State.position_ref[2]     = CORRIDOR_POINT_Z;
        Command_Now.Reference_State.yaw_ref             = 0.0;
        command_pub.publish(Command_Now);
        cout << "APF planner:"<<endl;
        cout << "desired_vel: " << desired_vel.x << " [m/s] "<< desired_vel.y << " [m/s] "<< desired_vel.z << " [m/s] "<<endl;
        cout << "drone_pos: " << _DroneState.position[0] << " [m] "<< _DroneState.position[1] << " [m] "<< _DroneState.position[2] << " [m] "<<endl;
        cout << "goal_pos: " << goal.pose.position.x << " [m] "<< goal.pose.position.y << " [m] "<< goal.pose.position.z << " [m] "<<endl;
        ros::spinOnce();
        ros::Duration(0.05).sleep();

        float dis = abs(_DroneState.position[0] - goal.pose.position.x);

        if(dis < (DIS_THRES+0.3))
        {
            State_Machine = 4;
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
            Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
            Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
            Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
            Command_Now.Reference_State.position_ref[0]     = CORRIDOR_POINT_X;
            Command_Now.Reference_State.position_ref[1]     = CORRIDOR_POINT_Y;
            Command_Now.Reference_State.position_ref[2]     = CORRIDOR_POINT_Z;
            Command_Now.Reference_State.yaw_ref             = CORRIDOR_POINT_YAW;
            command_pub.publish(Command_Now);
            cout << "Moving to CORRIDOR_POINT ..."<<endl;
            ros::Duration(2.0).sleep();
        }
    }

    //发布目标
    goal.pose.position.x = NUM_POINT_X;
    goal.pose.position.y = NUM_POINT_Y;
    goal.pose.position.z = NUM_POINT_Z;

    while(flag_get_cmd_a == 0)
    {
        goal_pub.publish(goal);
        local_planner_switch_pub.publish(switch_off);
        global_planner_switch_pub.publish(switch_on);
        cout << "Turn on the global planner and pub the goal point ..."<<endl;
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }

    while(State_Machine == 4)
    {   
        A_star_planner();

        float dis = abs(_DroneState.position[0] - goal.pose.position.x);

        if(dis < 0.4)
        {
            State_Machine = 5;
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
            Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
            Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
            Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
            Command_Now.Reference_State.position_ref[0]     = NUM_POINT_X;
            Command_Now.Reference_State.position_ref[1]     = NUM_POINT_Y;
            Command_Now.Reference_State.position_ref[2]     = NUM_POINT_Z;
            Command_Now.Reference_State.yaw_ref             = NUM_POINT_YAW;
            command_pub.publish(Command_Now);
            cout << "Moving to NUM_POINT ..."<<endl;
            ros::Duration(2.0).sleep();
        }
    }
    global_planner_switch_pub.publish(switch_off);

    color_det_switch_pub.publish(switch_on);


    //巡线
    while(State_Machine == 5)
    {   
        Command_Now.header.stamp                    = ros::Time::now();
        Command_Now.Command_ID                      = Command_Now.Command_ID + 1;

        Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XY_VEL_Z_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::MIX_FRAME;
        Command_Now.Reference_State.velocity_ref[0]     = FOLLOWING_VEL;
        Command_Now.Reference_State.velocity_ref[1]     = FOLLOWING_KP * error_body_y;
        Command_Now.Reference_State.position_ref[2]     = NUM_POINT_Z;
        Command_Now.Reference_State.yaw_ref             = yaw_sp;

        //Publish
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Command_ID   = Command_Now.Command_ID + 1;
        command_pub.publish(Command_Now);
        cout << "Color Line Following... "<< endl;
        cout << "error_body_y: " << error_body_y << " [m] "<<endl;
        cout << "yaw_sp: " << yaw_sp/3.1415926 *180 << " [deg] "<<endl;
        ros::spinOnce();
        ros::Duration(0.05).sleep();

        if(_DroneState.position[0] > LAND_POINT_X - 1.0)
        {
            State_Machine = 6;
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
            Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
            Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
            Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
            Command_Now.Reference_State.position_ref[0]     = LAND_POINT_X;
            Command_Now.Reference_State.position_ref[1]     = LAND_POINT_Y;
            Command_Now.Reference_State.position_ref[2]     = LAND_POINT_Z;
            Command_Now.Reference_State.yaw_ref             = LAND_POINT_YAW;
            command_pub.publish(Command_Now);
            cout << "Moving to LAND_POINT ..."<<endl;
            ros::Duration(4.0).sleep();
        }
    }

    pos_des_prev[0] = _DroneState.position[0];
    pos_des_prev[1] = _DroneState.position[1];
    pos_des_prev[2] = _DroneState.position[2];

    color_det_switch_pub.publish(switch_off);
    pad_det_switch_pub.publish(switch_on);
    
    while(State_Machine == 6)
    {   
        float distance_to_setpoint = landpad_det.pos_body_enu_frame.norm();
        cout <<"[autonomous_landing]: Tracking the Landing Pad, distance_to_setpoint : "<< distance_to_setpoint << " [m] " << endl;
        
        Command_Now.header.stamp                    = ros::Time::now();
        Command_Now.Command_ID                      = Command_Now.Command_ID + 1;
        Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
        Command_Now.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_POS;   //xy velocity z position
        Command_Now.Reference_State.yaw_ref = 0.0;
        Eigen::Vector3f vel_command;
        vel_command[0] = kpx_land * landpad_det.pos_body_enu_frame[0];
        vel_command[1] = kpy_land * landpad_det.pos_body_enu_frame[1];
        vel_command[2] = kpz_land * landpad_det.pos_body_enu_frame[2];
        for (int i=0; i<3; i++)
        {
            Command_Now.Reference_State.position_ref[i] = pos_des_prev[i] + vel_command[i]* 0.05;
            pos_des_prev[i] = Command_Now.Reference_State.position_ref[i];
        }

        command_pub.publish(Command_Now);
        cout << "Autonomous Landing... "<< endl;

        ros::spinOnce();
        ros::Duration(0.05).sleep();

        while(_DroneState.position[2] < 0.4)
        {
            State_Machine = 7;
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode                                = prometheus_msgs::ControlCommand::Disarm;
            Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
            command_pub.publish(Command_Now);
            cout << "Landing and disarm ..."<<endl;
            cout << "The end of the indoor competition ..."<<endl;
            ros::Duration(2.0).sleep();
            ros::spinOnce();
        }
    }
    pad_det_switch_pub.publish(switch_off);
    return 0;

}


void A_star_planner()
{
    float current_cmd_id = flag_get_cmd_a;
    //执行给定航点
    while( A_star.wp_id < A_star.Num_total_wp && flag_get_cmd_a == current_cmd_id)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.position_ref[0]     = A_star.path_cmd.poses[A_star.wp_id].pose.position.x;
        Command_Now.Reference_State.position_ref[1]     = A_star.path_cmd.poses[A_star.wp_id].pose.position.y;
        Command_Now.Reference_State.position_ref[2]     = A_star.path_cmd.poses[A_star.wp_id].pose.position.z;
        Command_Now.Reference_State.yaw_ref             = 0.0;
        
        command_pub.publish(Command_Now);
        cout << "A star planner:"<<endl;
        cout << "Moving to Waypoint: [ " << A_star.wp_id << " / "<< A_star.Num_total_wp<< " ] "<<endl;
        cout << "desired_point: "   << A_star.path_cmd.poses[A_star.wp_id].pose.position.x << " [m] "
                                    << A_star.path_cmd.poses[A_star.wp_id].pose.position.y << " [m] "
                                    << A_star.path_cmd.poses[A_star.wp_id].pose.position.z << " [m] "<<endl; 
        cout << "drone_pos: " << _DroneState.position[0] << " [m] "<< _DroneState.position[1] << " [m] "<< _DroneState.position[2] << " [m] "<<endl;
        
        float wait_time = 0.25;
        ros::spinOnce();
        A_star.wp_id++;
        ros::Duration(wait_time).sleep();
    }
}