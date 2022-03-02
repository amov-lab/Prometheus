#include "object_tracking.h"

void printf_cb(const ros::TimerEvent& e);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_tracking");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);

    // 【参数】悬停模式，用于测试检测精度
    nh.param("hold_mode", param.hold_mode, false);
    // 【参数】选择Gazebo仿真模式 或 真实实验模式
    nh.param("sim_mode", param.sim_mode, true);
    //【参数】相机安装偏差,规定为:相机在机体系(质心原点)的位置（设置为0影响也不大）
    nh.param("camera_offset_x", param.camera_offset[0], 0.0);
    nh.param("camera_offset_y", param.camera_offset[1], 0.0);
    nh.param("camera_offset_z", param.camera_offset[2], 0.0);
    //【参数】控制比例参数
    nh.param("kpx_track", param.kp_track[0], 0.1);
    nh.param("kpy_track", param.kp_track[1], 0.1);
    nh.param("kpz_track", param.kp_track[2], 0.1);
    //【参数】初始位置
    nh.param("start_point_x", start_point[0], 0.0);
    nh.param("start_point_y", start_point[1], 0.0);
    nh.param("start_point_z", start_point[2], 2.0);
    //【参数】追踪的前后间隔
    nh.param("tracking_delta_x", tracking_delta[0], 0.0);
    nh.param("tracking_delta_y", tracking_delta[1], 0.0);
    nh.param("tracking_delta_z", tracking_delta[2], 0.0);
    // 默认使用1号机进行实验
    string uav_name = "/uav" + std::to_string(1);   
    //【订阅】靶标位置
    //  方向定义： 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    //  标志位：   detected 用作标志位 ture代表识别到目标 false代表丢失目标
    aruco_sub = nh.subscribe<prometheus_msgs::ArucoInfo>("/prometheus/object_detection/aruco_det", 10, aruco_cb);
    //【订阅】真值，此信息仅做比较使用 不强制要求提供
    groundtruth_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/aruco_marker", 10, groundtruth_cb);
    //【订阅】无人机状态
    uav_state_sub = nh.subscribe<prometheus_msgs::UAVState>(uav_name + "/prometheus/drone_state", 10, uav_state_cb);
    //【发布】发送给控制模块
    uav_command_pub = nh.advertise<prometheus_msgs::UAVCommand>(uav_name + "/prometheus/control_command", 10);
    //【定时器】打印定时器
    ros::Timer printf_timer = nh.createTimer(ros::Duration(1.0), printf_cb);

    //打印现实检查参数
    printf_param();


    cout << GREEN << "object_tracking init!"<< TAIL << endl;

    uav_command.header.stamp = ros::Time::now();
    uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
    uav_command_pub.publish(uav_command);

    int start_flag;
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>Object Tracking Demo<<<<<<<<<<<<<<<<<<<<<< "<< endl;
    cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
    cin >> start_flag;

    uav_command.header.stamp = ros::Time::now();
    uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
    uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
    for (int i=0; i<3; i++)
    {
        uav_command.position_ref[i] = start_point[i];
    }
    uav_command.yaw_ref = 0.0;
    uav_command_pub.publish(uav_command);

    // 
    ros::spinOnce();
    pos_des_prev[0] = uav_pos[0];
    pos_des_prev[1] = uav_pos[1];
    pos_des_prev[2] = uav_pos[2];

    ros::Duration(3.0).sleep();

    while (ros::ok())
    {
        //回调
        ros::spinOnce();

        distance_to_setpoint = vision_info.aruco_body.norm();        
        if(!vision_info.is_detected)
        {
            uav_command.header.stamp = ros::Time::now();
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
            uav_command_pub.publish(uav_command);
            cout <<"[object_tracking]: Lost the Target "<< endl;
        }else 
        {
            cout <<"[object_tracking]: Tracking the Target, distance_to_setpoint : "<< distance_to_setpoint << " [m] " << endl;
            uav_command.header.stamp = ros::Time::now();
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;

            Eigen::Vector3f vel_command;
            vel_command[0] = param.kp_track[0] * (vision_info.aruco_body_enu[0] - tracking_delta[0]);
            vel_command[1] = param.kp_track[1] * (vision_info.aruco_body_enu[1] - tracking_delta[1]);
            vel_command[2] = param.kp_track[2] * (vision_info.aruco_body_enu[2] - tracking_delta[2]);
            for (int i=0; i<3; i++)
            {
                uav_command.position_ref[i] = pos_des_prev[i] + vel_command[i]* 0.05;
            }
            uav_command.yaw_ref = 0.0;
            uav_command_pub.publish(uav_command);
            
            for (int i=0; i<3; i++)
            {
                pos_des_prev[i] = uav_command.position_ref[i];
            }
        }
        
        rate.sleep();
    }

    return 0;

}
void printf_cb(const ros::TimerEvent& e)
{
    cout << ">>>>>>>>>>>>>>>>>>>>>>Obeject Tracking Demo<<<<<<<<<<<<<<<<<<<"<< endl;
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

    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Vision State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    if(vision_info.is_detected)
    {
        cout << "detected: [yes]" <<endl;
    }else
    {
        cout << "detected: [no]" <<endl;
    }

    cout << "Target_pos (body)    : " << vision_info.aruco_body[0] << " [m] "<< vision_info.aruco_body[1] << " [m] "<< vision_info.aruco_body[2] << " [m] "<<endl;
    cout << "Target_pos (body_enu): " << vision_info.aruco_body_enu[0] << " [m] "<< vision_info.aruco_body_enu[1] << " [m] "<< vision_info.aruco_body_enu[2] << " [m] "<<endl;
    cout << "Ground_truth(pos)    :  " << vision_info.GroundTruth.pose.pose.position.x << " [m] "<< vision_info.GroundTruth.pose.pose.position.y << " [m] "<< vision_info.GroundTruth.pose.pose.position.z << " [m] "<<endl;
    cout << "Detection_ENU(pos)   : " << vision_info.aruco_enu[0] << " [m] "<< vision_info.aruco_enu[1] << " [m] "<< vision_info.aruco_enu[2] << " [m] "<<endl;
}



