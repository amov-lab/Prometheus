#include <autonomous_landing.h>

void mainloop_cb(const ros::TimerEvent& e);
void search_cb(const ros::TimerEvent& e);
void control_cb(const ros::TimerEvent& e);
void printf_cb(const ros::TimerEvent& e);
// 主函数入口
int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");
    ros::Rate rate(100.0);

    // 【参数】悬停模式，用于测试检测精度
    nh.param<bool>("hold_mode", param.hold_mode, false);
    // 【参数】选择Gazebo仿真模式 或 真实实验模式
    nh.param<bool>("sim_mode", param.sim_mode, true);
    //【参数】相机安装偏差,规定为:相机在机体系(质心原点)的位置（设置为0影响也不大）
    nh.param<float>("camera_offset_x", param.camera_offset[0], 0.0);
    nh.param<float>("camera_offset_y", param.camera_offset[1], 0.0);
    nh.param<float>("camera_offset_z", param.camera_offset[2], 0.0);
    //【参数】控制比例参数
    nh.param<float>("kpx_land", param.kp_land[0], 0.1);
    nh.param<float>("kpy_land", param.kp_land[1], 0.1);
    nh.param<float>("kpz_land", param.kp_land[2], 0.1);
    // 默认使用1号机进行实验
    string uav_name = "/uav" + std::to_string(1);      
    //【订阅】靶标位置
    //  方向定义： 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    //  标志位：   detected 用作标志位 ture代表识别到目标 false代表丢失目标
    aruco_sub = nh.subscribe<prometheus_msgs::ArucoInfo>("/prometheus/object_detection/aruco_det", 10, aruco_cb);
    //【订阅】地面真值，此信息仅做比较使用 不强制要求提供
    groundtruth_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/aruco_marker", 10, groundtruth_cb);
    //【订阅】无人机状态
    uav_state_sub = nh.subscribe<prometheus_msgs::UAVState>(uav_name + "/prometheus/drone_state", 10, uav_state_cb);
    //【发布】发送给控制模块
    uav_command_pub = nh.advertise<prometheus_msgs::UAVCommand>(uav_name + "/prometheus/control_command", 10);

    //【定时器】主循环状态机
    ros::Timer mainloop_timer = nh.createTimer(ros::Duration(1.0), mainloop_cb);       
    //【定时器】搜索状态机
    ros::Timer search_timer = nh.createTimer(ros::Duration(2.0), search_cb);        
    //【定时器】无人机控制定时器
    ros::Timer control_timer = nh.createTimer(ros::Duration(0.05), control_cb); 
    //【定时器】打印定时器
    ros::Timer printf_timer = nh.createTimer(ros::Duration(1.0), printf_cb); 

    // 初始化命令
    uav_command.Agent_CMD           = prometheus_msgs::UAVCommand::Current_Pos_Hover;
    uav_command.position_ref[0]     = 0;
    uav_command.position_ref[1]     = 0;
    uav_command.position_ref[2]     = 0;
    uav_command.yaw_ref             = 0;

    vision_info.num_detected = 0;
    vision_info.num_lost = 0;
    vision_info.is_detected = false;
    vision_info.distance_to_pad = 0;

    exec_state = EXEC_STATE::INIT;      // 状态机初始化

    //打印参数,用于检查
    printf_param();

    cout << GREEN << "autonomous_landing init!"<< TAIL << endl;

    int start_flag;
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>Autonomous Landing Mission<<<<<<<<<<<<<<<<<<<<<< "<< endl;
    cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
    cin >> start_flag;

    while(ros::ok)
    {
        ros::spinOnce();
        rate.sleep();
    }
}


void mainloop_cb(const ros::TimerEvent& e)
{
    switch (exec_state)
    {
        case EXEC_STATE::INIT:

            uav_command.Agent_CMD           = prometheus_msgs::UAVCommand::Current_Pos_Hover;
            uav_command_pub.publish(uav_command);
    
        break;

        case EXEC_STATE::SEARCH:

            // 进入搜索模式，重置search_id
            if(start_search)
            {
                search_id = 0;
                start_search = false;
            }  // 当检测到目标，进入水平接近
            else if(vision_info.is_detected)
            {
                exec_state = EXEC_STATE::HORIZON_APPROACH;
                break;
            }

            if(search_id == 0)
            {
                search_point << 0.0, 0.0, 2.0;
            }else if(search_id == 1)
            {
                search_point << 0.0, 5.0, 2.0;
            }else if (search_id == 2)
            {
                search_point << 2.0, 5.0, 2.0;
            }else if (search_id == 3)
            {
                search_point << 2.0, 0.0, 2.0;
            }else if (search_id == 4)
            {
                search_point << 4.0, 0.0, 2.0;
            }else if (search_id == 5)
            {
                search_point << 4.0, 5.0, 2.0;
            }

            dis_to_search_point = (uav_pos - search_point).norm();

            if(dis_to_search_point < 0.2)
            {
                search_id++;
                // 超出最大搜索点，return
                if(search_id >= 6)
                {
                    exec_state = EXEC_STATE::LAND_NOW;
                }
            }
        break;
        
        case EXEC_STATE::HORIZON_APPROACH:

            // 水平靠近过程中丢失目标
            if(!vision_info.is_detected)
            {
                exec_state = EXEC_STATE::SEARCH;
                start_search = true;
            }
            // 水平距离达到阈值，进入下降阶段
            else if(vision_info.aruco_body[0]<0.05 && vision_info.aruco_body[1]<0.05)
            {
                exec_state = EXEC_STATE::DECEND;
            }

        break;

        case EXEC_STATE::DECEND:

            if(uav_pos[2]<0.2)
            {
                exec_state = EXEC_STATE::LAND_NOW;
            }

        break;

        case EXEC_STATE::LAND_NOW:

            // 原地降落模式
            uav_command.header.stamp = ros::Time::now();
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Land;
            uav_command_pub.publish(uav_command);

        break;
    }
}



void search_cb(const ros::TimerEvent& e)
{
    if(exec_state != EXEC_STATE::SEARCH)
    {
        return;
    }

    uav_command.header.stamp = ros::Time::now();
    uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
    uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
    uav_command.position_ref[0] = search_point(0);
    uav_command.position_ref[1] = search_point(1);
    uav_command.position_ref[2] = search_point(2);
    uav_command.yaw_ref = 0.0;
    uav_command_pub.publish(uav_command);
}

void control_cb(const ros::TimerEvent& e)
{
    // 水平接近
    if(exec_state == EXEC_STATE::HORIZON_APPROACH)
    {
        // 机体系速度控制(定高)
        uav_command.header.stamp = ros::Time::now();
        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
        uav_command.Move_mode = prometheus_msgs::UAVCommand::XY_VEL_Z_POS_BODY;
        uav_command.velocity_ref[0] = param.kp_land[0] * vision_info.aruco_body_enu[0];
        uav_command.velocity_ref[1] = param.kp_land[1] * vision_info.aruco_body_enu[1];
        uav_command.position_ref[2] = 2.0;
        uav_command.yaw_ref = 0.0;
        uav_command_pub.publish(uav_command);
    }
    // 下降阶段
    else if(exec_state == EXEC_STATE::DECEND)
    {
        // 机体系速度控制
        uav_command.header.stamp = ros::Time::now();
        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
        uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL_BODY;
        uav_command.velocity_ref[0] = param.kp_land[0] * vision_info.aruco_body_enu[0];
        uav_command.velocity_ref[1] = param.kp_land[1] * vision_info.aruco_body_enu[1];
        uav_command.velocity_ref[2] = -0.2;
        uav_command.yaw_ref = 0.0;
        uav_command_pub.publish(uav_command);
    }else
    {
        return;
    }
}


void printf_cb(const ros::TimerEvent& e)
{
    cout << ">>>>>>>>>>>>>>>>>>>>>>Autonomous Landing Mission<<<<<<<<<<<<<<<<<<<"<< endl;

    switch (exec_state)
    {
        case INIT:
            cout << "[INIT]" <<endl;
            break;

        case SEARCH:
            cout << "[SEARCH]" << "ENU_FRAME, XYZ_POS"<<endl;
            cout << "Reference [ X Y Z ]: " << uav_command.position_ref[0] << " [m] "<< uav_command.position_ref[1] << " [m] "<< uav_command.position_ref[2] << " [m] "<<endl;

            break;
        case HORIZON_APPROACH:
            cout << "[HORIZON_APPROACH]" << "ENU_FRAME, XY_VEL_Z_POS" <<endl;
            cout << "Reference [ X Y Z ]: " << uav_command.velocity_ref[0] << " [m/s] "<< uav_command.velocity_ref[1] << " [m/s] "<< uav_command.position_ref[2] << " [m] "<<endl;
            cout << "distance_to_pad: "  << vision_info.distance_to_pad<< endl;
            
            break;

        case DECEND:
            cout << "[DECEND]" << "ENU_FRAME, XYZ_VEL" <<endl;
            cout << "Reference [ X Y Z ]: " << uav_command.velocity_ref[0] << " [m/s] "<< uav_command.velocity_ref[1] << " [m/s] "<< uav_command.velocity_ref[2] << " [m/s] "<<endl;

            break;
        case LAND_NOW:
            cout << "[LAND_NOW]" <<endl;
            break;
    } 

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

