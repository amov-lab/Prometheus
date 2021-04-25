//ROS 头文件
#include <autonomous_landing_aruco.h>

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    // 悬停模式 - 仅用于观察检测结果
    nh.param<bool>("hold_mode", hold_mode, false);
    // 仿真模式 - 区别在于是否自动切换offboard模式
    nh.param<bool>("sim_mode", sim_mode, true);

    // 相机安装偏移,规定为:相机在机体系(质心原点)的位置
    nh.param<float>("camera_offset_x", camera_offset[0], 0.0);
    nh.param<float>("camera_offset_y", camera_offset[1], 0.0);
    nh.param<float>("camera_offset_z", camera_offset[2], 0.0);

    //追踪控制参数
    nh.param<float>("kpx_land", kp_land[0], 0.1);
    nh.param<float>("kpy_land", kp_land[1], 0.1);
    nh.param<float>("kpz_land", kp_land[2], 0.1);

    // 节点运行频率： 20hz 【视觉端解算频率大概为20HZ】
    ros::Rate rate(20.0);

    //【订阅】靶标位置
    //  方向定义： 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    //  标志位：   detected 用作标志位 ture代表识别到目标 false代表丢失目标
    aruco_sub = nh.subscribe<prometheus_msgs::ArucoInfo>("/prometheus/object_detection/aruco_det", 10, aruco_cb);

    //【订阅】无人机状态
    drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    //【订阅】地面真值，此信息仅做比较使用 不强制要求提供
    groundtruth_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/aruco_marker", 10, groundtruth_cb);

    //【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    // 【发布】用于地面站显示的提示消息
    message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/main", 10);

    // 状态机定时器
    ros::Timer mainloop_timer = nh.createTimer(ros::Duration(1.0), mainloop_cb);       

    ros::Timer search_timer = nh.createTimer(ros::Duration(2.0), search_cb);        

    // 无人机控制定时器 20Hz
    ros::Timer control_timer = nh.createTimer(ros::Duration(0.05), control_cb); 

    // 打印定时器
    ros::Timer printf_timer = nh.createTimer(ros::Duration(1.0), printf_cb); 

    Command_Now.Command_ID = 1;
    Command_Now.source = NODE_NAME;

    num_detected = 0;
    num_lost = 0;
    is_detected = false;
    distance_to_pad = 0;

    exec_state = EXEC_STATE::INIT;

    //打印参数,用于检查
    printf_param();

    while(ros::ok)
    {
       // 无人机结束任务 且上锁后
        if(exec_state == EXEC_STATE::DISARM && !_DroneState.armed)
        {
            return 0;
        }

        ros::spinOnce();
        rate.sleep();
    }
}

void search_cb(const ros::TimerEvent& e)
{
    if(exec_state == EXEC_STATE::SEARCH)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Command_ID   = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
        Command_Now.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_POS;  
        Command_Now.Reference_State.position_ref[0] = search_point(0);
        Command_Now.Reference_State.position_ref[1] = search_point(1);
        Command_Now.Reference_State.position_ref[2] = search_point(2);

        command_pub.publish(Command_Now);
    }else
    {
        return;
    }
    
}

void control_cb(const ros::TimerEvent& e)
{
    // 水平接近
    if(exec_state == EXEC_STATE::HORIZON_APPROACH)
    {
        // 机体系速度控制
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Command_ID   = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
        Command_Now.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.Move_mode = prometheus_msgs::PositionReference::XY_VEL_Z_POS;   //xy velocity z position
        
        Command_Now.Reference_State.velocity_ref[0] = kp_land[0] * marker_body_enu[0];
        Command_Now.Reference_State.velocity_ref[1] = kp_land[1] * marker_body_enu[1];
        Command_Now.Reference_State.velocity_ref[2] = 0.0;

        // 当前高度？
        Command_Now.Reference_State.position_ref[2] = 2.0;
        command_pub.publish(Command_Now);
    }
    // 下降阶段
    else if(exec_state == EXEC_STATE::DECEND)
    {
        // 机体系速度控制
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Command_ID   = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
        Command_Now.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_VEL;   //xy velocity z position
        
        Command_Now.Reference_State.velocity_ref[0] = kp_land[0] * marker_body_enu[0];
        Command_Now.Reference_State.velocity_ref[1] = kp_land[1] * marker_body_enu[1];
        // 定速下降
        Command_Now.Reference_State.velocity_ref[2] = -0.2;
        command_pub.publish(Command_Now);
    }else
    {
        return;
    }
}

void mainloop_cb(const ros::TimerEvent& e)
{
     switch (exec_state)
    {
        case INIT:
            if(sim_mode)
            {
                static int start_flag = 0;
                while(start_flag == 0)
                {
                    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>Autonomous Landing Mission<<<<<<<<<<<<<<<<<<<<<< "<< endl;
                    cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
                    cin >> start_flag;
                }

                // 解锁、切换offboard模式
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
                Command_Now.Command_ID = Command_Now.Command_ID + 1;
                Command_Now.source = NODE_NAME;
                Command_Now.Reference_State.yaw_ref = 999;
                command_pub.publish(Command_Now);   
                cout << "Switch to OFFBOARD and arm ..."<<endl;
            }

            // 当无人机进入offboard模式，且处于解锁状态,进入TAKEOFF
            if(_DroneState.mode == "OFFBOARD" && _DroneState.armed )
            {
                exec_state = EXEC_STATE::TAKEOFF;
            }else
            {
                cout << "Waiting  OFFBOARD mode and arm ..."<<endl;
            }
    
        break;

        case TAKEOFF:

            if(_DroneState.position[2]  <  0.8)
            {
                // 记录返航点
                return_point[0] = _DroneState.position[0];
                return_point[1] = _DroneState.position[1];
                return_point[2] = _DroneState.position[2];

                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode  = prometheus_msgs::ControlCommand::Takeoff;
                Command_Now.Command_ID = Command_Now.Command_ID + 1;
                Command_Now.source = NODE_NAME;
                Command_Now.Reference_State.yaw_ref = 0.0;
                command_pub.publish(Command_Now);   
                ros::Duration(1.0).sleep();
            }else
            {
                exec_state = EXEC_STATE::SEARCH;
                start_search = true;
            }

        break;

        case  SEARCH:

            // 进入搜索模式，重置search_id
            if(start_search)
            {
                search_id = 0;
                start_search = false;
            }  // 当检测到目标，进入水平接近
           else  if(is_detected)
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

            dis_to_search_point = (mav_pos - search_point).norm();

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
        
        case HORIZON_APPROACH:

            // 水平靠近过程中丢失目标
            if(!is_detected)
            {
                exec_state = EXEC_STATE::SEARCH;
                start_search = true;
            }
            // 水平距离达到阈值，进入下降阶段
            else if(marker_body[0]<0.05 && marker_body[1]<0.05)
            {
                exec_state = EXEC_STATE::DECEND;
            }

        break;

        case DECEND:

            if(_DroneState.position[2]<0.2)
            {
                exec_state = EXEC_STATE::DISARM;
            }

        break;

        case LAND_NOW:

            // 原地降落模式
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Command_ID   = Command_Now.Command_ID + 1;
            Command_Now.source = NODE_NAME;
            Command_Now.Mode = prometheus_msgs::ControlCommand::Land;
            command_pub.publish(Command_Now);

            if(_DroneState.position[2]<0.2)
            {
                exec_state = EXEC_STATE::DISARM;
            }

        break;

        case  DISARM:

            if(sim_mode)    //？？
            {
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Command_ID   = Command_Now.Command_ID + 1;
                Command_Now.source = NODE_NAME;
                Command_Now.Mode = prometheus_msgs::ControlCommand::Disarm;
                //Command_Now.Mode = prometheus_msgs::ControlCommand::Land;
                command_pub.publish(Command_Now);
            }else
            {
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Command_ID   = Command_Now.Command_ID + 1;
                Command_Now.source = NODE_NAME;
                Command_Now.Mode = prometheus_msgs::ControlCommand::Land;
                command_pub.publish(Command_Now);
            }

        break;
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

        case TAKEOFF:
            cout << "[TAKEOFF]" <<endl;
            break;

        case SEARCH:
            cout << "[SEARCH]" << "ENU_FRAME, XYZ_POS"<<endl;
            cout << "Reference [ X Y Z ]: " << Command_Now.Reference_State.position_ref[0] << " [m] "<< Command_Now.Reference_State.position_ref[1] << " [m] "<< Command_Now.Reference_State.position_ref[2] << " [m] "<<endl;

            break;
        case HORIZON_APPROACH:
            cout << "[HORIZON_APPROACH]" << "ENU_FRAME, XY_VEL_Z_POS" <<endl;

            cout << "Reference [ X Y Z ]: " << Command_Now.Reference_State.velocity_ref[0] << " [m/s] "<< Command_Now.Reference_State.velocity_ref[1] << " [m/s] "<< Command_Now.Reference_State.position_ref[2] << " [m] "<<endl;
            
            cout << "distance_to_pad: "  << distance_to_pad<< endl;
    // char message_chars[256];
    // sprintf(message_chars, "Tracking the Landing Pad, distance_to_the_pad :   %f [m] .", distance_to_pad);
    // message = message_chars;
    // cout << message <<endl;
    // pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
            
            break;

        case DECEND:
            cout << "[DECEND]" << "ENU_FRAME, XYZ_VEL" <<endl;
            cout << "Reference [ X Y Z ]: " << Command_Now.Reference_State.velocity_ref[0] << " [m/s] "<< Command_Now.Reference_State.velocity_ref[1] << " [m/s] "<< Command_Now.Reference_State.velocity_ref[2] << " [m/s] "<<endl;

            break;
        case LAND_NOW:
            cout << "[LAND_NOW]" <<endl;
            break;
            
        case DISARM:
            cout << "[DISARM]" <<endl;
            break;

    } 

    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Vision State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    if(is_detected)
    {
        cout << "detected: [yes]" <<endl;
    }else
    {
        cout << "detected: [no]" <<endl;
    }
    
    cout << "Target_pos (body): " << marker_body[0] << " [m] "<< marker_body[1] << " [m] "<< marker_body[2] << " [m] "<<endl;

    cout << "Target_pos (body_enu): " << marker_body_enu[0] << " [m] "<< marker_body_enu[1] << " [m] "<< marker_body_enu[2] << " [m] "<<endl;

    cout << "Ground_truth(pos):  " << GroundTruth.pose.pose.position.x << " [m] "<< GroundTruth.pose.pose.position.y << " [m] "<< GroundTruth.pose.pose.position.z << " [m] "<<endl;
    cout << "Detection_ENU(pos): " << marker_enu[0] << " [m] "<< marker_enu[1] << " [m] "<< marker_enu[2] << " [m] "<<endl;
}

