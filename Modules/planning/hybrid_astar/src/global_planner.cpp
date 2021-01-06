#include "global_planner.h"

namespace Global_Planning
{

// 初始化函数
void Global_Planner::init(ros::NodeHandle& nh)
{
    // 读取参数
    // 选择算法，　0 代表A_star; 1 代表混合A_star
    nh.param("global_planner/algorithm_mode", algorithm_mode, 0); 
    // TRUE代表2D平面规划及搜索,FALSE代表3D 
    nh.param("global_planner/is_2D", is_2D, true); 
    // 2D规划时,定高高度
    nh.param("global_planner/fly_height_2D", fly_height_2D, 1.0);  
    // 安全距离，若膨胀距离设置已考虑安全距离，建议此处设为0
    nh.param("global_planner/safe_distance", safe_distance, 0.05); 
    nh.param("global_planner/time_per_path", time_per_path, 1.0); 
    // 重规划频率 
    nh.param("global_planner/replan_time", replan_time, 2.0); 
    // 选择地图更新方式：　0代表全局点云，１代表局部点云，２代表激光雷达scan数据
    nh.param("global_planner/map_input", map_input, 0); 
    // 是否为仿真模式
    nh.param("global_planner/sim_mode", sim_mode, false); 

    nh.param("global_planner/map_groundtruth", map_groundtruth, false); 

    // 订阅 目标点
    goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/prometheus/planning/goal", 1, &Global_Planner::goal_cb, this);

    // 订阅 无人机状态
    drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, &Global_Planner::drone_state_cb, this);

    // 根据map_input选择地图更新方式
    if(map_input == 0)
    {
        Gpointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/prometheus/global_planning/global_pcl", 1, &Global_Planner::Gpointcloud_cb, this);
    }else if(map_input == 1)
    {
        Lpointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/prometheus/global_planning/local_pcl", 1, &Global_Planner::Lpointcloud_cb, this);
    }else if(map_input == 2)
    {
        laserscan_sub = nh.subscribe<sensor_msgs::LaserScan>("/prometheus/global_planning/laser_scan", 1, &Global_Planner::laser_cb, this);
    }

    // 发布 路径指令
    command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);
    // 发布提示消息
    message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/global_planner", 10);
    // 发布路径用于显示
    path_cmd_pub   = nh.advertise<nav_msgs::Path>("/prometheus/global_planning/path_cmd",  10); 
    // 定时器 安全检测
    // safety_timer = nh.createTimer(ros::Duration(2.0), &Global_Planner::safety_cb, this); 
    // 定时器 规划器算法执行周期
    mainloop_timer = nh.createTimer(ros::Duration(1.5), &Global_Planner::mainloop_cb, this);        
    // 路径追踪循环，快速移动场景应当适当提高执行频率
    // time_per_path
    track_path_timer = nh.createTimer(ros::Duration(time_per_path), &Global_Planner::track_path_cb, this);        


    Astar_ptr.reset(new KinodynamicAstar);
    Astar_ptr->init(nh);
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Kinodynamic A_star init.");

    // 规划器状态参数初始化
    exec_state = EXEC_STATE::WAIT_GOAL;
    odom_ready = false;
    drone_ready = false;
    goal_ready = false;
    sensor_ready = false;
    is_safety = true;
    is_new_path = false;

    // 初始化发布的指令
    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
    Command_Now.Command_ID = 0;
    Command_Now.source = NODE_NAME;
    desired_yaw = 0.0;

    //　仿真模式下直接发送切换模式与起飞指令
    if(sim_mode == true)
    {
        // Waiting for input
        int start_flag = 0;
        while(start_flag == 0)
        {
            cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Global Planner<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
            cout << "Please input 1 for start:"<<endl;
            cin >> start_flag;
        }
        // 起飞
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Reference_State.yaw_ref = 999;
        command_pub.publish(Command_Now);   
        cout << "Switch to OFFBOARD and arm ..."<<endl;
        ros::Duration(3.0).sleep();
        
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = prometheus_msgs::ControlCommand::Takeoff;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        command_pub.publish(Command_Now);
        cout << "Takeoff ..."<<endl;
        ros::Duration(3.0).sleep();
    }else
    {
        //　真实飞行情况：等待飞机状态变为offboard模式，然后发送起飞指令
        // while(_DroneState.mode != "OFFBOARD")
        // {
        //     Command_Now.header.stamp = ros::Time::now();
        //     Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
        //     Command_Now.Command_ID = 1 ;
        //     Command_Now.source = NODE_NAME;
        //     command_pub.publish(Command_Now);   
        //     cout << "Waiting for the offboard mode"<<endl;
        //     ros::Duration(1.0).sleep();
        //     ros::spinOnce();
        // }
        // Command_Now.header.stamp = ros::Time::now();
        // Command_Now.Mode = prometheus_msgs::ControlCommand::Takeoff;
        // Command_Now.Command_ID = Command_Now.Command_ID + 1;
        // Command_Now.source = NODE_NAME;
        // command_pub.publish(Command_Now);
        // cout << "Takeoff ..."<<endl;
    }

}

void Global_Planner::goal_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if (is_2D == true)
    {
        goal_pos << msg->pose.position.x, msg->pose.position.y, fly_height_2D;
    }else
    {
        goal_pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    }
        
    goal_vel.setZero();

    goal_ready = true;

    // 获得新目标点
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME,"Get a new goal point");

    cout << "Get a new goal point:"<< goal_pos(0) << " [m] "  << goal_pos(1) << " [m] "  << goal_pos(2)<< " [m] "   <<endl;

    if(goal_pos(0) == 99 && goal_pos(1) == 99 )
    {
        path_ok = false;
        goal_ready = false;
        exec_state = EXEC_STATE::LANDING;
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME,"Land");
    }

}

void Global_Planner::drone_state_cb(const prometheus_msgs::DroneStateConstPtr& msg)
{
    _DroneState = *msg;

    if (is_2D == true)
    {
        start_pos << msg->position[0], msg->position[1], fly_height_2D;
        start_vel << msg->velocity[0], msg->velocity[1], 0.0;

        if(abs(fly_height_2D - msg->position[2]) > 0.2)
        {
            pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME,"Drone is not in the desired height.");
        }
    }else
    {
        start_pos << msg->position[0], msg->position[1], msg->position[2];
        start_vel << msg->velocity[0], msg->velocity[1], msg->velocity[2];
    }

    start_acc << 0.0, 0.0, 0.0;

    odom_ready = true;

    if (_DroneState.connected == true && _DroneState.armed == true )
    {
        drone_ready = true;
    }else
    {
        drone_ready = false;
    }

    Drone_odom.header = _DroneState.header;
    Drone_odom.child_frame_id = "base_link";

    Drone_odom.pose.pose.position.x = _DroneState.position[0];
    Drone_odom.pose.pose.position.y = _DroneState.position[1];
    Drone_odom.pose.pose.position.z = _DroneState.position[2];

    Drone_odom.pose.pose.orientation = _DroneState.attitude_q;
    Drone_odom.twist.twist.linear.x = _DroneState.velocity[0];
    Drone_odom.twist.twist.linear.y = _DroneState.velocity[1];
    Drone_odom.twist.twist.linear.z = _DroneState.velocity[2];
}

// 根据全局点云更新地图
// 情况：已知全局点云的场景、由SLAM实时获取的全局点云
void Global_Planner::Gpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    /* need odom_ for center radius sensing */
    if (!odom_ready) 
    {
        return;
    }
    
    sensor_ready = true;

    if(!map_groundtruth)
    {
        // 对Astar中的地图进行更新
        Astar_ptr->Occupy_map_ptr->map_update_gpcl(msg);
        // 并对地图进行膨胀
        Astar_ptr->Occupy_map_ptr->inflate_point_cloud(); 
    }else
    {
        static int update_num=0;
        update_num++;

        // 此处改为根据循环时间计算的数值
        if(update_num == 10)
        {
            // 对Astar中的地图进行更新
            Astar_ptr->Occupy_map_ptr->map_update_gpcl(msg);
            // 并对地图进行膨胀
            Astar_ptr->Occupy_map_ptr->inflate_point_cloud(); 
            update_num = 0;
        } 
    }
    
}

// 根据局部点云更新地图
// 情况：RGBD相机、三维激光雷达
void Global_Planner::Lpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    /* need odom_ for center radius sensing */
    if (!odom_ready) 
    {
        return;
    }
    
    sensor_ready = true;

    // 对Astar中的地图进行更新（局部地图+odom）
    Astar_ptr->Occupy_map_ptr->map_update_lpcl(msg, Drone_odom);
    // 并对地图进行膨胀
    Astar_ptr->Occupy_map_ptr->inflate_point_cloud(); 
}

// 根据2维雷达数据更新地图
// 情况：2维激光雷达
void Global_Planner::laser_cb(const sensor_msgs::LaserScanConstPtr &msg)
{
    /* need odom_ for center radius sensing */
    if (!odom_ready) 
    {
        return;
    }
    
    sensor_ready = true;

    // 对Astar中的地图进行更新（laser+odom）
    Astar_ptr->Occupy_map_ptr->map_update_laser(msg, Drone_odom);
    // 并对地图进行膨胀
    Astar_ptr->Occupy_map_ptr->inflate_point_cloud(); 
}

void Global_Planner::track_path_cb(const ros::TimerEvent& e)
{
    if(!path_ok)
    {
        return;
    }

    // if(!is_safety)
    // {
    //     // 若无人机与障碍物之间的距离小于安全距离，则停止执行路径
    //     // 但如何脱离该点呢？
    //     pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Drone Position Dangerous! STOP HERE and wait for new goal.");
        
    //     Command_Now.header.stamp = ros::Time::now();
    //     Command_Now.Mode         = prometheus_msgs::ControlCommand::Hold;
    //     Command_Now.Command_ID   = Command_Now.Command_ID + 1;
    //     Command_Now.source = NODE_NAME;

    //     command_pub.publish(Command_Now);

    //     goal_ready = false;
    //     exec_state = EXEC_STATE::WAIT_GOAL;
        
    //     return;
    // }
    is_new_path = false;

    // 抵达终点
    if(cur_id == Num_total_wp - 1)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.position_ref[0]     = goal_pos[0];
        Command_Now.Reference_State.position_ref[1]     = goal_pos[1];
        Command_Now.Reference_State.position_ref[2]     = goal_pos[2];

        Command_Now.Reference_State.yaw_ref             = desired_yaw;
        command_pub.publish(Command_Now);

        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Reach the goal!");
        
        // 停止执行
        path_ok = false;
        // 转换状态为等待目标
        exec_state = EXEC_STATE::WAIT_GOAL;
        return;
    }
 
    // 计算距离开始追踪轨迹时间
    tra_running_time = get_time_in_sec(tra_start_time);

    int i = cur_id;

    cout << "Moving to Waypoint: [ " << cur_id << " / "<< Num_total_wp<< " ] "<<endl;
    cout << "Moving to Waypoint:"   << path_cmd.poses[i].pose.position.x  << " [m] "
                                    << path_cmd.poses[i].pose.position.y  << " [m] "
                                    << path_cmd.poses[i].pose.position.z  << " [m] "<<endl; 
    // 控制方式如果是走航点，则需要对无人机进行限速，保证无人机的平滑移动
    // 采用轨迹控制的方式进行追踪，期望速度 = （期望位置 - 当前位置）/预计时间；
    
    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
    Command_Now.source = NODE_NAME;
    Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::TRAJECTORY;
    Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0]     = path_cmd.poses[i].pose.position.x;
    Command_Now.Reference_State.position_ref[1]     = path_cmd.poses[i].pose.position.y;
    Command_Now.Reference_State.position_ref[2]     = path_cmd.poses[i].pose.position.z;
    Command_Now.Reference_State.velocity_ref[0]     = (path_cmd.poses[i].pose.position.x - _DroneState.position[0])/time_per_path;
    Command_Now.Reference_State.velocity_ref[1]     = (path_cmd.poses[i].pose.position.y - _DroneState.position[1])/time_per_path;
    Command_Now.Reference_State.velocity_ref[2]     = (path_cmd.poses[i].pose.position.z - _DroneState.position[2])/time_per_path;
    Command_Now.Reference_State.yaw_ref             = desired_yaw;
    
    command_pub.publish(Command_Now);

    cur_id = cur_id + 1;
}
 
// 主循环 
void Global_Planner::mainloop_cb(const ros::TimerEvent& e)
{
    static int exec_num=0;
    exec_num++;

    // 检查当前状态，不满足规划条件则直接退出主循环
    // 此处打印消息与后面的冲突了，逻辑上存在问题
    if(!odom_ready || !drone_ready || !sensor_ready)
    {
        // 此处改为根据循环时间计算的数值
        if(exec_num == 10)
        {
            if(!odom_ready)
            {
                message = "Need Odom.";
            }else if(!drone_ready)
            {
                message = "Drone is not ready.";
            }else if(!sensor_ready)
            {
                message = "Need sensor info.";
            }

            pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
            exec_num=0;
        }  

        return;
    }else
    {
        // 对检查的状态进行重置
        odom_ready = false;
        drone_ready = false;
        sensor_ready = false;
    }
    
    switch (exec_state)
    {
        case WAIT_GOAL:
        {
            path_ok = false;
            if(!goal_ready)
            {
                if(exec_num == 10)
                {
                    message = "Waiting for a new goal.";
                    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME,message);
                    exec_num=0;
                }
            }else
            {
                // 获取到目标点后，生成新轨迹
                exec_state = EXEC_STATE::PLANNING;
                goal_ready = false;
            }
            
            break;
        }
        case PLANNING:
        {
            
            // 重置规划器
            Astar_ptr->reset();
            // 使用规划器执行搜索，返回搜索结果

            
            bool init = false;
            bool dynamic = false;
            double time_start = 0;

            int astar_state = Astar_ptr->search(start_pos, start_vel, start_acc, goal_pos, goal_vel, init, dynamic, time_start);

            if(astar_state==KinodynamicAstar::NO_PATH)
            {
                path_ok = false;
                exec_state = EXEC_STATE::WAIT_GOAL;
                pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "a star find no path, please reset the goal!");
            }
            else
            {
                path_ok = true;
                is_new_path = true;
                path_cmd = Astar_ptr->get_ros_path();
                Num_total_wp = path_cmd.poses.size();
                start_point_index = get_start_point_id();
                cur_id = start_point_index;
                tra_start_time = ros::Time::now();
                exec_state = EXEC_STATE::TRACKING;
                path_cmd_pub.publish(path_cmd);
                pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "astart find path success!");
            }
            
            

            break;
        }
        case TRACKING:
        {
            // 本循环是1Hz,此处不是很精准
            if(exec_num >= replan_time)
            {
                exec_state = EXEC_STATE::PLANNING;
                exec_num = 0;
            }

            break;
        }
        case  LANDING:
        {
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode         = prometheus_msgs::ControlCommand::Land;
            Command_Now.Command_ID   = Command_Now.Command_ID + 1;
            Command_Now.source = NODE_NAME;

            command_pub.publish(Command_Now);
            break;
        }
    }


}

// 【获取当前时间函数】 单位：秒
float Global_Planner::get_time_in_sec(const ros::Time& begin_time)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin_time.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin_time.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void Global_Planner::safety_cb(const ros::TimerEvent& e)
{
    Eigen::Vector3d cur_pos(_DroneState.position[0], _DroneState.position[1], _DroneState.position[2]);
    
    is_safety = Astar_ptr->check_safety(cur_pos, safe_distance);
}

int Global_Planner::get_start_point_id(void)
{
    // 选择与当前无人机所在位置最近的点,并从该点开始追踪
    int id = 0;
    float distance_to_wp_min = abs(path_cmd.poses[0].pose.position.x - _DroneState.position[0])
                                + abs(path_cmd.poses[0].pose.position.y - _DroneState.position[1])
                                + abs(path_cmd.poses[0].pose.position.z - _DroneState.position[2]);
    
    float distance_to_wp;

    for (int j=1; j<Num_total_wp;j++)
    {
        distance_to_wp = abs(path_cmd.poses[j].pose.position.x - _DroneState.position[0])
                                + abs(path_cmd.poses[j].pose.position.y - _DroneState.position[1])
                                + abs(path_cmd.poses[j].pose.position.z - _DroneState.position[2]);
        
        if(distance_to_wp < distance_to_wp_min)
        {
            distance_to_wp_min = distance_to_wp;
            id = j;
        }
    }

    //　为防止出现回头的情况，此处对航点进行前馈处理
    if(id + 4 < Num_total_wp)
    {
        id = id + 4;
    }

    return id;
}



}