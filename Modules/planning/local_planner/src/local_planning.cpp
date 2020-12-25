#include "local_planning.h"

namespace Local_Planning
{

// 局部规划算法 初始化函数
void Local_Planner::init(ros::NodeHandle& nh)
{
    // 参数读取
    // 根据参数 planning/algorithm_mode 选择局部避障算法: 0为APF,1为VFH
    nh.param("local_planner/algorithm_mode", algorithm_mode, 0);
    // 激光雷达模型,0代表3d雷达,1代表2d雷达
    // 3d雷达输入类型为 <sensor_msgs::PointCloud2> 2d雷达输入类型为 <sensor_msgs::LaserScan>
    nh.param("local_planner/lidar_model", lidar_model, 0);
    // TRUE代表2D平面规划及搜索,FALSE代表3D 
    nh.param("local_planner/is_2D", is_2D, true); 
    // 2D规划时,定高高度
    nh.param("local_planner/fly_height_2D", fly_height_2D, 1.0);  
    // 是否为仿真模式
    nh.param("local_planner/sim_mode", sim_mode, false); 
    // 最大速度
    nh.param("local_planner/max_planning_vel", max_planning_vel, 0.4);

    // 订阅目标点
    goal_sub = nh.subscribe("/prometheus/planning/goal", 1, &Local_Planner::goal_cb, this);

    // 订阅 无人机状态
    drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, &Local_Planner::drone_state_cb, this);

    // 订阅传感器点云信息,该话题名字可在launch文件中任意指定
    if (lidar_model == 0)
    {
        local_point_clound_sub = nh.subscribe<sensor_msgs::PointCloud2>("/prometheus/planning/local_pcl", 1, &Local_Planner::localcloudCallback, this);
    }else if (lidar_model == 1)
    {
        local_point_clound_sub = nh.subscribe<sensor_msgs::LaserScan>("/prometheus/planning/local_pcl", 1, &Local_Planner::laserscanCallback, this);
    }

    // 发布 期望速度
    command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);
    
    // 发布提示消息
    message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/local_planner", 10);

    // 发布速度用于显示
    rviz_vel_pub = nh.advertise<geometry_msgs::Point>("/prometheus/local_planner/desired_vel", 10); 

    // 定时函数,执行周期为1Hz
    mainloop_timer = nh.createTimer(ros::Duration(0.2), &Local_Planner::mainloop_cb, this);

    // 控制定时器
    control_timer = nh.createTimer(ros::Duration(0.05), &Local_Planner::control_cb, this);

    // 选择避障算法
    if(algorithm_mode==0){
        local_alg_ptr.reset(new APF);
        local_alg_ptr->init(nh);
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "APF init.");
    }
    else if(algorithm_mode==1)
    {
        local_alg_ptr.reset(new VFH);
        local_alg_ptr->init(nh);
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "VFH init.");
    }

    // 规划器状态参数初始化
    exec_state = EXEC_STATE::WAIT_GOAL;
    odom_ready = false;
    drone_ready = false;
    goal_ready = false;
    sensor_ready = false;
    path_ok = false;

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
    }

    // 地图初始化
    sensor_msgs::PointCloud2ConstPtr init_local_map(new sensor_msgs::PointCloud2());
    local_map_ptr_ = init_local_map;

    ros::spin();
}

void Local_Planner::goal_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if (is_2D == true)
    {
        goal_pos << msg->pose.position.x, msg->pose.position.y, fly_height_2D;
    }else
    {
        goal_pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    }

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

void Local_Planner::drone_state_cb(const prometheus_msgs::DroneStateConstPtr& msg)
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

    local_alg_ptr->set_odom(Drone_odom);

}


void Local_Planner::laserscanCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
    /* need odom_ for center radius sensing */
    if (!odom_ready) 
    {
        return;
    }

    sensor_ready = true;

    sensor_msgs::LaserScan::ConstPtr _laser_scan;

    _laser_scan = msg;

    pcl::PointCloud<pcl::PointXYZ> _pointcloud;

    _pointcloud.clear();
    pcl::PointXYZ newPoint;
    double newPointAngle;

    int beamNum = _laser_scan->ranges.size();
    for (int i = 0; i < beamNum; i++)
    {
        newPointAngle = _laser_scan->angle_min + _laser_scan->angle_increment * i;
        newPoint.x = _laser_scan->ranges[i] * cos(newPointAngle);
        newPoint.y = _laser_scan->ranges[i] * sin(newPointAngle);
        newPoint.z = Drone_odom.pose.pose.position.z;
        _pointcloud.push_back(newPoint);
    }

    pcl_ptr = _pointcloud.makeShared();
    local_alg_ptr->set_local_map_pcl(pcl_ptr);


    latest_local_pcl_ = *pcl_ptr; 
}

void Local_Planner::localcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    /* need odom_ for center radius sensing */
    if (!odom_ready) 
    {
        return;
    }

    sensor_ready = true;

    local_map_ptr_ = msg;
    local_alg_ptr->set_local_map(local_map_ptr_);


    pcl::fromROSMsg(*msg, latest_local_pcl_);
}

void Local_Planner::control_cb(const ros::TimerEvent& e)
{
    if(!path_ok)
    {
        return;
    }

    distance_to_goal = (start_pos - goal_pos).norm();

    // 抵达终点
    if(distance_to_goal < MIN_DIS)
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

    // 目前仅支持定高飞行
    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
    Command_Now.source = NODE_NAME;
    Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XY_VEL_Z_POS;
    Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.velocity_ref[0]     = desired_vel[0];
    Command_Now.Reference_State.velocity_ref[1]     = desired_vel[1];
    Command_Now.Reference_State.position_ref[2]     = fly_height_2D;
    Command_Now.Reference_State.yaw_ref             = desired_yaw;
    
    command_pub.publish(Command_Now);

    //　发布rviz显示
    vel_rviz.x = desired_vel(0);
    vel_rviz.y = desired_vel(1);
    vel_rviz.z = desired_vel(2);

    rviz_vel_pub.publish(vel_rviz);
}

void Local_Planner::mainloop_cb(const ros::TimerEvent& e)
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
                if(exec_num == 20)
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
            // desired_vel是返回的规划速度；返回值为2时,飞机不安全(距离障碍物太近)
            planner_state = local_alg_ptr->compute_force(goal_pos, desired_vel);

            path_ok = true;

            //　对规划的速度进行限幅处理
            if(desired_vel.norm() > max_planning_vel)
            {
                desired_vel = desired_vel / desired_vel.norm() * max_planning_vel; 
            }

            if(exec_num==100)
            {
                char sp[100];
                if(planner_state == 1)
                {
                    sprintf(sp, "local planning desired vel: [%f, %f, %f]", desired_vel(0), desired_vel(1), desired_vel(2));
                }else if(planner_state == 2)
                {
                    sprintf(sp, "Dangerous!");
                }
                
                pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME,sp);
                exec_num=0;
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

}


