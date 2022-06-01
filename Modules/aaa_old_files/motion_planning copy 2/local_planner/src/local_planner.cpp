#include "local_planner.h"


    // 初始化函数
    LocalPlanner::LocalPlanner(ros::NodeHandle &nh)
    {
        // 【参数】编号，从1开始编号
        nh.param("uav_id", uav_id, 0);
        // 【参数】根据参数 planning/algorithm_mode 选择局部避障算法: 0为APF,1为VFH
        nh.param("local_planner/algorithm_mode", algorithm_mode, 0);
        // 【参数】激光雷达模型,0代表3d雷达,1代表2d雷达
        // 3d雷达输入类型为 <sensor_msgs::PointCloud2> 2d雷达输入类型为 <sensor_msgs::LaserScan>
        nh.param("local_planner/lidar_model", lidar_model, 0);
        // 【参数】定高高度
        nh.param("local_planner/fly_height", fly_height, 1.0);
        // 【参数】是否为仿真模式
        nh.param("local_planner/sim_mode", sim_mode, false);
        // 【参数】最大速度
        nh.param("local_planner/max_planning_vel", max_planning_vel, 0.4);

        //【订阅】订阅目标点
        goal_sub = nh.subscribe("/uav" + std::to_string(uav_id) + "/prometheus/planning/goal", 1, &LocalPlanner::goal_cb, this);

        //【订阅】无人机状态信息
        uav_state_sub = nh.subscribe<prometheus_msgs::UAVState>("/uav" + std::to_string(uav_id) + "/prometheus/drone_state",
                                                                1,
                                                                &LocalPlanner::uav_state_cb, this);

        // 订阅传感器点云信息,该话题名字可在launch文件中任意指定
        if (lidar_model == 0)
        {
            local_point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/uav" + std::to_string(uav_id) + "/prometheus/planning/local_pcl", 1, &LocalPlanner::pcl_cb, this);
        }
        else if (lidar_model == 1)
        {
            local_point_cloud_sub = nh.subscribe<sensor_msgs::LaserScan>("/uav" + std::to_string(uav_id) + "/prometheus/planning/local_pcl", 1, &LocalPlanner::laserscan_cb, this);
        }

        // 【发布】控制指令
        uav_cmd_pub = nh.advertise<prometheus_msgs::UAVCommand>("/uav" + std::to_string(uav_id) + "/prometheus/command", 1);

        // 【发布】速度用于显示
        rviz_vel_pub = nh.advertise<geometry_msgs::Point>("/uav" + std::to_string(uav_id) + "/prometheus/local_planner/desired_vel", 10);

        // 【定时器】执行周期为1Hz
        mainloop_timer = nh.createTimer(ros::Duration(0.2), &LocalPlanner::mainloop_cb, this);

        // 【定时器】控制定时器
        control_timer = nh.createTimer(ros::Duration(0.05), &LocalPlanner::control_cb, this);

        // 选择避障算法
        if (algorithm_mode == 0)
        {
            local_alg_ptr.reset(new APF);
            local_alg_ptr->init(nh);
            // pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "APF init.");
        }
        else if (algorithm_mode == 1)
        {
            local_alg_ptr.reset(new VFH);
            local_alg_ptr->init(nh);
            // pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "VFH init.");
        }

        // 规划器状态参数初始化
        exec_state = EXEC_STATE::WAIT_GOAL;
        odom_ready = false;
        drone_ready = false;
        goal_ready = false;
        sensor_ready = false;
        path_ok = false;

        // 初始化发布的指令
        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
        uav_command.position_ref[0] = 0;
        uav_command.position_ref[1] = 0;
        uav_command.position_ref[2] = 0;
        uav_command.yaw_ref = 0;
        desired_yaw = 0.0;

        //　仿真模式下直接发送切换模式与起飞指令
        // todo
        if (sim_mode == true)
        {
            // Waiting for input
            int start_flag = 0;
            while (start_flag == 0)
            {
                cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Global Planner<<<<<<<<<<<<<<<<<<<<<<<<<<< " << endl;
                cout << "Please input 1 for start:" << endl;
                cin >> start_flag;
            }
            // 起飞
            uav_command.header.stamp = ros::Time::now();
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
            uav_command.Command_ID = uav_command.Command_ID + 1;
            uav_cmd_pub.publish(uav_command);
            cout << "Takeoff ..." << endl;
            ros::Duration(3.0).sleep();
        }
        else
        {
            // todo
        }

        // 地图初始化
        sensor_msgs::PointCloud2ConstPtr init_local_map(new sensor_msgs::PointCloud2());
        local_map_ptr_ = init_local_map;
    }

    void LocalPlanner::goal_cb(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        // 2D定高飞行
        goal_pos << msg->pose.position.x, msg->pose.position.y, fly_height;
        goal_vel.setZero();
        goal_ready = true;

        cout << GREEN << NODE_NAME << "Get a new goal point:" << goal_pos(0) << " [m] " << goal_pos(1) << " [m] " << goal_pos(2) << " [m] " << TAIL << endl;

        if (goal_pos(0) == 99 && goal_pos(1) == 99)
        {
            path_ok = false;
            goal_ready = false;
            exec_state = EXEC_STATE::LANDING;
            cout << GREEN << NODE_NAME << "Land " << TAIL << endl;
        }
    }

    void LocalPlanner::uav_state_cb(const prometheus_msgs::UAVState::ConstPtr &msg)
    {
        uav_state = *msg;
        odom_ready = true;
        if (uav_state.connected == true && uav_state.armed == true)
        {
            drone_ready = true;
        }
        else
        {
            drone_ready = false;
        }

        uav_odom.header = uav_state.header;
        uav_odom.child_frame_id = "base_link";

        uav_odom.pose.pose.position.x = uav_state.position[0];
        uav_odom.pose.pose.position.y = uav_state.position[1];
        uav_odom.pose.pose.position.z = fly_height;

        uav_odom.pose.pose.orientation = uav_state.attitude_q;
        uav_odom.twist.twist.linear.x = uav_state.velocity[0];
        uav_odom.twist.twist.linear.y = uav_state.velocity[1];
        uav_odom.twist.twist.linear.z = uav_state.velocity[2];

        if (abs(fly_height - msg->position[2]) > 0.2)
        {
            cout << YELLOW << NODE_NAME << "UAV is not in the desired height. " << TAIL << endl;
        }

        start_pos << uav_odom.pose.pose.position.x, uav_odom.pose.pose.position.y, uav_odom.pose.pose.position.z;

        local_alg_ptr->set_odom(uav_odom);
    }

    void LocalPlanner::laserscan_cb(const sensor_msgs::LaserScanConstPtr &msg)
    {
        if (!odom_ready)
        {
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> _pointcloud;

        _pointcloud.clear();
        pcl::PointXYZ newPoint;
        double newPointAngle;

        int beamNum = msg->ranges.size();
        for (int i = 0; i < beamNum; i++)
        {
            newPointAngle = msg->angle_min + msg->angle_increment * i;
            newPoint.x = msg->ranges[i] * cos(newPointAngle);
            newPoint.y = msg->ranges[i] * sin(newPointAngle);
            newPoint.z = uav_odom.pose.pose.position.z;
            _pointcloud.push_back(newPoint);
        }

        pcl_ptr = _pointcloud.makeShared();
        local_alg_ptr->set_local_map_pcl(pcl_ptr);
        latest_local_pcl_ = *pcl_ptr; // 没用

        sensor_ready = true;
    }

    void LocalPlanner::pcl_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        if (!odom_ready)
        {
            return;
        }

        local_map_ptr_ = msg;
        local_alg_ptr->set_local_map(local_map_ptr_);
        pcl::fromROSMsg(*msg, latest_local_pcl_); // 没用

        sensor_ready = true;
    }

    void LocalPlanner::control_cb(const ros::TimerEvent &e)
    {
        if (!path_ok)
        {
            return;
        }

        distance_to_goal = (start_pos - goal_pos).norm();

        // 抵达终点
        if (distance_to_goal < MIN_DIS)
        {
            uav_command.header.stamp = ros::Time::now();
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            uav_command.Command_ID = uav_command.Command_ID + 1;

            uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
            uav_command.position_ref[0] = goal_pos[0];
            uav_command.position_ref[1] = goal_pos[1];
            uav_command.position_ref[2] = goal_pos[2];

            uav_command.yaw_ref = desired_yaw;
            uav_cmd_pub.publish(uav_command);

            cout << GREEN << NODE_NAME << "Reach the goal! " << TAIL << endl;
            // 停止执行
            path_ok = false;
            // 转换状态为等待目标
            exec_state = EXEC_STATE::WAIT_GOAL;
            return;
        }

        uav_command.header.stamp = ros::Time::now();
        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
        uav_command.Command_ID = uav_command.Command_ID + 1;

        uav_command.Move_mode = prometheus_msgs::UAVCommand::XY_VEL_Z_POS;
        uav_command.position_ref[2] = fly_height;
        uav_command.velocity_ref[0] = desired_vel[0];
        uav_command.velocity_ref[1] = desired_vel[1];
        uav_command.yaw_ref = desired_yaw;

        uav_cmd_pub.publish(uav_command);

        //　发布rviz显示
        vel_rviz.x = desired_vel(0);
        vel_rviz.y = desired_vel(1);
        vel_rviz.z = desired_vel(2);

        rviz_vel_pub.publish(vel_rviz);
    }

    void LocalPlanner::mainloop_cb(const ros::TimerEvent &e)
    {
        static int exec_num = 0;
        exec_num++;

        // 检查当前状态，不满足规划条件则直接退出主循环
        // 此处打印消息与后面的冲突了，逻辑上存在问题
        if (!odom_ready || !drone_ready || !sensor_ready)
        {
            // 此处改为根据循环时间计算的数值
            if (exec_num == 10)
            {
                if (!odom_ready)
                {
                    cout << YELLOW << NODE_NAME << "Need Odom." << TAIL << endl;
                }
                else if (!drone_ready)
                {
                    cout << YELLOW << NODE_NAME << "Drone is not ready." << TAIL << endl;
                }
                else if (!sensor_ready)
                {
                    cout << YELLOW << NODE_NAME << "Need sensor info." << TAIL << endl;
                }
                exec_num = 0;
            }

            return;
        }
        else
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
            if (!goal_ready)
            {
                if (exec_num == 20)
                {
                    cout << GREEN << NODE_NAME << "Waiting for a new goal." << TAIL << endl;
                    exec_num = 0;
                }
            }
            else
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
            if (desired_vel.norm() > max_planning_vel)
            {
                desired_vel = desired_vel / desired_vel.norm() * max_planning_vel;
            }

            if (exec_num == 100)
            {
                if (planner_state == 1)
                {
                    cout << GREEN << NODE_NAME << "local planning desired vel [XY]:" << desired_vel(0) << "[m/s]" << desired_vel(1) << "[m/s]" << TAIL << endl;
                }
                else if (planner_state == 2)
                {
                    cout << YELLOW << NODE_NAME << "Dangerous!" << TAIL << endl;
                }
                exec_num = 0;
            }

            break;
        }
        case LANDING:
        {
            uav_command.header.stamp = ros::Time::now();
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Land;
            uav_command.Command_ID = uav_command.Command_ID + 1;

            uav_cmd_pub.publish(uav_command);
            break;
        }
        }
    }


