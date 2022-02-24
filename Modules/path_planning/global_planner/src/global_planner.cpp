#include "global_planner.h"

namespace GlobalPlannerNS
{
    // 初始化函数
    GlobalPlanner::GlobalPlanner(ros::NodeHandle &nh)
    {
        // 【参数】编号，从1开始编号
        nh.param("uav_id", uav_id, 0);
        // 【参数】2D规划时,定高高度
        nh.param("global_planner/fly_height", fly_height, 1.0);
        // 【参数】安全距离，若膨胀距离设置已考虑安全距离，建议此处设为0
        nh.param("global_planner/safe_distance", safe_distance, 0.05);
        // 【参数】
        nh.param("global_planner/time_per_path", time_per_path, 1.0);
        // 【参数】重规划频率
        nh.param("global_planner/replan_time", replan_time, 2.0);
        // 【参数】选择地图更新方式：　0代表全局点云，１代表局部点云，２代表激光雷达scan数据
        nh.param("global_planner/map_input_source", map_input_source, 0);
        // 【参数】是否为仿真模式
        nh.param("global_planner/sim_mode", sim_mode, false);

        //【订阅】目标点
        goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav" + std::to_string(uav_id) + "/prometheus/planning/goal",
                                                            1,
                                                            &GlobalPlanner::goal_cb, this);

        //【订阅】无人机状态信息
        uav_state_sub = nh.subscribe<prometheus_msgs::UAVState>("/uav" + std::to_string(uav_id) + "/prometheus/drone_state",
                                                                1,
                                                                &GlobalPlanner::uav_state_cb, this);

        string uav_name = "/uav" + std::to_string(uav_id);
        //【订阅】根据map_input_source选择地图更新方式
        if (map_input_source == 0)
        {
            cout << GREEN << "Global pcl mode, subscirbe to " << uav_name << "/prometheus/global_planning/global_pcl" << TAIL << endl;
            Gpointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/uav" + std::to_string(uav_id) + "/prometheus/global_planning/global_pcl", 1, &GlobalPlanner::Gpointcloud_cb, this);
        }
        else if (map_input_source == 1)
        {
            cout << GREEN << "Local pcl mode, subscirbe to " << uav_name << "/prometheus/case2/local_pcl" << TAIL << endl;
            Lpointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/uav" + std::to_string(uav_id) + "/prometheus/global_planning/local_pcl", 1, &GlobalPlanner::Lpointcloud_cb, this);
        }
        else if (map_input_source == 2)
        {
            cout << GREEN << "Laser scan mode, subscirbe to " << uav_name << "/prometheus/case2/laser_scan" << TAIL << endl;
            laserscan_sub = nh.subscribe<sensor_msgs::LaserScan>("/uav" + std::to_string(uav_id) + "/prometheus/global_planning/laser_scan", 1, &GlobalPlanner::laser_cb, this);
        }

        // 【发布】 控制指令
        uav_cmd_pub = nh.advertise<prometheus_msgs::UAVCommand>("/uav" + std::to_string(uav_id) + "/prometheus/command", 1);
        // 【发布】发布路径用于显示
        path_cmd_pub = nh.advertise<nav_msgs::Path>("/uav" + std::to_string(uav_id) + "/prometheus/global_planning/path_cmd", 1);
        // 【定时器】 安全检测
        // safety_timer = nh.createTimer(ros::Duration(2.0), &GlobalPlanner::safety_cb, this);
        // 【定时器】 规划器算法执行周期
        mainloop_timer = nh.createTimer(ros::Duration(1.5), &GlobalPlanner::mainloop_cb, this);
        // 【定时器】路径追踪循环，快速移动场景应当适当提高执行频率
        // time_per_path
        track_path_timer = nh.createTimer(ros::Duration(time_per_path), &GlobalPlanner::track_path_cb, this);

        // 【初始化】Astar算法
        Astar_ptr.reset(new Astar);
        Astar_ptr->init(nh);
        cout << GREEN << NODE_NAME << "A_star init. " << TAIL << endl;

        // 规划器状态参数初始化
        exec_state = EXEC_STATE::WAIT_GOAL;
        odom_ready = false;
        drone_ready = false;
        goal_ready = false;
        sensor_ready = false;
        is_safety = true;
        is_new_path = false;

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
            while(start_flag == 0)
            {
                cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Global Planner<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
                cout << "Please input 1 for start:"<<endl;
                cin >> start_flag;
            }
            // 起飞
            uav_command.header.stamp = ros::Time::now();
            uav_command.Agent_CMD  = prometheus_msgs::UAVCommand::Init_Pos_Hover;
            uav_command.Command_ID = uav_command.Command_ID + 1;
            uav_cmd_pub.publish(uav_command);
            cout << "Takeoff ..."<<endl;
            ros::Duration(3.0).sleep();
        }
        else
        {
            // todo
        }
    }

    void GlobalPlanner::goal_cb(const geometry_msgs::PoseStampedConstPtr &msg)
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

    void GlobalPlanner::uav_state_cb(const prometheus_msgs::UAVState::ConstPtr &msg)
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

        // 更新无人车初始位置、速度、加速度，用于规划
        start_pos << uav_odom.pose.pose.position.x, uav_odom.pose.pose.position.y, uav_odom.pose.pose.position.z;
        start_vel << 0.0, 0.0, 0.0;
        start_acc << 0.0, 0.0, 0.0;
    }

    // 根据全局点云更新地图
    // 情况：已知全局点云的场景、由SLAM实时获取的全局点云
    void GlobalPlanner::Gpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        if (!odom_ready)
        {
            return;
        }
        sensor_ready = true;
        static int update_num = 0;
        update_num++;
        // 此处改为根据循环时间计算的数值
        if (update_num == 5)
        {
            // 对Astar中的地图进行更新
            Astar_ptr->Occupy_map_ptr->map_update_gpcl(msg);
            update_num = 0;
        }
    }

    // 根据局部点云更新地图
    // 情况：RGBD相机、三维激光雷达
    void GlobalPlanner::Lpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        if (!odom_ready)
        {
            return;
        }
        sensor_ready = true;
        Astar_ptr->Occupy_map_ptr->map_update_lpcl(msg, uav_odom);
    }

    // 根据2维雷达数据更新地图
    // 情况：2维激光雷达
    void GlobalPlanner::laser_cb(const sensor_msgs::LaserScanConstPtr &msg)
    {
        if (!odom_ready)
        {
            return;
        }
        sensor_ready = true;
        // 对Astar中的地图进行更新（laser+odom）并对地图进行膨胀
        Astar_ptr->Occupy_map_ptr->map_update_laser(msg, uav_odom);
    }

    void GlobalPlanner::track_path_cb(const ros::TimerEvent &e)
    {
        if (!path_ok)
        {
            return;
        }

        is_new_path = false;

        // 抵达终点
        if (cur_id == Num_total_wp - 1)
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

        // 计算距离开始追踪轨迹时间
        tra_running_time = get_time_in_sec(tra_start_time);

        int i = cur_id;

        cout << "Moving to Waypoint: [ " << cur_id << " / " << Num_total_wp << " ] " << endl;
        cout << "Moving to Waypoint:" << path_cmd.poses[i].pose.position.x << " [m] "
             << path_cmd.poses[i].pose.position.y << " [m] "
             << path_cmd.poses[i].pose.position.z << " [m] " << endl;
        // 控制方式如果是走航点，则需要对无人机进行限速，保证无人机的平滑移动
        // 采用轨迹控制的方式进行追踪，期望速度 = （期望位置 - 当前位置）/预计时间；

        uav_command.header.stamp = ros::Time::now();
        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
        uav_command.Command_ID = uav_command.Command_ID + 1;
        
        uav_command.Move_mode = prometheus_msgs::UAVCommand::TRAJECTORY;
        uav_command.position_ref[0] = path_cmd.poses[i].pose.position.x;
        uav_command.position_ref[1] = path_cmd.poses[i].pose.position.y;
        uav_command.position_ref[2] = path_cmd.poses[i].pose.position.z;
        uav_command.velocity_ref[0] = (path_cmd.poses[i].pose.position.x - uav_state.position[0]) / time_per_path;
        uav_command.velocity_ref[1] = (path_cmd.poses[i].pose.position.y - uav_state.position[1]) / time_per_path;
        uav_command.velocity_ref[2] = (path_cmd.poses[i].pose.position.z - uav_state.position[2]) / time_per_path;
        uav_command.yaw_ref = desired_yaw;

        uav_cmd_pub.publish(uav_command);

        cur_id = cur_id + 1;
    }

    // 主循环
    void GlobalPlanner::mainloop_cb(const ros::TimerEvent &e)
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
                    message = "Need Odom.";
                }
                else if (!drone_ready)
                {
                    message = "Drone is not ready.";
                }
                else if (!sensor_ready)
                {
                    message = "Need sensor info.";
                }

                cout << YELLOW << NODE_NAME << message << TAIL << endl;

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
                if (exec_num == 10)
                {
                    message = "Waiting for a new goal.";
                    cout << YELLOW << NODE_NAME << message << TAIL << endl;

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

            // 重置规划器
            Astar_ptr->reset();
            // 使用规划器执行搜索，返回搜索结果

            int astar_state;

            // Astar algorithm
            astar_state = Astar_ptr->search(start_pos, goal_pos);

            // 未寻找到路径
            if (astar_state == Astar::NO_PATH)
            {
                path_ok = false;
                exec_state = EXEC_STATE::WAIT_GOAL;
                cout << RED << NODE_NAME << "Planner can't find path!" << TAIL << endl;
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
                cout << GREEN << NODE_NAME << "Get a new path!" << TAIL << endl;
            }

            break;
        }
        case TRACKING:
        {
            // 本循环是1Hz,此处不是很精准
            if (exec_num >= replan_time)
            {
                exec_state = EXEC_STATE::PLANNING;
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

    // 【获取当前时间函数】 单位：秒
    float GlobalPlanner::get_time_in_sec(const ros::Time &begin_time)
    {
        ros::Time time_now = ros::Time::now();
        float currTimeSec = time_now.sec - begin_time.sec;
        float currTimenSec = time_now.nsec / 1e9 - begin_time.nsec / 1e9;
        return (currTimeSec + currTimenSec);
    }

    void GlobalPlanner::safety_cb(const ros::TimerEvent &e)
    {
        Eigen::Vector3d cur_pos(uav_state.position[0], uav_state.position[1], uav_state.position[2]);

        is_safety = Astar_ptr->check_safety(cur_pos, safe_distance);
    }

    int GlobalPlanner::get_start_point_id(void)
    {
        // 选择与当前无人机所在位置最近的点,并从该点开始追踪
        int id = 0;
        float distance_to_wp_min = abs(path_cmd.poses[0].pose.position.x - uav_state.position[0]) + abs(path_cmd.poses[0].pose.position.y - uav_state.position[1]) + abs(path_cmd.poses[0].pose.position.z - uav_state.position[2]);

        float distance_to_wp;

        for (int j = 1; j < Num_total_wp; j++)
        {
            distance_to_wp = abs(path_cmd.poses[j].pose.position.x - uav_state.position[0]) + abs(path_cmd.poses[j].pose.position.y - uav_state.position[1]) + abs(path_cmd.poses[j].pose.position.z - uav_state.position[2]);

            if (distance_to_wp < distance_to_wp_min)
            {
                distance_to_wp_min = distance_to_wp;
                id = j;
            }
        }

        //　为防止出现回头的情况，此处对航点进行前馈处理
        if (id + 2 < Num_total_wp)
        {
            id = id + 2;
        }

        return id;
    }

} // namespace GlobalPlannerNS