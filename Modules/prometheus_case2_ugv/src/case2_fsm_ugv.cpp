#include "case2_fsm_ugv.h"

namespace prometheus_case2_ugv
{
// 初始化函数
void Case2FSM_UGV::init(ros::NodeHandle& nh)
{
    // 读取参数
    // 集群数量
    nh.param("case2_ugv/swarm_num_ugv", swarm_num_ugv, 1);
    // 无人车编号 1号无人车则为1
    nh.param("case2_ugv/ugv_id", ugv_id, 0);
    // 无人车高度
    nh.param("case2_ugv/ugv_height", ugv_height, 0.0);
    // 是否为仿真模式
    nh.param("case2_ugv/sim_mode", sim_mode, false); 
    // A星算法 重规划频率 
    nh.param("case2_ugv/replan_time", replan_time, 1.0); 
    nh.param("case2_ugv/track_frequency", track_frequency, 0.1); 
    // 手动给定目标点模式 或 自动目标点模式
    nh.param("case2_ugv/manual_mode", manual_mode, false);
    // 是否追踪目标 （如果不追踪，case2 等同于 case3）
    nh.param("case2_ugv/follow_target", follow_target, false);
    // 检测范围
    nh.param("case2_ugv/detection_range_x_min", detection_range_x(0), -10.0);
    nh.param("case2_ugv/detection_range_x_max", detection_range_x(1), 10.0);
    nh.param("case2_ugv/detection_range_y_min", detection_range_y(0), -10.0);
    nh.param("case2_ugv/detection_range_y_max", detection_range_y(1), 10.0);

    ugv_name = "/ugv" + std::to_string(ugv_id);

    // 选择地图更新方式：　0代表全局点云，1代表局部点云，2代表激光雷达scan数据
    nh.param("case2_ugv/map_input_source", map_input_source, 2); 
    //【订阅】 根据map_input选择地图更新方式
    if(map_input_source == 0)
    {
        cout << GREEN << "Global pcl mode, subscirbe to "<< ugv_name << "/prometheus/case2/global_pcl" << TAIL <<endl;
        Gpointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(ugv_name + "/prometheus/case2/global_pcl", 1, &Case2FSM_UGV::Gpointcloud_cb, this);
    }else if(map_input_source == 1)
    {
        cout << GREEN << "Local pcl mode, subscirbe to "<< ugv_name << "/prometheus/case2/local_pcl" << TAIL <<endl;
        Lpointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(ugv_name + "/prometheus/case2/local_pcl", 1, &Case2FSM_UGV::Lpointcloud_cb, this);
    }else if(map_input_source == 2)
    {
        cout << GREEN << "Laser scan mode, subscirbe to "<< ugv_name << "/prometheus/case2/laser_scan" << TAIL <<endl;
        laserscan_sub = nh.subscribe<sensor_msgs::LaserScan>(ugv_name + "/prometheus/case2/laser_scan", 1, &Case2FSM_UGV::laser_cb, this);
    }

    if(manual_mode)
    {
        cout << GREEN << "Manual goal mode, subscirbe to "<< ugv_name << "/prometheus/case2/goal" << TAIL <<endl;
        // 【订阅】手动给定目标点模式 手动目标点
        goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(ugv_name + "/prometheus/case2/goal", 1, &Case2FSM_UGV::goal_cb, this);
    }
    
    // 【订阅】无人车状态
    ugv_state_sub = nh.subscribe<prometheus_msgs::UGVState>(ugv_name + "/prometheus/ugv_state", 10, &Case2FSM_UGV::ugv_state_cb, this);

    // 【订阅】其他无人车位置
    for(int i = 1; i <= swarm_num_ugv; i++)
    {
        if(i == ugv_id)
        {
            continue;
        }
        get_nei_odom[i] = false;
        odom_nei[i] << 99.9,99.9,99.9;
        nei_odom_sub[i] = nh.subscribe<nav_msgs::Odometry>("/ugv"+std::to_string(i)+"/prometheus/ugv_odom", 10, boost::bind(&Case2FSM_UGV::nei_odom_cb,this,_1,i));
    }

    // 【地面站交互】地面站控制指令
    station_cmd_sub = nh.subscribe<prometheus_msgs::StationCommand>(ugv_name + "/ground_station/ugv_cmd", 1, &Case2FSM_UGV::cmd_cb, this);
    
    // 【地面站交互】将检测结果发送至地面站
    case2_result_pub = nh.advertise<prometheus_msgs::Case2Result>(ugv_name + "/ground_station/ugv_result_case2", 10);

    // 【订阅】仿真中为aruco码检测，实机中更换为YOLO检测
    if(sim_mode)
    {
        detection_sub = nh.subscribe<prometheus_msgs::ArucoInfo>(ugv_name + "/prometheus/object_detection/aruco_det", 10, &Case2FSM_UGV::detection_cb, this);
    }else
    {
        detection_sub_real = nh.subscribe<prometheus_msgs::MultiDetectionInfo>(ugv_name + "/prometheus/object_detection/yolo", 1, &Case2FSM_UGV::detection_cb_real, this);
    }

    // 【发布】 路径指令 （发送至swarm_controller.cpp）
    command_pub = nh.advertise<prometheus_msgs::UGVCommand>(ugv_name + "/prometheus/ugv_command", 1);
    // 【发布】路径用于显示（rviz显示）
    path_cmd_pub   = nh.advertise<nav_msgs::Path>(ugv_name + "/prometheus/case2/path_cmd",  1); 

    // 【定时器】主循环执行
    mainloop_timer = nh.createTimer(ros::Duration(0.1), &Case2FSM_UGV::mainloop_cb, this);        
    // 【定时器】路径追踪
    track_path_timer = nh.createTimer(ros::Duration(track_frequency), &Case2FSM_UGV::track_path_cb, this);        
    // 【定时器】物体追踪
    object_tracking_timer = nh.createTimer(ros::Duration(0.02), &Case2FSM_UGV::object_tracking_cb, this);  
    // 【定时器】更新其他无人车位置
    send_nei_odom_timer = nh.createTimer(ros::Duration(0.02), &Case2FSM_UGV::send_nei_odom_cb, this); 

    // Astar algorithm
    Astar_ptr.reset(new Astar);
    Astar_ptr->init(nh);

    // 规划器状态参数初始化
    exec_state = EXEC_STATE::INIT;
    odom_ready = false;
    ugv_ready = false;
    get_goal = false;
    station_ready = false;
    sensor_ready = false;
    path_ok = false;
    in_return_mode = false;
    rotate_in_place = false;
    start_move = false;
    get_target_pos = false;
    counter_search = 0;
    yaw_ref = 0.0;

    // 手动目标点情况，不需要地面站指令
    if(manual_mode)
    {
        station_ready = true;
    }

    // 检测相关
    detected_by_myself = false;
    detected_by_others = false;
    num_count_vision_lost = 0;
    object_pos_body << 0.0, 0.0, 0.0; 
    object_pos_enu << 0.0, 0.0, 0.0;
    lost_object = true;

    // 初始化发布的指令
    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode  = prometheus_msgs::UGVCommand::Hold;
    Command_Now.Command_ID = 0;
}

void Case2FSM_UGV::nei_odom_cb(const nav_msgs::Odometry::ConstPtr& odom, int id) 
{
    odom_nei[id] << odom->pose.pose.position.x, odom->pose.pose.position.y, ugv_height; 

    // 距离大于最大距离的其他无人车不考虑
    if(sim_mode)
    {
        if((start_pos-odom_nei[id]).norm() > 5.0 /*米*/ )
        {
            get_nei_odom[id] = false;
        }else
        {
            get_nei_odom[id] = true;
        }
    }
    else
    {
        get_nei_odom[id] = true;
    }
}

void Case2FSM_UGV::send_nei_odom_cb(const ros::TimerEvent& e)
{
    Astar_ptr->Occupy_map_ptr->ugv_pcl_update(odom_nei, get_nei_odom);

    // for(int i = 1; i <= swarm_num_ugv; i++)
    // {
    //     get_nei_odom[i] = false;
    // }
}

void Case2FSM_UGV::goal_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    // 2D定高飞行
    goal_pos << msg->pose.position.x, msg->pose.position.y, ugv_height;
        
    goal_vel.setZero();

    get_goal = true;
    rotate_in_place = true;
    start_move = false;

    cout << GREEN << ugv_name + " Case2: Get a new manual goal: ["<< goal_pos(0) << ", "  << goal_pos(1)  << ", "  << goal_pos(2) << " ]"  << TAIL <<endl;
}

void Case2FSM_UGV::cmd_cb(const prometheus_msgs::StationCommandConstPtr& msg)
{
    // 将无人车ID与FLAG进行比较，判断是否执行本次任务
    if(msg->flag == 1 && msg->Command == prometheus_msgs::StationCommand::Start)
    {
        if(!station_ready)
        {
            station_ready = true;
            cout << GREEN << ugv_name + " Case 2 : Get station start command." << TAIL <<endl; 
        }
        target_pos = msg->goal;
        target_pos.pose.position.z = ugv_height;
        get_target_pos = true;
        cout << GREEN << ugv_name +  " Case 2 : Get target_pos from station command:"<<" ["<< target_pos.pose.position.x << ", "  << target_pos.pose.position.y  << ", "  << target_pos.pose.position.z<< " ]"  << TAIL <<endl;
    }

    if(msg->Command == prometheus_msgs::StationCommand::Return)
    {
        path_ok = false;
        exec_state = EXEC_STATE::RETURN;
        cout << GREEN << ugv_name + " Case2: Get station_cmd: [ RETURN ]."  << TAIL <<endl;
        return;
    }else if(msg->Command == prometheus_msgs::StationCommand::Stop)
    {
        path_ok = false;
        exec_state = EXEC_STATE::STOP;
        cout << YELLOW << ugv_name + " Case2: Get station_cmd: [ STOP ]."  << TAIL <<endl;
        return;
    }
}

void Case2FSM_UGV::ugv_state_cb(const prometheus_msgs::UGVStateConstPtr& msg)
{
    ugv_state = *msg;

    ugv_ready = true;
    odom_ready = true;

    //无人车里程计，用于建图
    ugv_odom.header = ugv_state.header;
    ugv_odom.child_frame_id = "base_link";

    ugv_odom.pose.pose.position.x = ugv_state.position[0];
    ugv_odom.pose.pose.position.y = ugv_state.position[1];
    ugv_odom.pose.pose.position.z = ugv_height;                                        //无人车固定高度
    ugv_odom.pose.pose.orientation = ugv_state.attitude_q;

    ugv_odom.twist.twist.linear.x = ugv_state.velocity[0];
    ugv_odom.twist.twist.linear.y = ugv_state.velocity[1];
    ugv_odom.twist.twist.linear.z = ugv_state.velocity[2];

    ugv_yaw = ugv_state.attitude[2];

    // 更新无人车初始位置、速度、加速度，用于规划
    start_pos << ugv_odom.pose.pose.position.x, ugv_odom.pose.pose.position.y,ugv_odom.pose.pose.position.z;
    start_vel << 0.0, 0.0, 0.0;
    start_acc << 0.0, 0.0, 0.0;
}

// 根据全局点云更新地图
// 情况：已知全局点云的场景、由SLAM实时获取的全局点云
void Case2FSM_UGV::Gpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    if (!odom_ready) 
    {
        return;
    }
    sensor_ready = true;
    static int update_num=0;
    update_num++;
    // 此处改为根据循环时间计算的数值
    if(update_num == 5)
    {
        // 对Astar中的地图进行更新
        Astar_ptr->Occupy_map_ptr->map_update_gpcl(msg);
        update_num = 0;
    }
}

// 根据局部点云更新地图
// 情况：RGBD相机、三维激光雷达
void Case2FSM_UGV::Lpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    if (!odom_ready) 
    {
        return;
    }
    sensor_ready = true;
    Astar_ptr->Occupy_map_ptr->map_update_lpcl(msg, ugv_odom);
}

// 根据2维雷达数据更新地图
// 情况：2维激光雷达
void Case2FSM_UGV::laser_cb(const sensor_msgs::LaserScanConstPtr &msg)
{
    if (!odom_ready) 
    {
        return;
    }

    sensor_ready = true;
    // 对Astar中的地图进行更新（laser+odom）并对地图进行膨胀
    Astar_ptr->Occupy_map_ptr->map_update_laser(msg, ugv_odom);
}

void Case2FSM_UGV::track_path_cb(const ros::TimerEvent& e)
{
    static int track_path_num = 0;
    
    if(exec_state == EXEC_STATE::OBEJECT_TRACKING)
    {
        return;
    }

    if(!path_ok)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode         = prometheus_msgs::UGVCommand::Hold;
        Command_Now.Command_ID   = Command_Now.Command_ID + 1;
        command_pub.publish(Command_Now); 
        return;
    }

    // 抵达终点
    if(cur_id == Num_total_wp - 1)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode              = prometheus_msgs::UGVCommand::Point_Control;
        Command_Now.Command_ID        = Command_Now.Command_ID + 1;
        Command_Now.position_ref[0]     = path_cmd.poses[cur_id].pose.position.x;
        Command_Now.position_ref[1]     = path_cmd.poses[cur_id].pose.position.y;
        Command_Now.yaw_ref      = yaw_ref;
        command_pub.publish(Command_Now);
        cout << GREEN << ugv_name + " Case2: Path tracking: [ Reach the goal ]."  << TAIL <<endl;
        
        // 停止执行
        path_ok = false;

        float error_pos = sqrt ((path_cmd.poses[cur_id].pose.position.x - ugv_state.position[0])*(path_cmd.poses[cur_id].pose.position.x - ugv_state.position[0])                  
                                                               +    (path_cmd.poses[cur_id].pose.position.y - ugv_state.position[1])*(path_cmd.poses[cur_id].pose.position.y - ugv_state.position[1])     );
        float error_yaw = abs(yaw_ref - ugv_yaw);

        // 等待无人车移动至目标状态
        if(error_pos < 0.2 && error_yaw<0.1)
        {
            if(in_return_mode)
            {
                exec_state = EXEC_STATE::STOP;
                cout << GREEN << ugv_name + " stop."  << TAIL <<endl;
            }else if(manual_mode)
            {
                exec_state = EXEC_STATE::WAIT_GOAL;
            }
            else 
            {
                if(exec_state == EXEC_STATE::PATH_TRACKING)
                {
                    if(!follow_target)
                    {
                        exec_state = EXEC_STATE::STOP;
                    }
                    else
                    {
                        if(num_count_vision_get > 5)
                        {
                            exec_state = EXEC_STATE::OBEJECT_TRACKING;
                        }else
                        {
                            exec_state = EXEC_STATE::RETURN;
                            if(sim_mode) exec_state = EXEC_STATE::WAIT_GOAL;
                        }
                    }
                }
            }
        }
        return;
    }

    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode              = prometheus_msgs::UGVCommand::Path_Control;
    Command_Now.Command_ID        = Command_Now.Command_ID + 1;
    Command_Now.position_ref[0]     = path_cmd.poses[cur_id].pose.position.x;
    Command_Now.position_ref[1]     = path_cmd.poses[cur_id].pose.position.y;
    Command_Now.yaw_ref      =   yaw_ref;

    command_pub.publish(Command_Now); 
    cur_id = cur_id + 1;
}
 

void Case2FSM_UGV::object_tracking_cb(const ros::TimerEvent& e)
{
    if(exec_state != EXEC_STATE::OBEJECT_TRACKING)
    {
        return;
    }

    if(!follow_target)
    {
        return;
    }

    // 面向目标，距离为0.5米左右
    if(sim_mode)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode              = prometheus_msgs::UGVCommand::Direct_Control_BODY;
        Command_Now.Command_ID        = Command_Now.Command_ID + 1;
        Command_Now.linear_vel[0]     = k_p*(path_cmd.poses[cur_id].pose.position.x - ugv_state.position[0]);
        Command_Now.linear_vel[1]     = k_p*(path_cmd.poses[cur_id].pose.position.y - ugv_state.position[1]);
        Command_Now.angular_vel       = 0.0;
    }else
    {

        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::UGVCommand::Direct_Control_ENU;
        Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
        if(abs(depth - 0.6)>0.1)                    
            Command_Now.linear_vel[0] = 0.5*(depth - 0.6);
        else
            Command_Now.linear_vel[0] = 0;
        if(abs(angle_y/M_PI *180 - 5.0)>8.0)                   
            Command_Now.linear_vel[1] = -1.0*(angle_y - 5.0*M_PI/180);
        else if(abs(angle_y/M_PI *180 - 5.0)>1.0)
            Command_Now.linear_vel[1] = -0.1*(angle_y - 5.0*M_PI/180);
        else
            Command_Now.linear_vel[1] = 0;

        Command_Now.yaw_ref = 0.0;
    }


    command_pub.publish(Command_Now);
}

void Case2FSM_UGV::detection_cb_real(const prometheus_msgs::MultiDetectionInfoConstPtr &msg)
{
    // return过程中不检测
    if(in_return_mode)
    {
        return;
    }

    if(!follow_target)
    {
        return;
    }

    if(exec_state == EXEC_STATE::WAIT_GOAL || exec_state == EXEC_STATE::INIT)
    {
        return;
    }

    if(num_count_vision_lost > 10 && exec_state == EXEC_STATE::OBEJECT_TRACKING)
    {
        cout << RED << ugv_name + "  [ lost target ]."  << TAIL <<endl;
        exec_state = EXEC_STATE::WAIT_GOAL;
    }

    if(msg->num_objs == 0)
    {
        num_count_vision_lost++;
        num_count_vision_get = 0;
        return;
    }

    if(msg->num_objs != 1)
    {
        cout << RED << "ERROR: detect more than one objects!"  << TAIL <<endl;
        return;
    }
    
    if(msg->detection_infos[0].detected)
    {
        num_count_vision_get++;
        num_count_vision_lost = 0;
    }

    // 无人车这个要重写！！ todo
    angle_y = msg->detection_infos[0].sight_angle[0];

    // 怎么计算精准的深度？
    // depth 为目标在无人车前方的长度，大于0，越远越大
    depth = msg->detection_infos[0].position[2];

    // cout << GREEN << " Case2 Detection result, angle_y: "<< angle_y/M_PI *180 <<" [deg], depth: "<<depth<<" [m]."  << TAIL <<endl;

    dis_to_object = sqrt( object_pos_body[0] * object_pos_body[0] + object_pos_body[1] * object_pos_body[1] );

    prometheus_msgs::Case2Result result;
    result.detected = true;
    result.moving_target = true;
    // 目标在ENU下的大约位置,todo
    result.enu_position[0] = start_pos[0] + msg->detection_infos[0].position[2];
    result.enu_position[1] = start_pos[1] - msg->detection_infos[0].position[0];
    result.enu_position[2] = start_pos[2];
    // 发布 case2_result
    case2_result_pub.publish(result);

}

void Case2FSM_UGV::detection_cb(const prometheus_msgs::ArucoInfoConstPtr &msg)
{
    // return过程中不检测
    if(in_return_mode)
    {
        return;
    }
    
    if(msg->detected)
    {
        num_count_vision_get++;
        num_count_vision_lost = 0;
    }else
    {
        num_count_vision_lost++;
        num_count_vision_get = 0;
    }

    //  方向定义： 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    // 注意，此处与无人车姿态相关！！
    // 无人车机体坐标系：前方x为正，左边y为正，上方z为正
    object_pos_body << msg->position[2], - msg->position[0], - msg->position[1];
    // cout << GREEN << ugv_name +" object_pos_body [X Y Z]: "<< object_pos_body  << TAIL <<endl;
}

// 主循环 
void Case2FSM_UGV::mainloop_cb(const ros::TimerEvent& e)
{
    static int exec_num=0;
    exec_num++;

    // 检查当前状态，不满足规划条件则直接退出主循环
    if(!odom_ready || !ugv_ready || !sensor_ready || !station_ready)
    {
        // 此处改为根据循环时间计算的数值
        if(exec_num == 100)
        {
            if(!odom_ready)
            {
                cout << YELLOW << ugv_name + " Case 2 Main loop init check: [ Need Odom ]."  << TAIL <<endl;
            }else if(!ugv_ready)
            {
                cout << YELLOW << ugv_name + " Case 2 Main loop init check: [ UGV is not ready ]."  << TAIL <<endl;
            }else if(!sensor_ready)
            {
                cout << YELLOW << ugv_name + " Case 2 Main loop init check: [ Need sensor info ]."  << TAIL <<endl;
            }else if(!station_ready)
            {
                cout << YELLOW << ugv_name + " Case 2 Main loop init check: [ Need station start cmd ]."  << TAIL <<endl;
            }
            exec_num=0;
        }  

        return;
    }else
    {
        // 对检查的状态进行重置,此处要求无人车状态、传感器信息回调频率要高于本定时器
        odom_ready = false;
        ugv_ready = false;
        if(map_input_source != 0)
        {
            sensor_ready = false;
        }

        if(exec_num >= 20)
        {
            // 状态打印
            printf_exec_state();
            exec_num=0;
        }  

    }
    
    switch (exec_state)
    {
        case EXEC_STATE::INIT:

            ros::Duration(1.0).sleep();
            // 起点位置设置为返航点
            return_pos = start_pos;
            cout << GREEN << ugv_name + " Case2: Return point is set as:" << return_pos(0) << ", "<< return_pos(1) << ", "<< return_pos(2) << TAIL <<endl; 
            exec_state = EXEC_STATE::WAIT_GOAL;
            break;

        case EXEC_STATE::WAIT_GOAL:
            // 等待目标点，不执行路径追踪逻辑
            path_ok = false;           
            
            // 手动目标点模式
            if(manual_mode)
            {
                // 等待手动输入的目标值
                if(!get_goal)
                {
                    if(exec_num == 100)
                    {
                        cout << YELLOW << ugv_name + " Case2: Waiting for a new goal, subscirbe to "<< ugv_name << "/prometheus/case2/goal" << TAIL <<endl;
                    }
                }else
                {
                    // 获取到目标点后，生成新轨迹
                    exec_state = EXEC_STATE::PLAN;
                    if(!sim_mode)
                    {
                        if(goal_pos(0)-ugv_odom.pose.pose.position.x>0) yaw_ref = 0.0;
                        else yaw_ref = 3.1415926;
                    }
                    get_goal = false;
                }
            }
            // 读取地面站的指令
            else
            {
                if(get_target_pos)
                {
                    // case3时，follow_target为false，此时仅有一个目标点，持续追踪
                    // case2 follow_target为true
                    // if(follow_target)
                    // {
                    //     get_target_pos = false;
                    // }
                    
                    goal_pos[0] = target_pos.pose.position.x;
                    goal_pos[1] = target_pos.pose.position.y;
                    goal_pos[2] = target_pos.pose.position.z;

                    if(!sim_mode)
                    {
                        if(goal_pos(0)-ugv_odom.pose.pose.position.x>0) yaw_ref = 0.0;
                        else yaw_ref = 3.1415926;
                    }
                    exec_state = EXEC_STATE::PLAN;
                    cout << GREEN << ugv_name + " Case2: Start PLAN" << TAIL <<endl;
                }else if(exec_num == 100)
                {
                    cout << YELLOW << ugv_name + " Case2: Waiting for target pos from station" << TAIL <<endl;
                }
            }

            break;
        
        case EXEC_STATE::PLAN:
            // 重置规划器
            Astar_ptr->reset();
            // 使用规划器执行搜索，返回搜索结果
            astar_state = Astar_ptr->search(start_pos, goal_pos);

            // 未寻找到路径
            if(astar_state==Astar::NO_PATH)
            {
                // 找不到路径：返回等待目标点，若在自动目标点模式，则会前往下一个目标点
                if(counter_search > 50)
                {
                    if(manual_mode)
                    {
                        path_ok = false;
                        exec_state = EXEC_STATE::WAIT_GOAL;
                    }else{
                        // backup goal to do
                        // exec_state = EXEC_STATE::RETURN;
                        if(sim_mode) exec_state = EXEC_STATE::WAIT_GOAL;
                    }

                    counter_search = 0;
                }
                counter_search++;
                cout << RED << ugv_name + " Case2: Main loop Planning [ Planner can't find path ]" << TAIL <<endl;
            }
            else
            {
                path_ok = true;
                counter_search = 0;
                path_cmd = Astar_ptr->get_ros_path();
                // 路径中航点数目
                Num_total_wp = path_cmd.poses.size();
                cur_id = 1;
                tra_start_time = ros::Time::now();
                // 路径规划成功，进入PATH_TRACKING
                exec_state = EXEC_STATE::PATH_TRACKING;
                // 发布路劲用于rviz显示
                path_cmd_pub.publish(path_cmd);
                // cout << GREEN << ugv_name + " Case2: Main loop Planning [ Get a new path ]" << TAIL <<endl;
            }

            break;
        
        case EXEC_STATE::PATH_TRACKING:
        
            // 执行时间达到阈值，重新执行一次规划
            if(get_time_in_sec(tra_start_time) >= replan_time)
            {
                if(in_return_mode)
                {
                    exec_state = EXEC_STATE::RETURN;
                }else
                {
                    exec_state = EXEC_STATE::PLAN;
                }
            }

            break;
        
        case EXEC_STATE::OBEJECT_TRACKING:
            
            path_ok = false;

            // 设定退出条件，return或降落
            // 也可以不设定，等待地面站消息

            break;

        case EXEC_STATE::RETURN:

            // 抵达返航点附近，降落
            if( (start_pos - return_pos).norm() < 0.2)
            {
                exec_state = EXEC_STATE::STOP;
                cout << GREEN << ugv_name + " stop 2."  << TAIL <<endl;
            }

            in_return_mode = true;

            if(!sim_mode)
            {
                if(return_pos(0)-ugv_odom.pose.pose.position.x>0) yaw_ref = 0.0;
                else yaw_ref = 3.1415926;
            }

            // 重置规划器
            Astar_ptr->reset();
            // 使用规划器执行搜索，返回搜索结果
            astar_state = Astar_ptr->search(start_pos, return_pos);
            // 未寻找到路径
            if(astar_state==Astar::NO_PATH)
            {
                // 找不到路径：返回等待目标点，若在自动目标点模式，则会前往下一个目标点
                if(counter_search > 50)
                {
                    exec_state = EXEC_STATE::STOP;
                    cout << RED << ugv_name + " Case2: Main loop RETURN [ Planner can't find path, STOP ]" << TAIL <<endl;
                }
                counter_search++;
            }
            else
            {
                path_ok = true;
                path_cmd = Astar_ptr->get_ros_path();
                Num_total_wp = path_cmd.poses.size();
                cur_id = 1;
                tra_start_time = ros::Time::now();
                exec_state = EXEC_STATE::PATH_TRACKING;
                path_cmd_pub.publish(path_cmd);
            }
            break;

        case EXEC_STATE::STOP:
            path_ok = false;
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode         = prometheus_msgs::UGVCommand::Hold;
            Command_Now.Command_ID   = Command_Now.Command_ID + 1;
            command_pub.publish(Command_Now);
            break;
    }

}

// 【获取当前时间函数】 单位：秒
float Case2FSM_UGV::get_time_in_sec(const ros::Time& begin_time)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin_time.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin_time.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void Case2FSM_UGV::printf_exec_state()
{
    switch (exec_state)
    {
        case EXEC_STATE::INIT: 
            cout << GREEN << ugv_name + " Case2: Main loop Exec_state: [ INIT ]."  << TAIL <<endl;
            break;
        case EXEC_STATE::WAIT_GOAL:
            cout << GREEN << ugv_name + " Case2: Main loop Exec_state: [ WAIT_GOAL ]."  << TAIL <<endl;
            break;
        case EXEC_STATE::PLAN: 
            cout << GREEN << ugv_name + " Case2: Main loop Exec_state: [ PLAN ]."  << TAIL <<endl;
            break;
        case EXEC_STATE::PATH_TRACKING:
            cout << GREEN << ugv_name + " Case2: Main loop Exec_state: [ PATH_TRACKING ]."  << TAIL <<endl;
            break;
        case EXEC_STATE::OBEJECT_TRACKING:
            cout << GREEN << ugv_name + " Case2: Main loop Exec_state: [ OBEJECT_TRACKING ]."  << TAIL <<endl;
            break;
        case EXEC_STATE::RETURN: 
            cout << GREEN << ugv_name + " Case2: Main loop Exec_state: [ RETURN ]."  << TAIL <<endl;
            break;
        case EXEC_STATE::STOP:
            cout << GREEN << ugv_name + " Case2: Main loop Exec_state: [ STOP ]."  << TAIL <<endl;
            break;  
    }    

    // if(detected_by_myself)
    // {
    //     cout << GREEN << "detected_by_myself, the target pos is:" << object_pos_enu[0] << object_pos_enu[1] << object_pos_enu[2] << TAIL <<endl;
    // }else if(detected_by_others)
    // {
    //     cout << GREEN << "detected_by_others, return."  << TAIL <<endl;
    // }else
    // {
    //     cout << GREEN << "no one find the target, keep searching."  << TAIL <<endl;
    // }
}



int Case2FSM_UGV::get_start_point_id(void)
{
    // 选择与当前无人车所在位置最近的点,并从该点开始追踪
    int id = 0;
    float distance_to_wp_min = abs(path_cmd.poses[0].pose.position.x - ugv_state.position[0])
                                + abs(path_cmd.poses[0].pose.position.y - ugv_state.position[1])
                                + abs(path_cmd.poses[0].pose.position.z - ugv_state.position[2]);
    
    float distance_to_wp;

    for(int j=1; j<Num_total_wp;j++)
    {
        distance_to_wp = abs(path_cmd.poses[j].pose.position.x - ugv_state.position[0])
                                + abs(path_cmd.poses[j].pose.position.y - ugv_state.position[1])
                                + abs(path_cmd.poses[j].pose.position.z - ugv_state.position[2]);
        
        if(distance_to_wp < distance_to_wp_min)
        {
            distance_to_wp_min = distance_to_wp;
            id = j;
        }
    }

    //　为防止出现回头的情况，此处对航点进行前馈处理
    if(id + 1 < Num_total_wp)
    {
        id = id + 1;
    }

    return id;
}

const int Case2FSM_UGV::get_track_point_id()
{
    // 选择与当前无人车所在位置最近的点,并从该点开始追踪
    int id = get_start_point_id();

    if (id == Num_total_wp-1){ // 如果已经是终点
        return id;
    }

    double dist_sum = 0;

    double next_dist = sqrt(pow((path_cmd.poses[id+1].pose.position.x - path_cmd.poses[id].pose.position.x), 2)
        + pow((path_cmd.poses[id+1].pose.position.y - path_cmd.poses[id].pose.position.y), 2));

    while(dist_sum + next_dist < 1.2){
        // std::cout << next_dist << std::endl;
        id++;
        dist_sum += next_dist;
        if (!(id+1 < Num_total_wp)){
            next_dist = sqrt(pow((path_cmd.poses[id+1].pose.position.x - path_cmd.poses[id].pose.position.x), 2)
                + pow((path_cmd.poses[id+1].pose.position.y - path_cmd.poses[id].pose.position.y), 2));
        }
        else{
            continue;
        }
    }

    return id;

}

}