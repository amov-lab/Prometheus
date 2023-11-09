#include "global_planner_ugv.h"

namespace global_planner_ugv
{
// 初始化函数
void GlobalPlannerUGV::init(ros::NodeHandle& nh)
{
    // 读取参数
    // 集群数量
    nh.param("global_planner_ugv/swarm_num_ugv", swarm_num_ugv, 1);
    // 无人车编号 1号无人车则为1
    nh.param("global_planner_ugv/ugv_id", ugv_id, 0);
    // 无人车高度
    nh.param("global_planner_ugv/ugv_height", ugv_height, 0.0);
    // 是否为仿真模式
    nh.param("global_planner_ugv/sim_mode", sim_mode, false); 
    // A星算法 重规划频率 
    nh.param("global_planner_ugv/replan_time", replan_time, 1.0); 
    nh.param("global_planner_ugv/track_frequency", track_frequency, 0.1); 
    // 手动给定目标点模式 或 自动目标点模式
    nh.param("global_planner_ugv/manual_mode", manual_mode, false);

    ugv_name = "/ugv" + std::to_string(ugv_id);

    // 选择地图更新方式：　0代表全局点云，1代表局部点云，2代表激光雷达scan数据
    nh.param("global_planner_ugv/map_input_source", map_input_source, 2); 
    //【订阅】 根据map_input选择地图更新方式
    if(map_input_source == 0)
    {
        cout << GREEN << "Global pcl mode, subscirbe to "<< ugv_name << "/prometheus/global_planner_ugv/global_pcl" << TAIL <<endl;
        Gpointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(ugv_name + "/prometheus/global_planner_ugv/global_pcl", 1, &GlobalPlannerUGV::Gpointcloud_cb, this);
    }else if(map_input_source == 1)
    {
        cout << GREEN << "Local pcl mode, subscirbe to "<< ugv_name << "/prometheus/global_planner_ugv/local_pcl" << TAIL <<endl;
        Lpointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(ugv_name + "/prometheus/global_planner_ugv/local_pcl", 1, &GlobalPlannerUGV::Lpointcloud_cb, this);
    }else if(map_input_source == 2)
    {
        cout << GREEN << "Laser scan mode, subscirbe to "<< ugv_name << "/prometheus/global_planner_ugv/laser_scan" << TAIL <<endl;
        laserscan_sub = nh.subscribe<sensor_msgs::LaserScan>(ugv_name + "/prometheus/global_planner_ugv/laser_scan", 1, &GlobalPlannerUGV::laser_cb, this);
    }

    if(manual_mode)
    {
        cout << GREEN << "Manual goal mode, subscirbe to "<< ugv_name << "/prometheus/global_planner_ugv/goal" << TAIL <<endl;
        // 【订阅】手动给定目标点模式 手动目标点
        goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(ugv_name + "/prometheus/global_planner_ugv/goal", 1, &GlobalPlannerUGV::goal_cb, this);
    }
    
    // 【订阅】无人车状态
    ugv_state_sub = nh.subscribe<prometheus_msgs::UGVState>(ugv_name + "/prometheus/ugv_state", 10, &GlobalPlannerUGV::ugv_state_cb, this);

    // 【订阅】其他无人车位置
    for(int i = 1; i <= swarm_num_ugv; i++)
    {
        if(i == ugv_id)
        {
            continue;
        }
        get_nei_odom[i] = false;
        odom_nei[i] << 99.9,99.9,99.9;
        nei_odom_sub[i] = nh.subscribe<nav_msgs::Odometry>("/ugv"+std::to_string(i)+"/prometheus/ugv_odom", 10, boost::bind(&GlobalPlannerUGV::nei_odom_cb,this,_1,i));
    }

    // 【地面站交互】地面站控制指令
    station_cmd_sub = nh.subscribe<prometheus_msgs::StationCommand>(ugv_name + "/ground_station/ugv_cmd", 1, &GlobalPlannerUGV::cmd_cb, this);
    

    // 【发布】 路径指令 （发送至swarm_controller.cpp）
    command_pub = nh.advertise<prometheus_msgs::UGVCommand>(ugv_name + "/prometheus/ugv_command", 1);
    // 【发布】路径用于显示（rviz显示）
    path_cmd_pub   = nh.advertise<nav_msgs::Path>(ugv_name + "/prometheus/global_planner_ugv/path_cmd",  1); 

    // 【定时器】主循环执行
    mainloop_timer = nh.createTimer(ros::Duration(0.1), &GlobalPlannerUGV::mainloop_cb, this);        
    // 【定时器】路径追踪
    track_path_timer = nh.createTimer(ros::Duration(track_frequency), &GlobalPlannerUGV::track_path_cb, this);        

    // 【定时器】更新其他无人车位置
    send_nei_odom_timer = nh.createTimer(ros::Duration(0.02), &GlobalPlannerUGV::send_nei_odom_cb, this); 

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

    // 初始化发布的指令
    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode  = prometheus_msgs::UGVCommand::Hold;
    Command_Now.Command_ID = 0;
}

void GlobalPlannerUGV::nei_odom_cb(const nav_msgs::Odometry::ConstPtr& odom, int id) 
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

void GlobalPlannerUGV::send_nei_odom_cb(const ros::TimerEvent& e)
{
    Astar_ptr->Occupy_map_ptr->ugv_pcl_update(odom_nei, get_nei_odom);
}

void GlobalPlannerUGV::goal_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    // 2D定高飞行
    goal_pos << msg->pose.position.x, msg->pose.position.y, ugv_height;
        
    goal_vel.setZero();

    get_goal = true;
    rotate_in_place = true;
    start_move = false;

    cout << GREEN << ugv_name + " Global_Planner_UGV: Get a new manual goal: ["<< goal_pos(0) << ", "  << goal_pos(1)  << ", "  << goal_pos(2) << " ]"  << TAIL <<endl;
}

void GlobalPlannerUGV::cmd_cb(const prometheus_msgs::StationCommandConstPtr& msg)
{
    // 将无人车ID与FLAG进行比较，判断是否执行本次任务
    if(msg->flag == 1 && msg->Command == prometheus_msgs::StationCommand::Start)
    {
        if(!station_ready)
        {
            station_ready = true;
            cout << GREEN << ugv_name + " Global_Planner_UGV : Get station start command." << TAIL <<endl; 
        }
        target_pos = msg->goal;
        target_pos.pose.position.z = ugv_height;
        get_target_pos = true;
        cout << GREEN << ugv_name +  " Global_Planner_UGV : Get target_pos from station command:"<<" ["<< target_pos.pose.position.x << ", "  << target_pos.pose.position.y  << ", "  << target_pos.pose.position.z<< " ]"  << TAIL <<endl;
    }

    if(msg->Command == prometheus_msgs::StationCommand::Return)
    {
        path_ok = false;
        exec_state = EXEC_STATE::RETURN;
        cout << GREEN << ugv_name + " Global_Planner_UGV: Get station_cmd: [ RETURN ]."  << TAIL <<endl;
        return;
    }else if(msg->Command == prometheus_msgs::StationCommand::Stop)
    {
        path_ok = false;
        exec_state = EXEC_STATE::STOP;
        cout << YELLOW << ugv_name + " Global_Planner_UGV: Get station_cmd: [ STOP ]."  << TAIL <<endl;
        return;
    }
}

void GlobalPlannerUGV::ugv_state_cb(const prometheus_msgs::UGVStateConstPtr& msg)
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
void GlobalPlannerUGV::Gpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
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
void GlobalPlannerUGV::Lpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
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
void GlobalPlannerUGV::laser_cb(const sensor_msgs::LaserScanConstPtr &msg)
{
    if (!odom_ready) 
    {
        return;
    }

    sensor_ready = true;
    // 对Astar中的地图进行更新（laser+odom）并对地图进行膨胀
    Astar_ptr->Occupy_map_ptr->map_update_laser(msg, ugv_odom);
}

void GlobalPlannerUGV::track_path_cb(const ros::TimerEvent& e)
{
    static int track_path_num = 0;
    
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
        cout << GREEN << ugv_name + " Global_Planner_UGV: Path tracking: [ Reach the goal ]."  << TAIL <<endl;
        
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
                        exec_state = EXEC_STATE::STOP;  
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
 
// 主循环 
void GlobalPlannerUGV::mainloop_cb(const ros::TimerEvent& e)
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
                cout << YELLOW << ugv_name + " Global_Planner_UGV Main loop init check: [ Need Odom ]."  << TAIL <<endl;
            }else if(!ugv_ready)
            {
                cout << YELLOW << ugv_name + " Global_Planner_UGV Main loop init check: [ UGV is not ready ]."  << TAIL <<endl;
            }else if(!sensor_ready)
            {
                cout << YELLOW << ugv_name + " Global_Planner_UGV Main loop init check: [ Need sensor info ]."  << TAIL <<endl;
            }else if(!station_ready)
            {
                cout << YELLOW << ugv_name + " Global_Planner_UGV Main loop init check: [ Need station start cmd ]."  << TAIL <<endl;
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
            cout << GREEN << ugv_name + " Global_Planner_UGV: Return point is set as:" << return_pos(0) << ", "<< return_pos(1) << ", "<< return_pos(2) << TAIL <<endl; 
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
                        cout << YELLOW << ugv_name + " Global_Planner_UGV: Waiting for a new goal, subscirbe to "<< ugv_name << "/prometheus/global_planner_ugv/goal" << TAIL <<endl;
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
                    
                    goal_pos[0] = target_pos.pose.position.x;
                    goal_pos[1] = target_pos.pose.position.y;
                    goal_pos[2] = target_pos.pose.position.z;

                    if(!sim_mode)
                    {
                        if(goal_pos(0)-ugv_odom.pose.pose.position.x>0) yaw_ref = 0.0;
                        else yaw_ref = 3.1415926;
                    }
                    exec_state = EXEC_STATE::PLAN;
                    cout << GREEN << ugv_name + " Global_Planner_UGV: Start PLAN" << TAIL <<endl;
                }else if(exec_num == 100)
                {
                    cout << YELLOW << ugv_name + " Global_Planner_UGV: Waiting for target pos from station" << TAIL <<endl;
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
                cout << RED << ugv_name + " Global_Planner_UGV: Main loop Planning [ Planner can't find path ]" << TAIL <<endl;
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
                // cout << GREEN << ugv_name + " Global_Planner_UGV: Main loop Planning [ Get a new path ]" << TAIL <<endl;
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
                    cout << RED << ugv_name + " Global_Planner_UGV: Main loop RETURN [ Planner can't find path, STOP ]" << TAIL <<endl;
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
float GlobalPlannerUGV::get_time_in_sec(const ros::Time& begin_time)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin_time.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin_time.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void GlobalPlannerUGV::printf_exec_state()
{
    switch (exec_state)
    {
        case EXEC_STATE::INIT: 
            cout << GREEN << ugv_name + " Global_Planner_UGV: Main loop Exec_state: [ INIT ]."  << TAIL <<endl;
            break;
        case EXEC_STATE::WAIT_GOAL:
            cout << GREEN << ugv_name + " Global_Planner_UGV: Main loop Exec_state: [ WAIT_GOAL ]."  << TAIL <<endl;
            break;
        case EXEC_STATE::PLAN: 
            cout << GREEN << ugv_name + " Global_Planner_UGV: Main loop Exec_state: [ PLAN ]."  << TAIL <<endl;
            break;
        case EXEC_STATE::PATH_TRACKING:
            cout << GREEN << ugv_name + " Global_Planner_UGV: Main loop Exec_state: [ PATH_TRACKING ]."  << TAIL <<endl;
            break;
        case EXEC_STATE::RETURN: 
            cout << GREEN << ugv_name + " Global_Planner_UGV: Main loop Exec_state: [ RETURN ]."  << TAIL <<endl;
            break;
        case EXEC_STATE::STOP:
            cout << GREEN << ugv_name + " Global_Planner_UGV: Main loop Exec_state: [ STOP ]."  << TAIL <<endl;
            break;  
    }    
}



int GlobalPlannerUGV::get_start_point_id(void)
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

const int GlobalPlannerUGV::get_track_point_id()
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