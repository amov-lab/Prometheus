#include "swarm_planner.h"

namespace Swarm_Planning
{

// 初始化函数
void Swarm_Planner::init(ros::NodeHandle& nh)
{
    // 读取参数
    // TRUE代表2D平面规划及搜索,FALSE代表3D 
    nh.param("swarm_planner/is_2D", is_2D, true); 
    // 2D规划时,定高高度
    nh.param("swarm_planner/fly_height_2D", fly_height_2D, 1.0);  
    // 安全距离，若膨胀距离设置已考虑安全距离，建议此处设为0
    nh.param("swarm_planner/safe_distance", safe_distance, 0.25); 
    // 执行每段路径时间，与地图分辨率相关 无人机移动速度 = 地图分辨率/时间
    nh.param("swarm_planner/time_per_path", time_per_path, 1.0); 
    // 选择地图更新方式,0为全局点云，1为2维激光雷达，等等
    nh.param("swarm_planner/map_input", map_input, 0); 


    // 订阅 目标点
    goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/prometheus/swarm_planning/goal", 1, &Swarm_Planner::goal_cb, this);

    // 订阅 无人机状态
    drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, &Swarm_Planner::drone_state_cb, this);

    // 根据map_input选择地图更新方式
    if(map_input == 0)
    {
        Gpointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/prometheus/swarm_planning/global_pcl", 1, &Swarm_Planner::Gpointcloud_cb, this);
    }else if(map_input == 1)
    {
        Lpointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/prometheus/swarm_planning/local_pcl", 1, &Swarm_Planner::Lpointcloud_cb, this);
    }else if(map_input == 2)
    {
        laserscan_sub = nh.subscribe<sensor_msgs::LaserScan>("/prometheus/swarm_planning/laser_scan", 1, &Swarm_Planner::laser_cb, this);
    }    

    // 发布 路径指令
    command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);
    // 发布提示消息
    message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/swarm_planning", 10);
    // 发布 地图rviz显示
    global_map_marker_pub = nh.advertise<visualization_msgs::Marker>("/prometheus/swarm_planning/global_map_marker",  10);  

    // 定时器 安全检测
    safety_timer = nh.createTimer(ros::Duration(2.0), &Swarm_Planner::safety_cb, this);
    // 定时器 规划器算法执行周期，快速移动场景应当适当提高执行频率
    // 该时间应当小于time_per_path，保证在控制周期发送合适的路径点控制信息
    mainloop_timer = nh.createTimer(ros::Duration(0.5), &Swarm_Planner::mainloop_cb, this);        

    // 设置全局点云
    sensor_msgs::PointCloud2ConstPtr init_global_map(new sensor_msgs::PointCloud2());
    global_map_ptr_ = init_global_map;

    // Astar algorithm
    Astar_ptr.reset(new Astar);
    Astar_ptr->init(nh);

    // 规划器状态参数初始化
    exec_state = EXEC_STATE::INIT;

    odom_ready = false;
    drone_ready = false;
    goal_ready = false;
    sensor_ready = false;
    is_safety = false;
    is_new_path = false;

    // 初始化发布的指令
    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
    Command_Now.Command_ID = 0;
    Command_Now.source = NODE_NAME;
    desired_yaw = 0.0;

    // init visualization
    visualization_.reset(new PlanningVisualization(nh));
    vis_resolution = 0.1;
    vis_goal_color = Eigen::Vector4d(1.0, 0, 0, 1);
    vis_path_color = Eigen::Vector4d(1.0, 0, 0, 1);

}

void Swarm_Planner::goal_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if (is_2D == true)
    {
        goal_pos << msg->pose.position.x, msg->pose.position.y, fly_height_2D;
    }else
    {
        goal_pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    }
        
    goal_vel.setZero();

    ROS_INFO("---global planning_: get waypoint: [ %f, %f, %f]!---", goal_pos(0),
                                                            goal_pos(1), 
                                                            goal_pos(2));

    visualization_->drawGoal(goal_pos, 3*vis_resolution, vis_goal_color, 1);

    goal_ready = true;

    // 获得新目标点
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME,"Get a new goal point");
}

void Swarm_Planner::drone_state_cb(const prometheus_msgs::DroneStateConstPtr& msg)
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
}

void Swarm_Planner::laser_cb(const sensor_msgs::LaserScanConstPtr &msg)
{
    // sensor_msgs::LaserScan
    // /* need odom_ for center radius sensing */
    // if (!drone_ready) 
    // {
    //     pub_message(message_pub, prometheus_msgs::Message::WARM, NODE_NAME, "Drone is not ready.");
    //     return;
    // }

    // pcl::fromROSMsg(*msg, latest_global_pcl_);
    // has_point_map_ = true;

    // global_map_ptr_ = msg;

    // // 对Astar中的地图进行更新
    // Astar_ptr->map_update(global_map_ptr_);

    // visualization_msgs::Marker m;
    // getOccupancyMarker(m, 0, Eigen::Vector4d(0, 0.5, 0.5, 1.0));
    // global_map_marker_Pub.publish(m);
}

void Swarm_Planner::Gpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    /* need odom_ for center radius sensing */
    if (!odom_ready) 
    {
        return;
    }

    pcl::fromROSMsg(*msg, latest_global_pcl_);
    
    sensor_ready = true;

    global_map_ptr_ = msg;

    // 对Astar中的地图进行更新
    Astar_ptr->map_update(global_map_ptr_);

    visualization_msgs::Marker m;
    getOccupancyMarker(m, 0, Eigen::Vector4d(0, 0.5, 0.5, 1.0));
    global_map_marker_pub.publish(m);
}

void Swarm_Planner::Lpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    /* need odom_ for center radius sensing */
    if (!odom_ready) 
    {
        return;
    }

    pcl::fromROSMsg(*msg, latest_global_pcl_);
    
    sensor_ready = true;

    global_map_ptr_ = msg;

    // 对Astar中的地图进行更新
    Astar_ptr->map_update(global_map_ptr_);

    visualization_msgs::Marker m;
    getOccupancyMarker(m, 0, Eigen::Vector4d(0, 0.5, 0.5, 1.0));
    global_map_marker_pub.publish(m);
}
 
// 主循环 
// 不一定要1.5秒就重新规划一次
// 以及如何确定多久重新规划一次
void Swarm_Planner::mainloop_cb(const ros::TimerEvent& e)
{
    static int exec_num=0;
    exec_num++;

    // 检查当前状态，不满足规划条件则直接退出主循环
    // 此处打印消息与后面的冲突了，逻辑上存在问题
    if(!odom_ready || !drone_ready || !sensor_ready || !goal_ready)
    {
        // 此处改为根据循环时间计算的数值
        if(exec_num == 2)
        {
            if(!odom_ready)
            {
                message = "Need Odom.";
            }else if(!drone_ready)
            {
                message = "Drone is not ready.";
            }else if(!sensor_ready)
            {
                message = "Waiting for sensor topic.";
            }else if(!goal_ready)
            {
                message = "Waiting for goal.";
            }

            pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
            exec_num=0;
        }  

        return;
    }

    switch (exec_state)
    {
        case INIT:
        {
            if(!drone_ready){
                message = "Drone is not ready.";
            }else if(!sensor_ready)
            {
                message = "Waiting for sensor topic.";
            }

            pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME,message);
            // 初始化状态后，转为等待目标点状态
            exec_state = EXEC_STATE::WAIT_GOAL;

            break;
        }
        case WAIT_GOAL:
        {
            if(!goal_ready)
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

                message = "Waiting for a new goal.";
                pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME,message);

            }else
            {
                // 获取到目标点后，生成新轨迹
                exec_state = EXEC_STATE::GEN_NEW_TRAJ;
            }
            
            break;
        }
        case GEN_NEW_TRAJ:
        {
            // 重置规划器
            Astar_ptr->reset();
            // 使用规划器执行搜索，返回搜索结果
            int astar_state = Astar_ptr->search(start_pos, goal_pos);
            // 未寻找到路径
            if(astar_state==Astar::NO_PATH)
            {
                goal_ready = false;
                exec_state = EXEC_STATE::WAIT_GOAL;
                pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Planner can't find path!");
            }
            else
            {
                // 若规划成功，则切换为执行轨迹
                exec_state = EXEC_STATE::EXEC_TRAJ;

                tra_start_time = ros::Time::now();

                pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Planner find path success!");

                path_cmd = Astar_ptr->get_ros_path();

                Num_total_wp = path_cmd.poses.size();

                is_new_path = true;
                
                // 可视化路径
                visualization_->drawrosPath(path_cmd, vis_resolution, vis_path_color, 1); 
            }
            break;
        }
        case EXEC_TRAJ:
        {
            distance_to_goal = sqrt(  pow(_DroneState.position[0] - goal_pos[0], 2) 
                            + pow(_DroneState.position[1] - goal_pos[1], 2) 
                            + pow(_DroneState.position[2] - goal_pos[2], 2));

            // 抵达目标点附近后（MIN_DIS可在全局变量中修改），无人机将直接悬停于目标点
            if(distance_to_goal < MIN_DIS)
            {
                exec_state = EXEC_STATE::EXEC_TRAJ;
                break;
            }else
            {
                // 生成控制指令，可否直接发送给控制模块？
                generate_CMD(path_cmd);
            }


            // 根据时间或者当前状态信息判断是否进行重规划
            if( 0 == 1)
            {
                exec_state = EXEC_STATE::GEN_NEW_TRAJ;
            }

            break;


        }

    }


}

void Swarm_Planner::generate_CMD(nav_msgs::Path path)
{
    if(is_safety)
    {
        // 若无人机与障碍物之间的距离小于安全距离，则停止执行路径
        // 但如何脱离该点呢？
        pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Drone Position Dangerous! STOP HERE and wait for new goal.");
        
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode         = prometheus_msgs::ControlCommand::Hold;
        Command_Now.Command_ID   = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;

        command_pub.publish(Command_Now);

        goal_ready = false;
        exec_state = EXEC_STATE::WAIT_GOAL;
        
        return;
    }

    // 如果是重新规划的轨迹，则先计算距离当前位置最近的路径点
    if(is_new_path)
    {
        is_new_path = false;
        
        // 选择与当前无人机所在位置最近的点,并从该点开始追踪
        start_point_index = 0;
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
                start_point_index = j;
            }
        }
    }

    // 计算距离开始追踪轨迹时间
    tra_running_time = get_time_in_sec(tra_start_time);

    int i = floor(tra_running_time / time_per_path) + start_point_index;
    

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
}

// 【获取当前时间函数】 单位：秒
float Swarm_Planner::get_time_in_sec(const ros::Time& begin_time)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin_time.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin_time.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void Swarm_Planner::safety_cb(const ros::TimerEvent& e)
{
    Eigen::Vector3d cur_pos(_DroneState.position[0], _DroneState.position[1], _DroneState.position[2]);
    
    is_safety = Astar_ptr->check_safety(cur_pos, safe_distance);
}

// 显示点云marker
// 待补充
// 考虑将其移动至显示专用class中，并且对参数进行调节
void Swarm_Planner::getOccupancyMarker(visualization_msgs::Marker &m, int id, Eigen::Vector4d color) 
{
    m.header.frame_id = "map";
    m.id = id;
    m.type = visualization_msgs::Marker::CUBE_LIST;
    m.action = visualization_msgs::Marker::MODIFY;
    m.scale.x = 0.1; // resolustion
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.color.a = color(3);
    m.color.r = color(0);
    m.color.g = color(1);
    m.color.b = color(2);

    // iterate the map
    pcl::PointXYZ pt;
    for (size_t i = 0; i < latest_global_pcl_.points.size(); ++i) {
        pt = latest_global_pcl_.points[i];

        geometry_msgs::Point p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        m.points.push_back(p);
    }
}





}