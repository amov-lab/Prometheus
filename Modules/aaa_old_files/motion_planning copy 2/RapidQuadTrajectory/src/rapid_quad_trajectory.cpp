#include "rapid_quad_trajectory.h"

// 本部分代码基于 https://github.com/markwmuller/RapidQuadrocopterTrajectories 二次开发
// 论文信息：
// 作者：M.W. Mueller, M. Hehn, and R. D'Andrea
// 题目：A computationally efficient motion primitive for quadrocopter trajectory generation
// 期刊：IEEE Transactions on Robotics

// todo:
// 1,阅读论文,理解概念
// 2,阅读源码,是否支持多个路径点的设置
// 3,与控制联调(需要使用轨迹追踪控制器)
// 4,加入高飞的轨迹生成,作为补充
RapidQuadTrajectory::RapidQuadTrajectory(ros::NodeHandle &nh)
{
    // 【参数】编号，从1开始编号
    nh.param("uav_id", uav_id, 0);
    // 【参数】平均速度
    nh.param("velocity_mean", velocity_mean, 0.0);
    // 【参数】平均速度
    nh.param("replan_time", replan_time, 10.0);

    //　能用，但距离使用还差前端　和　后端约束

    //【订阅】目标点
    goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav" + std::to_string(uav_id) + "/prometheus/planning/goal",
                                                        1,
                                                        &RapidQuadTrajectory::goal_cb, this);

    //【订阅】无人机状态信息
    uav_state_sub = nh.subscribe<prometheus_msgs::UAVState>("/uav" + std::to_string(uav_id) + "/prometheus/drone_state",
                                                            1,
                                                            &RapidQuadTrajectory::uav_state_cb, this);
    // 【发布】控制指令
    uav_cmd_pub = nh.advertise<prometheus_msgs::UAVCommand>("/uav" + std::to_string(uav_id) + "/prometheus/command", 1);

    optimal_path_pub = nh.advertise<nav_msgs::Path>("/uav" + std::to_string(uav_id) + "/prometheus/motion_planning/ref_trajectory", 10);

    // 【定时器】状态机主循环
    mainloop_timer = nh.createTimer(ros::Duration(1.5), &RapidQuadTrajectory::mainloop_cb, this);
    // 【定时器】路径规划结果发布（轨迹追踪）
    trajectory_tracking_timer = nh.createTimer(ros::Duration(0.01), &RapidQuadTrajectory::trajectory_tracking_cb, this);
    // 【定时器】debug
    debug_timer = nh.createTimer(ros::Duration(0.2), &RapidQuadTrajectory::debug_cb, this);

    // 规划器状态参数初始化
    exec_state = EXEC_STATE::WAIT_GOAL;
    odom_ready = false;
    drone_ready = false;
    goal_ready = false;

    // 初始化发布的指令
    uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
    uav_command.position_ref[0] = 0;
    uav_command.position_ref[1] = 0;
    uav_command.position_ref[2] = 0;
    uav_command.yaw_ref = 0;
}

void RapidQuadTrajectory::goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal_pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    goal_ready = true;

    posf = Vec3(goal_pos[0], goal_pos[1], goal_pos[2]);
    velf = Vec3(0.0, 0.0, 0.0);
    accf = Vec3(0, 0, 0); //acceleration

    cout << GREEN << NODE_NAME << "Get a new goal point:" << goal_pos(0) << " [m] " << goal_pos(1) << " [m] " << goal_pos(2) << " [m] " << TAIL << endl;

    if (goal_pos(0) == 99 && goal_pos(1) == 99)
    {
        path_ok = false;
        goal_ready = false;
        exec_state = EXEC_STATE::LANDING;
        cout << GREEN << NODE_NAME << "Land " << TAIL << endl;
    }
}

void RapidQuadTrajectory::uav_state_cb(const prometheus_msgs::UAVState::ConstPtr &msg)
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
    uav_odom.pose.pose.position.z = uav_state.position[2];

    uav_odom.pose.pose.orientation = uav_state.attitude_q;
    uav_odom.twist.twist.linear.x = uav_state.velocity[0];
    uav_odom.twist.twist.linear.y = uav_state.velocity[1];
    uav_odom.twist.twist.linear.z = uav_state.velocity[2];

    // 更新无人车初始位置、速度、加速度，用于规划
    pos0 = Vec3(uav_state.position[0], uav_state.position[1], uav_state.position[2]);
    vel0 = Vec3(uav_state.velocity[0], uav_state.velocity[1], uav_state.velocity[2]);
    acc0 = Vec3(0, 0, 0); //acceleration

    if (goal_ready)
    {
        distance_to_goal = (Eigen::Vector3d(uav_state.position[0], uav_state.position[1], uav_state.position[2]) - goal_pos).norm();
    }
}

void RapidQuadTrajectory::generate_trajectory()
{
    double fmin = 5;          //[m/s**2]
    double fmax = 25;         //[m/s**2]
    double wmax = 20;         //[rad/s]
    double minTimeSec = 0.02; //[s]

    //Define the state constraints. We'll only check that we don't fly into the floor:
    Vec3 floorPos = Vec3(0, 0, 0);    //any point on the boundary
    Vec3 floorNormal = Vec3(0, 0, 1); //we want to be in this direction of the boundary

    // 定义轨迹生成类，并且存入初始位置、速度、加速度和重力
    RapidTrajectoryGenerator traj(pos0, vel0, acc0, gravity);
    // 设置目标位置、速度、加速度
    traj.SetGoalPosition(posf);
    traj.SetGoalVelocity(velf);
    traj.SetGoalAcceleration(accf);

    // Note: if you'd like to leave some states free, you can encode it like below.
    // Here we would be leaving the velocity in `x` (axis 0) free:
    //
    // traj.SetGoalVelocityInAxis(1,velf[1]);
    // traj.SetGoalVelocityInAxis(2,velf[2]);

    //　根据目标点重新预估时间,希望平均速度为 velocity_mean
    total_time = sqrt(pow(posf[0] - pos0[0], 2) + pow(posf[1] - pos0[1], 2) + pow(posf[2] - pos0[2], 2)) / velocity_mean;

    // 生成轨迹，total_time为期望时间
    traj.Generate(total_time);

    for (int i = 0; i < 3; i++)
    {
        Alpha[i] = traj.GetAxisParamAlpha(i);
        Beta[i] = traj.GetAxisParamBeta(i);
        Gamma[i] = traj.GetAxisParamGamma(i);
    }

    cout << GREEN << "Generate a new trajectory!" << TAIL << endl;
    cout << GREEN << "Total_time =  " << total_time << " [s] " << TAIL << endl;
    cout << GREEN << "Total cost = " << traj.GetCost() << TAIL << endl;

    // 这里改成轨迹是否可行的判断依据
    cout << GREEN << "Input feasible? " << GetInputFeasibilityResultName(traj.CheckInputFeasibility(fmin, fmax, wmax, minTimeSec)) << TAIL << endl;
    cout << GREEN << "Position feasible? " << GetStateFeasibilityResultName(traj.CheckPositionFeasibility(floorPos, floorNormal)) << TAIL << endl;

    pub_optimal_path();

    // 得到新轨迹，轨迹时间清零
    time_now = 0.0;
}

//Two simple helper function to make testing easier
const char* RapidQuadTrajectory::GetInputFeasibilityResultName(RapidTrajectoryGenerator::InputFeasibilityResult fr)
{
    switch(fr)
    {
    case RapidTrajectoryGenerator::InputFeasible:             return "Feasible";
    case RapidTrajectoryGenerator::InputIndeterminable:       return "Indeterminable";
    case RapidTrajectoryGenerator::InputInfeasibleThrustHigh: return "InfeasibleThrustHigh";
    case RapidTrajectoryGenerator::InputInfeasibleThrustLow:  return "InfeasibleThrustLow";
    case RapidTrajectoryGenerator::InputInfeasibleRates:      return "InfeasibleRates";
    }
    return "Unknown!";
}

const char* RapidQuadTrajectory::GetStateFeasibilityResultName(RapidTrajectoryGenerator::StateFeasibilityResult fr)
{
    switch(fr)
    {
    case RapidTrajectoryGenerator::StateFeasible:   return "Feasible";
    case RapidTrajectoryGenerator::StateInfeasible: return "Infeasible";
    }
    return "Unknown!";
}


void RapidQuadTrajectory::trajectory_tracking_cb(const ros::TimerEvent &e)
{
    if (!path_ok)
    {
        return;
    }

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

        uav_command.yaw_ref = 0.0;
        uav_cmd_pub.publish(uav_command);

        cout << GREEN << NODE_NAME << "Reach the goal, HOLD! " << TAIL << endl;
        // 停止执行
        path_ok = false;
        // 转换状态为等待目标
        exec_state = EXEC_STATE::WAIT_GOAL;
        return;
    }
    else if (time_now >= total_time)
    {
        uav_command.header.stamp = ros::Time::now();
        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
        uav_command.Command_ID = uav_command.Command_ID + 1;
        uav_cmd_pub.publish(uav_command);

        cout << GREEN << NODE_NAME << "Time out, HOLD! " << TAIL << endl;
        // 停止执行
        path_ok = false;
        // 转换状态为等待目标
        exec_state = EXEC_STATE::WAIT_GOAL;
        return;
    }

    uav_command.header.stamp = ros::Time::now();
    uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
    uav_command.Command_ID = uav_command.Command_ID + 1;
    uav_command.Move_mode = prometheus_msgs::UAVCommand::TRAJECTORY;
    for (int i = 0; i < 3; i++)
    {
        //时间戳怎么解决？
        uav_command.position_ref[i] = 1 / 120.0 * Alpha[i] * pow(time_now, 5) + 1 / 24.0 * Beta[i] * pow(time_now, 4) + 1 / 6.0 * Gamma[i] * pow(time_now, 3) + 1 / 2.0 * acc0[i] * pow(time_now, 2) + vel0[i] * time_now + pos0[i];
        uav_command.velocity_ref[i] = 1 / 24.0 * Alpha[i] * pow(time_now, 4) + 1 / 6.0 * Beta[i] * pow(time_now, 3) + 1 / 2.0 * Gamma[i] * pow(time_now, 2) + acc0[i] * time_now + vel0[i];
        uav_command.acceleration_ref[i] = 1 / 6.0 * Alpha[i] * pow(time_now, 3) + 1 / 2.0 * Beta[i] * pow(time_now, 2) + Gamma[i] * time_now + acc0[i];
    }

    uav_command.yaw_ref = 0.0;
    time_now = time_now + 0.01;
    uav_cmd_pub.publish(uav_command);
}

void RapidQuadTrajectory::debug_cb(const ros::TimerEvent &e)
{
    cout << GREEN << NODE_NAME << "Rapid Quad Trajectory Generator." << TAIL << endl;

    if (exec_state == TRACKING)
    {
        cout << GREEN << "goal_pos: " << goal_pos[0] << " [m] " << goal_pos[1] << " [m] " << goal_pos[2] << " [m] " << TAIL << endl;
        cout << GREEN << "Trajectory tracking: " << time_now << " / " << total_time << " [ s ]" << TAIL << endl;
        cout << GREEN << "desired_point: "
             << uav_command.position_ref[0] << " [m] "
             << uav_command.position_ref[1] << " [m] "
             << uav_command.position_ref[2] << " [m] " << TAIL << endl;
    }
}

void RapidQuadTrajectory::mainloop_cb(const ros::TimerEvent &e)
{
    static int exec_num = 0;
    exec_num++;
    // 检查当前状态，不满足规划条件则直接退出主循环
    // 此处打印消息与后面的冲突了，逻辑上存在问题
    if (!odom_ready || !drone_ready)
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
            exec_num = 0;
        }

        return;
    }
    else
    {
        // 对检查的状态进行重置
        odom_ready = false;
        drone_ready = false;
    }

    switch (exec_state)
    {
    case WAIT_GOAL:
        path_ok = false;
        if (!goal_ready)
        {
            if (exec_num == 10)
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
    case PLANNING:

        generate_trajectory();
        if (path_ok)
        {
            exec_state = EXEC_STATE::TRACKING;
        }
        else
        {
            exec_state = EXEC_STATE::WAIT_GOAL;
            cout << RED << NODE_NAME << "Planner can't find path!" << TAIL << endl;
        }

        break;
    case TRACKING:

        // 本循环是1Hz,此处不是很精准
        if (exec_num >= replan_time)
        {
            exec_state = EXEC_STATE::PLANNING;
            exec_num = 0;
        }
        break;

    case LANDING:
        uav_command.header.stamp = ros::Time::now();
        uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Land;
        uav_command.Command_ID = uav_command.Command_ID + 1;

        uav_cmd_pub.publish(uav_command);
        break;
    }
}

void RapidQuadTrajectory::pub_optimal_path()
{
    optimal_path.header.frame_id = "world";
    optimal_path.header.stamp = ros::Time::now();
    optimal_path.poses.clear();

    time_now = 0.0;
    int k = 0;
    while (time_now < total_time)
    {
        geometry_msgs::PoseStamped way_point;
        way_point.header.frame_id = "world";
        way_point.pose.position.x = 1 / 120.0 * Alpha[0] * pow(time_now, 5) + 1 / 24.0 * Beta[0] * pow(time_now, 4) + 1 / 6.0 * Gamma[0] * pow(time_now, 3) + 1 / 2.0 * acc0[0] * pow(time_now, 2) + vel0[0] * time_now + pos0[0];
        way_point.pose.position.y = 1 / 120.0 * Alpha[1] * pow(time_now, 5) + 1 / 24.0 * Beta[1] * pow(time_now, 4) + 1 / 6.0 * Gamma[1] * pow(time_now, 3) + 1 / 2.0 * acc0[1] * pow(time_now, 2) + vel0[1] * time_now + pos0[1];
        way_point.pose.position.z = 1 / 120.0 * Alpha[2] * pow(time_now, 5) + 1 / 24.0 * Beta[2] * pow(time_now, 4) + 1 / 6.0 * Gamma[2] * pow(time_now, 3) + 1 / 2.0 * acc0[2] * pow(time_now, 2) + vel0[2] * time_now + pos0[2];
        optimal_path.poses.push_back(way_point);
        time_now = time_now + 0.01;
    }

    optimal_path_pub.publish(optimal_path);
}
