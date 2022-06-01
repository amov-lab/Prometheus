#include "fake_uav.h"

void Fake_UAV::init(ros::NodeHandle& nh, int id, Eigen::Vector3d init_pos, double init_yaw)
{
    agent_id = id;
    fake_mode = FAKE_MODE::POS_CONTROL_MODE;
    model_name = "fake_p230_" + std::to_string(agent_id);
    node_name = "fake_p230_" + std::to_string(agent_id);
    get_pos_cmd = false;
    get_att_cmd = false;
    get_ego_cmd = false;
    uav_state_update = false;

    // 初始化 uav_state
    uav_state.pos = init_pos;
    uav_state.vel << 0.0,0.0,0.0;
    uav_state.acc << 0.0,0.0,0.0;
    uav_state.euler << 0.0,0.0,init_yaw;
    uav_state.quat2 = quaternion_from_rpy(uav_state.euler);
    uav_state.quat.x = uav_state.quat2.x();
    uav_state.quat.y = uav_state.quat2.y();
    uav_state.quat.z = uav_state.quat2.z();
    uav_state.quat.w = uav_state.quat2.w();
    uav_state.Rb = uav_state.quat2.toRotationMatrix();
    // 初始化 gazebo_model_state
    gazebo_model_state.model_name = model_name;
    gazebo_model_state.pose.position.x = init_pos[0];
    gazebo_model_state.pose.position.y = init_pos[1];
    gazebo_model_state.pose.position.z = init_pos[2];
    gazebo_model_state.pose.orientation.x = uav_state.quat2.x();
    gazebo_model_state.pose.orientation.y = uav_state.quat2.y();
    gazebo_model_state.pose.orientation.z = uav_state.quat2.z();
    gazebo_model_state.pose.orientation.w = uav_state.quat2.w();
    gazebo_model_state.reference_frame = "ground_plane::link";

    // 初始化
    // quad.setStatePos(init_pos);

    // 干扰
    disturbance.f << 0.0,0.0,0.0;
    disturbance.m << 0.0,0.0,0.0;

    // 参数初始化
    control_param.quad_mass = 1.0;
    control_param.gravity   = 9.8;
    control_param.k_pos     = 0.8;
    control_param.k_vel     = 0.8;
    control_param.tilt_angle_max = 25.0;
    control_param.hover_per = 0.5;

    if(fake_mode == FAKE_MODE::POS_CONTROL_MODE)
    {
        delta_time = 0.02;
        pos_cmd_sub = nh.subscribe<mavros_msgs::PositionTarget>("/uav"+std::to_string(agent_id) + "/mavros/setpoint_raw/local", 1, &Fake_UAV::pos_cmd_cb, this);
    }else if(fake_mode == FAKE_MODE::ATT_CONTROL_MODE)
    {
        delta_time = 0.01;
        att_cmd_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("/uav"+std::to_string(agent_id) + "/mavros/setpoint_raw/attitude", 1, &Fake_UAV::att_cmd_cb, this);
    }
    
    // ego_cmd_sub     = nh.subscribe<quadrotor_msgs::PositionCommand>( "/uav"+std::to_string(agent_id) + "/prometheus/ego/traj_cmd", 1, &Fake_UAV::ego_cmd_cb, this);
    // 先发布fake_odom，在统一发布到gazebo 是因为gazebo不能同时处理太多信息
    fake_odom_pub    = nh.advertise<nav_msgs::Odometry>("/uav"+std::to_string(agent_id) + "/prometheus/fake_odom", 1);
    
    fake_odom_timer = nh.createTimer(ros::Duration(delta_time), &Fake_UAV::fake_odom_process, this);
    fake_odom_pub_timer = nh.createTimer(ros::Duration(0.02), &Fake_UAV::fake_odom_pub_cb, this);
    // debug_timer = nh.createTimer(ros::Duration(0.1), &Fake_UAV::debug_cb, this);

    cout << GREEN  << node_name << "---> Fake_UAV init sucess in position: " << init_pos[0] <<" [ m ] "<<init_pos[1] <<" [ m ] "<<init_pos[2] <<" [ m ] "<< TAIL <<endl;
}

 // 得到期望位置/期望速度/期望加速度+期望偏航角 统一转化为期望三轴推力+期望偏航角
void Fake_UAV::pos_cmd_cb(const mavros_msgs::PositionTarget::ConstPtr& msg)
{
    if(fake_mode != FAKE_MODE::POS_CONTROL_MODE)
    {
        return;
    }

    Eigen::Vector3d pos_error,vel_error;
    if(msg->type_mask == 0x4000)
    {
        control.u_cmd << 0.0, 0.0, 0.0;
    }else if(msg->type_mask == 0b100111111000)
    {
        // send_pos_setpoint + yaw
        pos_cmd.pos_sp << msg->position.x , msg->position.y , msg->position.z;
        pos_cmd.yaw_sp = msg->yaw;

        // 模拟基本串级控制
        pos_error = pos_cmd.pos_sp - uav_state.pos;
        vel_error  = control_param.k_pos * pos_error - uav_state.vel;
        control.u_cmd = control_param.k_vel * vel_error;
        control.yaw = pos_cmd.yaw_sp;
    }else if(msg->type_mask == 0b100111000111)
    {
        // send_vel_setpoint + yaw
        pos_cmd.vel_sp << msg->velocity.x , msg->velocity.y , msg->velocity.z;
        pos_cmd.yaw_sp = msg->yaw;

        vel_error  = pos_cmd.vel_sp - uav_state.vel;
        control.u_cmd = control_param.k_vel * vel_error;
        control.yaw = pos_cmd.yaw_sp;
    }else if(msg->type_mask == 0b100111000011)
    {
        // send_vel_xy_pos_z_setpoint + yaw
        pos_cmd.pos_sp << 0.0 , 0.0 , msg->position.z;
        pos_cmd.vel_sp << msg->velocity.x , msg->velocity.y , 0.0;
        pos_cmd.yaw_sp = msg->yaw;

        pos_error(2) = pos_cmd.pos_sp(2) - uav_state.pos(2);
        vel_error(0)  = pos_cmd.vel_sp(0) - uav_state.vel(0);
        vel_error(1)  = pos_cmd.vel_sp(1) - uav_state.vel(1);
        vel_error(2)  = control_param.k_pos * pos_error(2) - uav_state.vel(2);
        control.u_cmd = control_param.k_vel * vel_error;
        control.yaw = pos_cmd.yaw_sp;
    }else if(msg->type_mask == 0b100111000000)
    {
        // send_pos_vel_xyz_setpoint + yaw
        pos_cmd.pos_sp << msg->position.x , msg->position.y , msg->position.z;
        pos_cmd.vel_sp << msg->velocity.x , msg->velocity.y , msg->velocity.z;
        pos_cmd.yaw_sp = msg->yaw;        

        pos_error = pos_cmd.pos_sp - uav_state.pos;
        vel_error = pos_cmd.vel_sp + control_param.k_pos * pos_error - uav_state.vel;
        control.u_cmd = control_param.k_vel * vel_error;
        control.yaw = pos_cmd.yaw_sp;
    }else
    {
        cout << RED << node_name << "---> wrong pos_cmd type_mask."<< TAIL << endl;
        return;
    }
    
	// 期望力 = 质量*控制量 + 重力抵消 + 期望加速度*质量*Ka
    control.u_cmd(2) = control.u_cmd(2) + control_param.gravity;
	control.thrust_enu = control_param.quad_mass * control.u_cmd;
    
	// 如果向上推力小于重力的一半
	// 或者向上推力大于重力的两倍
	if (control.thrust_enu(2) < 0.5 * control_param.quad_mass * control_param.gravity)
	{
		control.thrust_enu = control.thrust_enu / control.thrust_enu(2) * (0.5 * control_param.quad_mass * control_param.gravity);
	}
	else if (control.thrust_enu(2) > 2 * control_param.quad_mass * control_param.gravity)
	{
		control.thrust_enu = control.thrust_enu / control.thrust_enu(2) * (2 * control_param.quad_mass * control_param.gravity);
	}

	// 角度限制幅度
	if (std::fabs(control.thrust_enu(0)/control.thrust_enu(2)) > std::tan(uav_utils::toRad(control_param.tilt_angle_max)))
	{
		control.thrust_enu(0) = control.thrust_enu(0)/std::fabs(control.thrust_enu(0)) * control.thrust_enu(2) * std::tan(uav_utils::toRad(control_param.tilt_angle_max));
	}

	// 角度限制幅度
	if (std::fabs(control.thrust_enu(1)/control.thrust_enu(2)) > std::tan(uav_utils::toRad(control_param.tilt_angle_max)))
	{
		control.thrust_enu(1) = control.thrust_enu(1)/std::fabs(control.thrust_enu(1)) * control.thrust_enu(2) * std::tan(uav_utils::toRad(control_param.tilt_angle_max));	
	}

    // thrust是位于ENU坐标系的,F_c是FLU
    Eigen::Matrix3d wRc = uav_utils::rotz(uav_state.euler(2));
    Eigen::Vector3d F_c = wRc.transpose() * control.thrust_enu;
    double fx = F_c(0);
    double fy = F_c(1);
    double fz = F_c(2);

    // 期望roll, pitch
    control.euler(0)  = std::atan2(-fy, fz);
    control.euler(1)  = std::atan2( fx, fz);
    control.euler(2)  = control.yaw;
    // 缺偏航角动态（位置控制认为直接赋值，euler_sp(2)已经在前面赋值过了）
    
    // thrust_enu_sp = control.thrust_enu;
    get_pos_cmd = true;
}

 // 得到期望姿态+期望油门 统一转化为期望三轴推力+期望偏航角
void Fake_UAV::att_cmd_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
{
    if(msg->type_mask != 0b00000111 )
    {
        cout << RED << node_name << "---> wrong att_cmd type_mask."<< TAIL << endl;
        return;
    }

    // 假设角度完全响应
    uav_state.quat = msg->orientation;
    uav_state.quat2.w() = msg->orientation.w;
    uav_state.quat2.x() = msg->orientation.x;
    uav_state.quat2.y() = msg->orientation.y;
    uav_state.quat2.z() = msg->orientation.z;   
    uav_state.Rb = uav_state.quat2.toRotationMatrix();
    att_cmd.euler_sp = quaternion_to_euler(uav_state.quat2);

    // 期望油门
    att_cmd.throttle_sp = msg->thrust;
    // 根据姿态和油门计算期望三轴推力（合力，不考虑重力）
    // 此处att_control计算得到的控制指令 是没有考虑速度的
    control.thrust_body << 0.0, 0.0, control_param.quad_mass*control_param.gravity* att_cmd.throttle_sp / control_param.hover_per;
    control.thrust_enu = uav_state.Rb * control.thrust_body;
    control.euler = att_cmd.euler_sp;
    get_att_cmd = true;
}

// 输入：thrust_enu_sp，euler_sp, 上一时刻的位姿
// 输出：fake_odom
void Fake_UAV::fake_odom_process(const ros::TimerEvent &e)
{
    // 如何合并下面三种情况
    // 针对位置环控制、位置环集群控制、简易避障等情况
    if(fake_mode == FAKE_MODE::POS_CONTROL_MODE)
    {
        if(!get_pos_cmd)
        {
            return;
        }

        // 假设推力可以直接响应
        uav_state.thrust_enu = control.thrust_enu;
        // 计算合力
        uav_state.thrust_enu(2) = uav_state.thrust_enu(2) - control_param.quad_mass*control_param.gravity;
        // 计算加速度,速度,位置
        uav_state.acc = uav_state.thrust_enu / control_param.quad_mass;
        uav_state.vel = uav_state.vel + uav_state.acc * delta_time;
        uav_state.pos = uav_state.pos + uav_state.vel * delta_time;

        // 假设姿态角可以直接响应 (无角度动态,可以考虑加入一个一阶滤波)
        uav_state.euler = control.euler;

        uav_state.quat2 = quaternion_from_rpy(uav_state.euler);
        uav_state.quat.x = uav_state.quat2.x();
        uav_state.quat.y = uav_state.quat2.y();
        uav_state.quat.z = uav_state.quat2.z();
        uav_state.quat.w = uav_state.quat2.w();
        uav_state.Rb = uav_state.quat2.toRotationMatrix();
    }else if(fake_mode == FAKE_MODE::ATT_CONTROL_MODE)
    {
        if(!get_att_cmd)
        {
            return;
        }
        // 假设姿态角可以直接响应 (无角度动态,可以考虑加入一个一阶滤波)
        uav_state.euler = control.euler;
        // 假设推力可以直接响应
        uav_state.thrust_enu = control.thrust_enu;
        // 计算合力
        uav_state.thrust_enu(2) = uav_state.thrust_enu(2) - control_param.quad_mass*control_param.gravity;
        // 计算加速度,速度,位置
        uav_state.acc = uav_state.thrust_enu / control_param.quad_mass;
        uav_state.vel = uav_state.vel + uav_state.acc * delta_time;
        uav_state.pos = uav_state.pos + uav_state.vel * delta_time;

        // 姿态已经更新过了

        // 模拟姿态环控制 输入：期望姿态角 输出：期望推力+力矩， todo
        // attitude_control();

        // 模拟混控器 输入：期望推力+力矩 输出：期望转速  todo
        // mixer();
    }else if(fake_mode == FAKE_MODE::RPM_CONTROL_MODE)
    {
        // 根据无人机动力学进行解算
        // // 设置输入
        // quad.setInput(control.rpm[0], control.rpm[1], control.rpm[2], control.rpm[3]);
        // // 设置外部干扰力
        // quad.setExternalForce(disturbance.f);
        // // 设置外部干扰力矩
        // quad.setExternalMoment(disturbance.m);
        // // 更新 - 得到odom
        // quad.step(delta_time);
    }

    // 模拟地面
    if (uav_state.pos(2) < 0.0 && uav_state.vel(2) < 0)
    {
        uav_state.pos(2) = 0.0;
        uav_state.vel(2) = 0.0;
    }

    uav_state_update = true;
}

void Fake_UAV::attitude_control()
{
    
}
void Fake_UAV::mixer()
{
    
}


Eigen::Vector3d Fake_UAV::get_uav_pos()
{
    return uav_state.pos;
}

gazebo_msgs::ModelState Fake_UAV::get_model_state()
{
    return gazebo_model_state;
}

void Fake_UAV::fake_odom_pub_cb(const ros::TimerEvent &e)
{
    // 发布fake odom
    fake_odom.header.stamp = ros::Time::now();
    fake_odom.header.frame_id = "world";
    fake_odom.child_frame_id = "base_link";
    fake_odom.pose.pose.position.x = uav_state.pos[0];
    fake_odom.pose.pose.position.y = uav_state.pos[1];
    fake_odom.pose.pose.position.z = uav_state.pos[2];
    fake_odom.pose.pose.orientation = uav_state.quat;
    fake_odom.twist.twist.linear.x = uav_state.vel[0];
    fake_odom.twist.twist.linear.y = uav_state.vel[1];
    fake_odom.twist.twist.linear.z = uav_state.vel[2];
    fake_odom_pub.publish(fake_odom);

    // 更新 gazebo_model_state
    // 注意：这个话题发布的是相对位置，且无法移除初始位置的影响（因此将所有无人机初始位置设为0）
    // 注意：他这个坐标转换是先转角度再加位移
    gazebo_model_state.model_name = model_name;
    gazebo_model_state.pose.position.x = uav_state.pos[0];
    gazebo_model_state.pose.position.y = uav_state.pos[1];
    gazebo_model_state.pose.position.z = uav_state.pos[2];
    gazebo_model_state.pose.orientation = uav_state.quat;
    gazebo_model_state.reference_frame = "ground_plane::link";
}

void Fake_UAV::debug_cb(const ros::TimerEvent &e)
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Fake Odom <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    // cout << GREEN  << "UAV_id : " <<  agent_id << "   UAV_name : " <<  uav_name << TAIL << endl;
    // cout << GREEN  << "pos_control_type : " <<  pos_control_type  << TAIL << endl;
    // cout << GREEN  << "pos_sp [X Y Z]         : " << pos_sp[0] << " [ m ] "<< pos_sp[1]<<" [ m ] "<<pos_sp[2]<<" [ m ] "<< TAIL <<endl;
    // cout << GREEN <<  "vel_sp  [X Y Z]         : " << vel_sp[0] << " [m/s] "<< vel_sp[1]<<" [m/s] "<<vel_sp[2]<<" [m/s] "<< TAIL <<endl;
    // cout << GREEN <<  "euler_sp                   : " << euler_sp[0] << " [rad] "<< euler_sp[1]<<" [rad] "<<euler_sp[2]<<" [rad] "<< TAIL <<endl;
    // cout << GREEN <<  "throttle_sp[0-1]    : " << throttle_sp<< TAIL <<endl;
    // cout << GREEN  << "uav_state.pos [X Y Z]    : " << uav_state.pos[0] << " [ m ] "<< uav_state.pos[1]<<" [ m ] "<<uav_state.pos[2]<<" [ m ] "<< TAIL <<endl;
    // cout << GREEN <<  "uav_state.vel  [X Y Z]    : " << uav_state.vel[0] << " [m/s] "<< uav_state.vel[1]<<" [m/s] "<<uav_state.vel[2]<<" [m/s] "<< TAIL <<endl;
    // cout << GREEN <<  "uav_state.euler              : " << uav_state.euler[0] << " [rad] "<< uav_state.euler[1]<<" [rad] "<<uav_state.euler[2]<<" [rad] "<< TAIL <<endl;


}

// void Fake_UAV::ego_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
// {
//     if(msg->velocity.x == 0 && msg->velocity.y == 0)
//     {
//         get_ego_cmd = false;
//     }else
//     {
//         get_pos_cmd = false;
//         get_ego_cmd = true;
//         uav_state_update = true;

//         uav_state.pos(0) = msg->position.x;
//         uav_state.pos(1) = msg->position.y;
//         uav_state.pos(2) = msg->position.z;
//         uav_state.vel(0) = msg->velocity.x;
//         uav_state.vel(1) = msg->velocity.y;
//         uav_state.vel(2) = msg->velocity.z;
//         uav_state.acc(0) = msg->acceleration.x;
//         uav_state.acc(1) = msg->acceleration.y;
//         uav_state.acc(2) = msg->acceleration.z;
//         // 根据加速度计算姿态角
//         Eigen::Vector3d alpha = Eigen::Vector3d(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z) + 9.8*Eigen::Vector3d(0,0,1);
//         Eigen::Vector3d xC(cos(msg->yaw), sin(msg->yaw), 0);
//         Eigen::Vector3d yC(-sin(msg->yaw), cos(msg->yaw), 0);
//         Eigen::Vector3d xB = (yC.cross(alpha)).normalized();
//         Eigen::Vector3d yB = (alpha.cross(xB)).normalized();
//         Eigen::Vector3d zB = xB.cross(yB);
//         uav_state.Rb.col(0) = xB;
//         uav_state.Rb.col(1) = yB;
//         uav_state.Rb.col(2) = zB;
//         Eigen::Quaterniond q(uav_state.Rb);
//         uav_state.quat2 = q;
//         uav_state.euler = quaternion_to_euler(uav_state.quat2);
//         uav_state.quat.x = uav_state.quat2.x();
//         uav_state.quat.y = uav_state.quat2.y();
//         uav_state.quat.z = uav_state.quat2.z();
//         uav_state.quat.w = uav_state.quat2.w();
//     }
// }