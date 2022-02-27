#include "fake_ugv.h"

void Fake_UGV::init(ros::NodeHandle& nh, int id, Eigen::Vector3d init_pos, double init_yaw)
{
    agent_id = id;
    car_model == CAR_MODEL::POINT;
    model_name = "cxy_fake_ugv_" + std::to_string(agent_id);
    node_name = "fake_ugv" + std::to_string(agent_id);
    get_vel_cmd = false;
    ugv_state_update = false;
    delta_time = 0.02;

    // 初始化 ugv_state
    ugv_state.pos = init_pos;
    ugv_state.vel << 0.0,0.0,0.0;
    ugv_state.euler << 0.0,0.0,init_yaw;
    ugv_state.quat2 = quaternion_from_rpy(ugv_state.euler);
    ugv_state.quat.x = ugv_state.quat2.x();
    ugv_state.quat.y = ugv_state.quat2.y();
    ugv_state.quat.z = ugv_state.quat2.z();
    ugv_state.quat.w = ugv_state.quat2.w();
    // 初始化 gazebo_model_state
    gazebo_model_state.model_name = model_name;
    gazebo_model_state.pose.position.x = init_pos[0];
    gazebo_model_state.pose.position.y = init_pos[1];
    gazebo_model_state.pose.position.z = init_pos[2];
    ugv_state.euler << 0.0,0.0,init_yaw;
    ugv_state.quat2 = quaternion_from_rpy(ugv_state.euler);
    gazebo_model_state.pose.orientation.x = ugv_state.quat2.x();
    gazebo_model_state.pose.orientation.y = ugv_state.quat2.y();
    gazebo_model_state.pose.orientation.z = ugv_state.quat2.z();
    gazebo_model_state.pose.orientation.w = ugv_state.quat2.w();
    gazebo_model_state.reference_frame = "ground_plane::link";

    // 参数初始化
    control_param.k_pos     = 0.2;
    control_param.k_vel     = 0.4;

    vel_cmd_sub      = nh.subscribe<geometry_msgs::Twist>("/ugv"+std::to_string(agent_id) + "/cmd_vel", 1, &Fake_UGV::vel_cmd_cb, this);
    
    fake_odom_pub    = nh.advertise<nav_msgs::Odometry>("/ugv"+std::to_string(agent_id) + "/prometheus/fake_odom", 1);
    
    fake_odom_timer = nh.createTimer(ros::Duration(delta_time), &Fake_UGV::fake_odom_process, this);
    fake_odom_pub_timer = nh.createTimer(ros::Duration(0.02), &Fake_UGV::fake_odom_pub_cb, this);
    // debug_timer = nh.createTimer(ros::Duration(0.2), &Fake_UGV::debug_cb, this);

    cout << GREEN  << node_name << "---> Fake_UGV init sucess in position: " << init_pos[0] <<" [ m ] "<<init_pos[1] <<" [ m ] "<< TAIL <<endl;
}

void Fake_UGV::vel_cmd_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    get_vel_cmd = true;

    vel_sp_body(0) = msg->linear.x;
    vel_sp_body(1) = msg->linear.y;
    vel_sp_body(2) = msg->linear.z;

    // 机体系转惯性系 （全向轮）
    vel_sp_enu[0] = vel_sp_body(0) * cos(ugv_state.euler[2]) - vel_sp_body(1) * sin(ugv_state.euler[2]);
    vel_sp_enu[1] = vel_sp_body(0) * sin(ugv_state.euler[2]) + vel_sp_body(1) * cos(ugv_state.euler[2]);
    vel_sp_enu[2] = 0.0;

    yaw_rate_sp = msg->angular.z; 
}

void Fake_UGV::fake_odom_process(const ros::TimerEvent &e)
{
    if(!get_vel_cmd)
    {
        return;
    }

    if(car_model == CAR_MODEL::POINT)
    {
        ugv_state.euler[0] = 0.0;
        ugv_state.euler[1] = 0.0;
        ugv_state.euler[2] = ugv_state.euler[2] + yaw_rate_sp * delta_time;
        // 收到的cmd_vel速度是机体系 系数减速
        ugv_state.vel = control_param.k_vel * vel_sp_enu;
        ugv_state.pos = ugv_state.pos + ugv_state.vel * delta_time;
    }else if(car_model == CAR_MODEL::DIFFERENTIAL_WHELL)
    {
        ugv_state.euler[0] = 0.0;
        ugv_state.euler[1] = 0.0;
        ugv_state.euler[2] = ugv_state.euler[2] + yaw_rate_sp * delta_time;
        ugv_state.vel = control_param.k_vel * vel_sp_enu;//机体系
        ugv_state.pos[0] = ugv_state.pos[0] + ugv_state.vel[0] * cos(ugv_state.euler[2]) * delta_time;
        ugv_state.pos[1] = ugv_state.pos[1] + ugv_state.vel[1] * sin(ugv_state.euler[2]) * delta_time;
        ugv_state.pos[2] = 0.0;
    }else
    {
        cout << RED << node_name << "---> wrong car_model."<< TAIL << endl;
    }

    // 计算姿态角(无角度动态,可以考虑加入一个一阶滤波)
    ugv_state.quat2 = quaternion_from_rpy(ugv_state.euler);
    ugv_state.quat.x = ugv_state.quat2.x();
    ugv_state.quat.y = ugv_state.quat2.y();
    ugv_state.quat.z = ugv_state.quat2.z();
    ugv_state.quat.w = ugv_state.quat2.w();

    ugv_state_update = true;
}

Eigen::Vector3d Fake_UGV::get_ugv_pos()
{
    return ugv_state.pos;
}

gazebo_msgs::ModelState Fake_UGV::get_model_state()
{
    return gazebo_model_state;
}

void Fake_UGV::fake_odom_pub_cb(const ros::TimerEvent &e)
{
    // 发布fake odom
    fake_odom.header.stamp = ros::Time::now();
    fake_odom.header.frame_id = "world";
    fake_odom.child_frame_id = "base_link";
    fake_odom.pose.pose.position.x = ugv_state.pos[0];
    fake_odom.pose.pose.position.y = ugv_state.pos[1];
    fake_odom.pose.pose.position.z = ugv_state.pos[2];
    fake_odom.pose.pose.orientation = ugv_state.quat;
    fake_odom.twist.twist.linear.x = ugv_state.vel[0];
    fake_odom.twist.twist.linear.y = ugv_state.vel[1];
    fake_odom.twist.twist.linear.z = ugv_state.vel[2];
    fake_odom_pub.publish(fake_odom);

    // 更新 gazebo_model_state
    // 注意：这个话题发布的是相对位置，且无法移除初始位置的影响（因此将所有无人机初始位置设为0）
    // 注意：他这个坐标转换是先转角度再加位移
    gazebo_model_state.model_name = model_name;
    gazebo_model_state.pose.position.x = ugv_state.pos[0];
    gazebo_model_state.pose.position.y = ugv_state.pos[1];
    gazebo_model_state.pose.position.z = ugv_state.pos[2];
    gazebo_model_state.pose.orientation = ugv_state.quat;
    gazebo_model_state.reference_frame = "ground_plane::link";
}

void Fake_UGV::debug_cb(const ros::TimerEvent &e)
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

    cout << GREEN <<  "vel_sp_enu  [X Y Z]   : " << vel_sp_enu[0] << " [m/s] "<< vel_sp_enu[1]<<" [m/s] "<<vel_sp_enu[2]<<" [m/s] "<< TAIL <<endl;
    cout << GREEN <<  "yaw_rate_sp       : " << yaw_rate_sp << " [rad/s] "<< TAIL <<endl;
    cout << GREEN  << "ugv_state.pos [X Y Z]  : " << ugv_state.pos[0] << " [ m ] "<< ugv_state.pos[1]<<" [ m ] "<<ugv_state.pos[2]<<" [ m ] "<< TAIL <<endl;
    cout << GREEN <<  "ugv_state.vel  [X Y Z] : " << ugv_state.vel[0] << " [m/s] "<< ugv_state.vel[1]<<" [m/s] "<<ugv_state.vel[2]<<" [m/s] "<< TAIL <<endl;
    cout << GREEN <<  "ugv_state.euler        : " << ugv_state.euler[0] << " [rad] "<< ugv_state.euler[1]<<" [rad] "<<ugv_state.euler[2]<<" [rad] "<< TAIL <<endl;
}

