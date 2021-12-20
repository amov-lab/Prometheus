#include "fake_ugv.h"

void Fake_UGV::init(ros::NodeHandle& nh, int id, Eigen::Vector3d init_pos, double init_yaw)
{
    agent_id = id;
    model_name = "fake_ugv_" + std::to_string(agent_id);
    node_name = "fake_ugv" + std::to_string(agent_id);
    get_move_cmd = false;
    ugv_state_update = false;
    cmd_id = 0;
    block_size = 0.2;

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

    move_cmd_sub      = nh.subscribe<prometheus_drl::ugv_move_cmd>("/ugv"+std::to_string(agent_id) + "/move_cmd", 1, &Fake_UGV::move_cmd_cb, this);
    
    fake_odom_pub    = nh.advertise<nav_msgs::Odometry>("/ugv"+std::to_string(agent_id) + "/prometheus/fake_odom", 1);
    
    fake_odom_pub_timer = nh.createTimer(ros::Duration(0.05), &Fake_UGV::fake_odom_pub_cb, this);
    // debug_timer = nh.createTimer(ros::Duration(0.2), &Fake_UGV::debug_cb, this);

    cout << GREEN  << node_name << "---> Fake_UGV init sucess in position: " << init_pos[0] <<" [ m ] "<<init_pos[1] <<" [ m ] "<< TAIL <<endl;
}

void Fake_UGV::move_cmd_cb(const prometheus_drl::ugv_move_cmd::ConstPtr& msg)
{
    if(msg->ID <= cmd_id)
    {
        cout << RED << node_name << "---> wrong cmd id."<< TAIL << endl;
        return;
    }

    get_move_cmd = true;

    if(msg->CMD == prometheus_drl::ugv_move_cmd::HOLD)
    {

    }else if(msg->CMD == prometheus_drl::ugv_move_cmd::FORWARD)
    {
        ugv_state.pos[0] = ugv_state.pos[0] + block_size;
    }else if(msg->CMD == prometheus_drl::ugv_move_cmd::BACK)
    {
        ugv_state.pos[0] = ugv_state.pos[0] - block_size;
    }else if(msg->CMD == prometheus_drl::ugv_move_cmd::LEFT)
    {
        ugv_state.pos[1] = ugv_state.pos[1] + block_size;
    }else if(msg->CMD == prometheus_drl::ugv_move_cmd::RIGHT)
    {
        ugv_state.pos[1] = ugv_state.pos[1] - block_size;
    }else
    {
        cout << RED << node_name << "---> wrong move cmd."<< TAIL << endl;
    }
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


    // 发布TF用于RVIZ显示
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped tfs;
    //  |----头设置
    tfs.header.frame_id = "world";  //相对于世界坐标系
    tfs.header.stamp = ros::Time::now();  //时间戳
    
    //  |----坐标系 ID
    tfs.child_frame_id =  model_name + "/lidar_link";  //子坐标系，无人机的坐标系

    //  |----坐标系相对信息设置  偏移量  无人机相对于世界坐标系的坐标
    tfs.transform.translation.x = ugv_state.pos[0];
    tfs.transform.translation.y = ugv_state.pos[1];
    tfs.transform.translation.z = ugv_state.pos[2];
    //  |--------- 四元数设置  
    tfs.transform.rotation.x = fake_odom.pose.pose.orientation.x;
    tfs.transform.rotation.y = fake_odom.pose.pose.orientation.y;
    tfs.transform.rotation.z = fake_odom.pose.pose.orientation.z;
    tfs.transform.rotation.w = fake_odom.pose.pose.orientation.w;

    //  5-3.广播器发布数据
    broadcaster.sendTransform(tfs);

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

    cout << GREEN  << "ugv_state.pos [X Y Z]  : " << ugv_state.pos[0] << " [ m ] "<< ugv_state.pos[1]<<" [ m ] "<<ugv_state.pos[2]<<" [ m ] "<< TAIL <<endl;
}

Eigen::Vector3d Fake_UGV::get_ugv_pos()
{
    return ugv_state.pos;
}

gazebo_msgs::ModelState Fake_UGV::get_model_state()
{
    return gazebo_model_state;
}
