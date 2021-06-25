#include <swarm_ego_controller.h>

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_ego_controller");
    ros::NodeHandle nh("~");
    ros::Rate rate(100.0);

    // 集群数量
    nh.param<int>("swarm_num", swarm_num, 1);
    // 无人机编号 1号无人机则为1
    nh.param<int>("uav_id", uav_id, 0);
    uav_name = "/uav" + std::to_string(uav_id);

    nh.param<float>("quad_mass" , quad_mass, 1.0f);
    nh.param<float>("hov_percent" , hov_percent, 0.4f);
    nh.param<float>("Limit/pxy_int_max"  , int_max[0], 1.0);
    nh.param<float>("Limit/pxy_int_max"  , int_max[1], 1.0);
    nh.param<float>("Limit/pz_int_max"   , int_max[2], 1.0);
    nh.param<double>("hover_gain/Kp_xy", Kp_hover(0,0), 2.0f);
    nh.param<double>("hover_gain/Kp_xy", Kp_hover(1,1), 2.0f);
    nh.param<double>("hover_gain/Kp_z" , Kp_hover(2,2), 2.0f);
    nh.param<double>("hover_gain/Kv_xy", Kv_hover(0,0), 2.0f);
    nh.param<double>("hover_gain/Kv_xy", Kv_hover(1,1), 2.0f);
    nh.param<double>("hover_gain/Kv_z" , Kv_hover(2,2), 2.0f);
    nh.param<double>("hover_gain/Kvi_xy", Kvi_hover(0,0), 0.3f);
    nh.param<double>("hover_gain/Kvi_xy", Kvi_hover(1,1), 0.3f);
    nh.param<double>("hover_gain/Kvi_z" , Kvi_hover(2,2), 0.3f);
    nh.param<double>("hover_gain/Ka_xy", Ka_hover(0,0), 1.0f);
    nh.param<double>("hover_gain/Ka_xy", Ka_hover(1,1), 1.0f);
    nh.param<double>("hover_gain/Ka_z" , Ka_hover(2,2), 1.0f);
    nh.param<float>("hover_gain/tilt_angle_max" , tilt_angle_max_hover, 20.0f);

    nh.param<double>("track_gain/Kp_xy", Kp_track(0,0), 2.0f);
    nh.param<double>("track_gain/Kp_xy", Kp_track(1,1), 2.0f);
    nh.param<double>("track_gain/Kp_z" , Kp_track(2,2), 2.0f);
    nh.param<double>("track_gain/Kv_xy", Kv_track(0,0), 2.0f);
    nh.param<double>("track_gain/Kv_xy", Kv_track(1,1), 2.0f);
    nh.param<double>("track_gain/Kv_z" , Kv_track(2,2), 2.0f);
    nh.param<double>("track_gain/Kvi_xy", Kvi_track(0,0), 0.3f);
    nh.param<double>("track_gain/Kvi_xy", Kvi_track(1,1), 0.3f);
    nh.param<double>("track_gain/Kvi_z" , Kvi_track(2,2), 0.3f);
    nh.param<double>("track_gain/Ka_xy", Ka_track(0,0), 1.0f);
    nh.param<double>("track_gain/Ka_xy", Ka_track(1,1), 1.0f);
    nh.param<double>("track_gain/Ka_z" , Ka_track(2,2), 1.0f);
    nh.param<float>("track_gain/tilt_angle_max" , tilt_angle_max_track, 20.0f);

    drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>(uav_name + "/prometheus/drone_state", 10, drone_state_cb);
    odom_sub = nh.subscribe<nav_msgs::Odometry>(uav_name + "/prometheus/drone_odom", 10, drone_odom_cb);
    cmd_sub  = nh.subscribe<prometheus_msgs::SwarmCommand>(uav_name + "/prometheus/swarm_command", 10, cmd_cb);

    setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>(uav_name + "/mavros/setpoint_raw/local", 10);
    setpoint_raw_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>(uav_name +  "/mavros/setpoint_raw/attitude", 10);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>(uav_name + "/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(uav_name + "/mavros/set_mode");

    ros::Timer debug_timer = nh.createTimer(ros::Duration(10.0), debug_cb);

    init();

    while(ros::ok())
    {
        // 执行回调函数
        ros::spinOnce();

        switch (Command_Now.Mode)
        {
        // 【Idle】 怠速旋转，此时可以切入offboard模式，但不会起飞。
        case prometheus_msgs::SwarmCommand::Idle:
            
            idle();

            // 设定yaw_ref=999时，切换offboard模式，并解锁
            if(Command_Now.yaw_ref == 999)
            {
                if(_DroneState.mode != "OFFBOARD")
                {
                    mode_cmd.request.custom_mode = "OFFBOARD";
                    set_mode_client.call(mode_cmd);
                    ROS_INFO_STREAM_ONCE ("\033[1;32m---->Setting to OFFBOARD Mode....\033[0m");
                }
                
                if(!_DroneState.armed)
                {
                    arm_cmd.request.value = true;
                    arming_client.call(arm_cmd);
                    ROS_INFO_STREAM_ONCE ("\033[1;32m---->Arming....\033[0m");
                }
            }

            break;

        // 【Takeoff】 从摆放初始位置原地起飞至指定高度，偏航角也保持当前角度    
        case prometheus_msgs::SwarmCommand::Takeoff:

            // 设定起飞点
            if (Command_Last.Mode != prometheus_msgs::SwarmCommand::Takeoff)
            {
                Takeoff_position = pos_drone;
                pos_des[0] = pos_drone[0];
                pos_des[1] = pos_drone[1];
                pos_des[2] = pos_drone[2] + 1.0;
                vel_des << 0.0, 0.0, 0.0;
                acc_des << 0.0, 0.0, 0.0;
                yaw_des    = yaw_drone;
            }
            break;

        // 【Hold】 悬停。当前位置悬停
        case prometheus_msgs::SwarmCommand::Hold:

            if (Command_Last.Mode != prometheus_msgs::SwarmCommand::Hold)
            {
                // 设定起飞位置
                pos_des = pos_drone;
                vel_des << 0.0, 0.0, 0.0;
                acc_des << 0.0, 0.0, 0.0;
                yaw_des = yaw_drone;
            }
            break;

        case prometheus_msgs::SwarmCommand::Land:

            if (Command_Last.Mode != prometheus_msgs::SwarmCommand::Land)
            {
                // 设定起飞位置
                pos_des[0] = pos_drone[0];
                pos_des[1] = pos_drone[1];
                pos_des[2] = 0.0;
                vel_des << 0.0, 0.0, -0.2;
                acc_des << 0.0, 0.0, 0.0;
                yaw_des = yaw_drone;
                ROS_INFO ("---->Landing....");
            }

            //如果距离起飞高度小于30厘米，则直接切换为land模式；
            if( pos_drone[2] <= (Takeoff_position(2) + 0.2f))
            {
                mode_cmd.request.custom_mode = "MANUAL";
                set_mode_client.call(mode_cmd);

                arm_cmd.request.value = false;
                arming_client.call(arm_cmd);
                ROS_INFO_STREAM_ONCE ("---->Landed....");
            }

            break;

        case prometheus_msgs::SwarmCommand::Move:
            // 若收到轨迹控制指令，则按轨迹飞行
            // 否则，按照定点控制
            if(Command_Now.Move_mode  == prometheus_msgs::SwarmCommand::TRAJECTORY)
            {
                for(int i=0; i<3; i++)
                {
                    pos_des(i) = Command_Now.position_ref[i];
                    vel_des(i) = Command_Now.velocity_ref[i];
                    acc_des(i) = Command_Now.acceleration_ref[i];
                }
                yaw_des = Command_Now.yaw_ref;
                
            }else
            {
                pos_des[0] = Command_Now.position_ref[0];
                pos_des[1] = Command_Now.position_ref[1];
                pos_des[2] = Command_Now.position_ref[2];
                vel_des << 0.0, 0.0, 0.0;
                acc_des << 0.0, 0.0, 0.0;
                yaw_des = Command_Now.yaw_ref;
            }
            break;

        case prometheus_msgs::SwarmCommand::Disarm:

            mode_cmd.request.custom_mode = "MANUAL";
            set_mode_client.call(mode_cmd);
            arm_cmd.request.value = false;
            arming_client.call(arm_cmd);
            
            break;
        }

        if( Command_Now.Mode == prometheus_msgs::SwarmCommand::Takeoff ||
            Command_Now.Mode == prometheus_msgs::SwarmCommand::Hold    ||
            Command_Now.Mode == prometheus_msgs::SwarmCommand::Land    ||
            Command_Now.Mode == prometheus_msgs::SwarmCommand::Move   )
        {
            pos_controller();

            // 计算控制
            send_attitude_setpoint(u_att);
        }


        Command_Last = Command_Now;
        rate.sleep();
    }

}