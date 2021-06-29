#include "ugv_controller.h"

using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_controller");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);

    init(nh);

    //【订阅】无人车控制指令
    command_sub = nh.subscribe<prometheus_msgs::UgvCommand>(ugv_name + "/prometheus/ugv_command", 10, ugv_command_cb);
    //【订阅】本机状态信息
    ugv_state_sub = nh.subscribe<prometheus_msgs::UgvState>(ugv_name + "/prometheus/ugv_state", 10, ugv_state_cb);

    // vinson: 订阅yaw速度
    odom_sub = nh.subscribe<nav_msgs::Odometry>(ugv_name + "/odom", 10, odom_cb);

    if(sim_mode)
    {
        //【发布】用于地面站显示的提示消息
        turtlebot_cmd_pub = nh.advertise<geometry_msgs::Twist>(ugv_name + "/cmd_vel", 10);
    }else
    {
        //【服务】解锁/上锁
        arming_client = nh.serviceClient<mavros_msgs::CommandBool>(ugv_name + "/mavros/cmd/arming");
        //【服务】修改系统模式
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(ugv_name + "/mavros/set_mode");
        //【发布】efk origin
        ekf_origin_pub = nh.advertise<geographic_msgs::GeoPointStamped>(ugv_name + "/mavros/global_position/set_gp_origin", 10);
        //【发布】位置/速度/加速度期望值 坐标系 ENU系
        setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>(ugv_name + "/mavros/setpoint_raw/local", 10);
        //【发布】body系vel
        vel_body_pub = nh.advertise<geometry_msgs::Twist>(ugv_name + "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    }

    if(flag_printf)
    {
        printf_param();
    }

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        // 执行回调函数
        ros::spinOnce();

        // 一般选择不打印
        if(flag_printf)
        {
            printf_state();
        }

        // Check for geo fence: If ugv is out of the geo fence, it will hold now.
        if(check_failsafe() == 1)
        {
            Command_Now.Mode = prometheus_msgs::UgvCommand::Hold;
        }

        switch (Command_Now.Mode)
        {
        // 【Idle】 怠速旋转，此时可以切入offboard模式，但不会起飞。
        case prometheus_msgs::UgvCommand::Start:
            
            if(sim_mode)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.linear.z = 0.0;

                cmd_vel.angular.x = 0.0;
                cmd_vel.angular.y = 0.0;
                cmd_vel.angular.z = 0.0;

                turtlebot_cmd_pub.publish(cmd_vel);
            }else
            {
                // 设定linear_vel = 999时，切换offboard模式，并解锁
                if(Command_Now.linear_vel[0] == 999)
                {
                    if(!_UgvState.guided)
                    {
                        // 设定EKF origin(APM固件必须设定ekf orgin，随意指定经纬度即可)
                        geographic_msgs::GeoPointStamped origin;
                        origin.position.latitude = 0.0;
                        origin.position.longitude = 0.0;
                        origin.position.altitude = 0.0;
                        ekf_origin_pub.publish(origin);

                        sleep(0.5);

                        mode_cmd.request.custom_mode = "GUIDED";
                        set_mode_client.call(mode_cmd);
                        ROS_INFO ("\033[1;32m---->Setting to GUIDED Mode....\033[0m");
                    }else
                    {
                        // 成功进入GUIDED模式后，自动转入Hold
                        Command_Now.Mode = prometheus_msgs::UgvCommand::Hold;
                    }

                    cmd_vel.linear.x = 0.0;
                    cmd_vel.linear.y = 0.0;
                    cmd_vel.linear.z = 0.0;

                    cmd_vel.angular.x = 0.0;
                    cmd_vel.angular.y = 0.0;
                    cmd_vel.angular.z = 0.0;

                    vel_body_pub.publish(cmd_vel);
                }
            }

            break;

        case prometheus_msgs::UgvCommand::Hold:

            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;

            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = 0.0;

            if(sim_mode)
            {
                turtlebot_cmd_pub.publish(cmd_vel);
            }else
            {
                vel_body_pub.publish(cmd_vel);
            }
            
            break;

        case prometheus_msgs::UgvCommand::Disarm:

            if(!sim_mode)
            {
                ROS_INFO_STREAM_ONCE ("\033[1;32m---->Disarm: switch to MANUAL flight mode...\033[0m");
                if(_UgvState.mode == "GUIDED")
                {
                    mode_cmd.request.custom_mode = "MANUAL";
                    set_mode_client.call(mode_cmd);
                }

                if(_UgvState.armed)
                {
                    arm_cmd.request.value = false;
                    arming_client.call(arm_cmd);
                }
            }else
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.linear.z = 0.0;

                cmd_vel.angular.x = 0.0;
                cmd_vel.angular.y = 0.0;
                cmd_vel.angular.z = 0.0;
                turtlebot_cmd_pub.publish(cmd_vel);
            }

            break;

        case prometheus_msgs::UgvCommand::Point_Control:

            // 与目标距离
            dist = sqrt(pow( Command_Now.position_ref[0] - pos_ugv[0] , 2) + pow( Command_Now.position_ref[1] - pos_ugv[1] , 2));
            // 目标点方向角（弧度）
            direct = atan2(Command_Now.position_ref[1] - pos_ugv[1], Command_Now.position_ref[0] - pos_ugv[0]);

            yaw_error = Command_Now.yaw_ref - yaw_ugv; 

            if(only_rotate)
            {
                if(dist < 0.05)
                {
                    cmd_vel.linear.x = 0.0;
                    //到达目标点附近，原地旋转
                    if(abs(yaw_error) < 0.1)
                    {
                        cmd_vel.angular.z = 0.0;
                    }else
                    {
                        cmd_vel.angular.z = k_yaw * yaw_error;
                    }
                }else 
                {   
                    if (abs((direct - yaw_ugv)) < 0.1)
                    {
                        only_rotate = false;
                        cmd_vel.linear.x = 0.0;
                        cmd_vel.angular.z = 0.0;
                    }else
                    {
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = k_yaw *(direct - yaw_ugv);
                    }
                }
            }else
            {
                if(dist < 0.05)
                {
                    only_rotate = true;
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                }else
                {
                    cmd_vel.linear.x = k_p * dist;
                    cmd_vel.angular.z = k_yaw *(direct - yaw_ugv);
                }
            }

            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;

            if(sim_mode)
            {
                turtlebot_cmd_pub.publish(cmd_vel);
            }else
            {
                vel_body_pub.publish(cmd_vel);
            }

            break;

        case prometheus_msgs::UgvCommand::Direct_Control:
            // 这个相当于机体系速度控制
            cmd_vel.linear.x = Command_Now.linear_vel[0];
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;

            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = Command_Now.angular_vel;

            if(sim_mode)
            {
                turtlebot_cmd_pub.publish(cmd_vel);
            }else
            {
                vel_body_pub.publish(cmd_vel);
            }
            break;

        case prometheus_msgs::UgvCommand::ENU_Vel_Control:
            // 这个相当于惯性系速度控制
            state_sp = Eigen::Vector3d(Command_Now.linear_vel[0],Command_Now.linear_vel[1],0.0);

            if(sim_mode)
            {
                // 不支持
            }else
            {
                send_vel_setpoint(state_sp, Command_Now.yaw_ref);
            }
            break;

        case prometheus_msgs::UgvCommand::Path_Control:

            // 与目标距离
            dist = sqrt(pow( Command_Now.position_ref[0] - pos_ugv[0] , 2) + pow( Command_Now.position_ref[1] - pos_ugv[1] , 2));
            // 目标点方向角（弧度）
            direct = atan2(Command_Now.position_ref[1] - pos_ugv[1], Command_Now.position_ref[0] - pos_ugv[0]);
            yaw_error = angles::shortest_angular_distance(yaw_ugv, Command_Now.yaw_ref);    // use shortest dist

            if (dist<0.5){
                stop_flag = true;
            }

            if (stop_flag && abs(yaw_error)<0.1){ // 已经到达目标
                cmd_vel.linear.x = cmd_vel.angular.z = 0.0;
            }
            else if (stop_flag){     // 距离上到达了
                if (first_arrive && sqrt(pow(vel_ugv[0] , 2) + pow(vel_ugv[1] , 2) + pow(vel_ugv[2] , 2)) > 0.01){ //刚到达，需要停下
                    cmd_vel.linear.x = cmd_vel.angular.z = 0;
                }
                else    // 已经停下来了
                {
                    first_arrive = false;
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = k_yaw * yaw_error;
                    sm.smooth(0, _odom.twist.twist.angular.z, cmd_vel.linear.x, cmd_vel.angular.z);

                }
            }
            else{   // 在移动
                cmd_vel.linear.x = k_p*dist;
                cmd_vel.angular.z = k_yaw *angles::shortest_angular_distance(yaw_ugv, direct);
                sm.smooth(sqrt(pow(vel_ugv[0] , 2) + pow(vel_ugv[1] , 2)), _odom.twist.twist.angular.z, cmd_vel.linear.x, cmd_vel.angular.z);
            }

            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;

            if(sim_mode)
            {
                turtlebot_cmd_pub.publish(cmd_vel);
            }else
            {
                vel_body_pub.publish(cmd_vel);
            }

            break;
        }

        Command_Last = Command_Now;


        rate.sleep();
    }
}