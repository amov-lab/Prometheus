#include <fmt_test.h>



void fmt_test::fmt_state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    uav_state.connected = msg->connected;
    uav_state.armed = msg->armed;
    uav_state.mode = msg->mode;
    uav_state_update = true;
}
void fmt_test::fmt_battery_cb(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    uav_state.battery_state = msg->voltage;
    uav_state.battery_percetage = msg->percentage;
}
void fmt_test::fmt_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav_state.position[0] = msg->pose.position.x;
    uav_state.position[1] = msg->pose.position.y;
    uav_state.position[2] = msg->pose.position.z;
}
void fmt_test::fmt_global_rel_alt_cb(const std_msgs::Float64::ConstPtr &msg)
{
    uav_state.rel_alt = msg->data;
}
void fmt_test::fmt_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    uav_state.velocity[0] = msg->twist.linear.x;
    uav_state.velocity[1] = msg->twist.linear.y;
    uav_state.velocity[2] = msg->twist.linear.z;
}
void fmt_test::fmt_att_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    Eigen::Quaterniond q_fcu = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    Eigen::Vector3d euler_fcu = quaternion_to_euler(q_fcu);

    uav_state.attitude_q = msg->orientation;
    uav_state.attitude[0] = euler_fcu[0];
    uav_state.attitude[1] = euler_fcu[1];
    uav_state.attitude[2] = euler_fcu[2];
    uav_state.attitude_rate[0] = msg->angular_velocity.x;
    uav_state.attitude_rate[1] = msg->angular_velocity.y;
    uav_state.attitude_rate[2] = msg->angular_velocity.z;
}
void fmt_test::gps_status_cb(const mavros_msgs::GPSRAW::ConstPtr &msg)
{
    uav_state.gps_status = msg->fix_type;
}


void fmt_test::fmt_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    uav_state.latitude = msg->latitude;
    uav_state.longitude = msg->longitude;
    uav_state.altitude = msg->altitude;
}


//主函数
fmt_test::fmt_test(ros::NodeHandle &nh)
{

    
    // 订阅FMT飞控相关信息
    fmt_state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, &fmt_test::fmt_state_cb,this);
    fmt_battery_sub = nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery", 1, &fmt_test::fmt_battery_cb,this);
    fmt_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &fmt_test::fmt_pos_cb,this);
    fmt_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 1, &fmt_test::fmt_vel_cb,this);
    fmt_attitude_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1, &fmt_test::fmt_att_cb,this);
    // 【订阅】GPS状态，来自飞控
    gps_status_sub = nh.subscribe<mavros_msgs::GPSRAW>("/mavros/gpsstatus/gps1/raw", 10, &fmt_test::gps_status_cb,this);
    // 【订阅】无人机当前经纬度，来自飞控
    fmt_global_position_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 1, &fmt_test::fmt_global_pos_cb,this);
    // 【订阅】无人机当前真实高度，来自飞控
    fmt_rel_alt_sub = nh.subscribe<std_msgs::Float64>("/mavros/global_position/rel_alt", 1, &fmt_test::fmt_global_rel_alt_cb,this);
   
   
   
    //system time
    //sim_time_pub = nh.advertise<rosgraph_msgs::Clock>("/mavros/clock", 10);

    //RC_CHANNELS
    //rc_in_pub = rc_nh.advertise<mavros_msgs::RCIn>("/mavros/rc/in", 10);
	//rc_out_pub = rc_nh.advertise<mavros_msgs::RCOut>("/mavros/rc/out", 10);
    //override_sub = rc_nh.subscribe("/mavros/rc/override", 10, &RCIOPlugin::override_cb, this);
    
    //HOME_POSITION

    //hp_sub = hp_nh.subscribe("/mavros/home_position/set", 10, &HomePositionPlugin::home_position_cb, this);
    //hp_pub = hp_nh.advertise<mavros_msgs::HomePosition>("home", 2, true);

    // gps global origin
	//gp_global_origin_pub = gp_nh.advertise<geographic_msgs::GeoPointStamped>("gp_origin", 10);
	//gp_set_global_origin_sub = gp_nh.subscribe("set_gp_origin", 10, &GlobalPositionPlugin::set_gp_origin_cb, this);
 

    //ALTITUDE 
    //altitude_pub = nh.advertise<mavros_msgs::Altitude>("altitude", 10);
 
    // MAVLINK_MSG_ID_HIGHRES_IMU是ID，100是频率
    set_stream_rate(nh, MAVLINK_MSG_ID_HIGHRES_IMU, 100);
    set_stream_rate(nh, MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50);
    set_stream_rate(nh, MAVLINK_MSG_ID_ATTITUDE, 100);
    set_stream_rate(nh, MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 10);


    //未验证：
    //set_stream_rate(nh, MAVLINK_MSG_ID_ALTITUDE, 100);
    //set_stream_rate(nh, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, 50);
    //set_stream_rate(nh, MAVLINK_MSG_ID_HOME_POSITION, 100);
    //set_stream_rate(nh, MAVLINK_MSG_ID_RC_CHANNELS, 10);
    //set_stream_rate(nh, MAVLINK_MSG_ID_SYSTEM_TIME, 10);

    // todo 继续添加其他mavlink消息

    cout << GREEN << "Waiting...." << TAIL << endl;
    sleep(10.0);

   // while(ros::ok())
   // {
        //调用一次回调函数
   //     ros::spinOnce();
        printf_uav_state();
       // rate.sleep();
   // }

}

void fmt_test::set_stream_rate(ros::NodeHandle& nh, int msg_id, int msg_rate)
{
    ros::ServiceClient stream_rate_client = nh.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");

    mavros_msgs::StreamRate srv;
    srv.request.stream_id = msg_id;
    srv.request.message_rate = msg_rate;
    srv.request.on_off = true;

    if (stream_rate_client.call(srv)) 
    {
        cout << GREEN << "Set MAVLINK ID [" << msg_id << "] rate to "<< msg_rate << "Hz successfully!" << TAIL << endl;
    } else 
    {
        cout << RED << "Set MAVLINK ID [" << msg_id << "] rate to "<< msg_rate << "Hz failed!" << TAIL << endl;
    }
}

void fmt_test::printf_uav_state()
{
    cout << GREEN << ">>>>>>>>>>>>>>>>>>>> FMT UAV State  <<<<<<<<<<<<<<<<<<<<" << TAIL << endl;
    //固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为n位
    cout << setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    // 打印 无人机状态
    if (uav_state.connected == true)
    {
        cout << GREEN << "FMT Status:  [ Connected ] ";
    }
    else
    {
        cout << RED << "FMT Status:[ Unconnected ] ";
    }
    //是否上锁
    if (uav_state.armed == true)
    {
        cout << GREEN << "[  Armed   ] ";
    }
    else
    {
        cout << RED << "[ DisArmed ] ";
    }

    cout << "[ " << uav_state.mode << " ] " << TAIL << endl;

    cout << GREEN << "Location: [ GPS ] " << TAIL;
    printf_gps_status();

    cout << GREEN << "UAV_pos [X Y Z] : " << uav_state.position[0] << " [ m ] " << uav_state.position[1] << " [ m ] " << uav_state.position[2] << " [ m ] " << TAIL << endl;
    cout << GREEN << "UAV_vel [X Y Z] : " << uav_state.velocity[0] << " [m/s] " << uav_state.velocity[1] << " [m/s] " << uav_state.velocity[2] << " [m/s] " << TAIL << endl;
    cout << GREEN << "UAV_att [R P Y] : " << uav_state.attitude[0] * 180 / M_PI << " [deg] " << uav_state.attitude[1] * 180 / M_PI << " [deg] " << uav_state.attitude[2] * 180 / M_PI << " [deg] " << TAIL << endl;
    cout << GREEN << "rel_alt         : " << uav_state.rel_alt<< " [ m ] "<< TAIL << endl;

    cout << GREEN << "Battery Voltage : " << uav_state.battery_state << " [V] "
         << "  Battery Percent : " << uav_state.battery_percetage << TAIL << endl;
}


void fmt_test::printf_gps_status()
{
    switch (uav_state.gps_status)
    {
    case prometheus_msgs::UAVState::GPS_FIX_TYPE_NO_GPS:
        cout << RED << " [GPS_FIX_TYPE_NO_GPS] " << TAIL << endl;
        break;
    case prometheus_msgs::UAVState::GPS_FIX_TYPE_NO_FIX:
        cout << RED << " [GPS_FIX_TYPE_NO_FIX] " << TAIL << endl;
        break;
    case prometheus_msgs::UAVState::GPS_FIX_TYPE_2D_FIX:
        cout << YELLOW << " [GPS_FIX_TYPE_2D_FIX] " << TAIL << endl;
        break;
    case prometheus_msgs::UAVState::GPS_FIX_TYPE_3D_FIX:
        cout << GREEN << " [GPS_FIX_TYPE_3D_FIX] " << TAIL << endl;
        break;
    }

    cout << GREEN << "GPS [lat lon alt] : " << uav_state.latitude << " [ deg ] " << uav_state.longitude << " [ deg ] " << uav_state.altitude << " [ m ] " << TAIL << endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fmt_node");
    	ros::NodeHandle nh("~");
	fmt_test fmt(nh);
	ros::spin();

    return 0;
}

