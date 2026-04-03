#ifndef UAV_ESTIMATOR_H
#define UAV_ESTIMATOR_H

// 头文件
#include <ros/ros.h>
#include <iostream>
#include <bitset>
#include <Eigen/Eigen>
#include <mavros/frame_tf.h>
#include <GeographicLib/Geocentric.hpp>

#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/TextInfo.h>
#include <prometheus_msgs/OffsetPose.h>
#include <prometheus_msgs/GPSData.h>
#include <prometheus_msgs/LinktrackNodeframe2.h>
#include <prometheus_msgs/LinktrackNode2.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/GPSRAW.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/StreamRate.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>

#include "math_utils.h"
#include "printf_utils.h"
#include "fmt_test.h"


#include <mavros/mavros_plugin.h>
#include <sensor_msgs/TimeReference.h>
#include <std_msgs/Duration.h>
#include <mavros_msgs/TimesyncStatus.h>
#include <rosgraph_msgs/Clock.h>


#include <mavros_msgs/EstimatorStatus.h>
#include <mavros_msgs/ExtendedState.h>




#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/OverrideRCIn.h>


#include <mavros_msgs/HomePosition.h>

#include <array>
#include <unordered_map>
#include <mavros/utils.h>
#include <ros/console.h>


#include <angles/angles.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/UInt32.h>
#include <sensor_msgs/NavSatStatus.h>
#include <geographic_msgs/GeoPointStamped.h>



using namespace std;





// 宏定义
#define MAVLINK_MSG_ID_HIGHRES_IMU 105
#define MAVLINK_MSG_ID_LOCAL_POSITION_NED 32
#define MAVLINK_MSG_ID_ATTITUDE 30
#define MAVLINK_MSG_ID_ALTITUDE 000
#define MAVLINK_MSG_ID_EXTENDED_SYS_STATE 245
#define MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN 49
#define MAVLINK_MSG_ID_HOME_POSITION 242
#define MAVLINK_MSG_ID_RC_CHANNELS 65
#define MAVLINK_MSG_ID_SYSTEM_TIME 2




class fmt_test
{
    public:
        fmt_test(ros::NodeHandle& nh);

        // 订阅话题
        ros::Subscriber fmt_state_sub;
		ros::Subscriber fmt_battery_sub;
        ros::Subscriber fmt_position_sub;
        ros::Subscriber fmt_velocity_sub;
        ros::Subscriber fmt_attitude_sub;
        ros::Subscriber gps_status_sub;
        ros::Subscriber fmt_global_position_sub;
        ros::Subscriber fmt_rel_alt_sub;
		//ros::Publisher sim_time_pub;
	
		//ros::Publisher rc_in_pub;
		//ros::Publisher rc_out_pub;
		//ros::Subscriber override_sub;

		//ros::Publisher hp_pub;
		//ros::Subscriber hp_sub;

		//ros::Publisher gp_global_origin_pub;
		//ros::Publisher gp_global_offset_pub;

		//ros::Publisher altitude_pub;

		// 基本变量
		prometheus_msgs::UAVState uav_state;
		bool uav_state_update{false};
		void set_stream_rate(ros::NodeHandle& nh, int msg_id, int msg_rate);
		void printf_uav_state();
		void printf_gps_status();

    private:
    	
    	void fmt_state_cb(const mavros_msgs::State::ConstPtr &msg);
    	void fmt_battery_cb(const sensor_msgs::BatteryState::ConstPtr &msg);
    	void fmt_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    	void fmt_global_rel_alt_cb(const std_msgs::Float64::ConstPtr &msg);
    	void fmt_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
    	void fmt_att_cb(const sensor_msgs::Imu::ConstPtr &msg);
    	void gps_status_cb(const mavros_msgs::GPSRAW::ConstPtr &msg);
    	void fmt_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr &msg);
		
		
		
		//以下是新加，TODO，待修改
	

		//system time
		void handle_system_time(const mavlink::mavlink_message_t *msg, mavlink::common::msg::SYSTEM_TIME &mtime)
			{
	
			}

		//RC_CHANNELS
		void handle_rc_channels(const mavlink::mavlink_message_t *msg, mavlink::common::msg::RC_CHANNELS &channels)
		{
		

		}

		//HOME_POSITION
		void handle_home_position(const mavlink::mavlink_message_t *msg, mavlink::common::msg::HOME_POSITION &home_position)
		{
		
		}

		//ALTITUDE
		void handle_altitude(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ALTITUDE &altitude)
		{

		}



		//GPS_GLOBAL_ORIGIN
		void handle_gps_global_origin(const mavlink::mavlink_message_t *msg, mavlink::common::msg::GPS_GLOBAL_ORIGIN &glob_orig)
	{
		
	}
};


#endif
