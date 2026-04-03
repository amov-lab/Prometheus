#include <ros/ros.h>
#include "quadrotor_msgs/PositionCommand.h"
#include "quadrotor_msgs/PolynomialTrajectory.h"
#include "mini_snap_traj_utils/polynomial_traj.h"
#include <geometry_msgs/PoseArray.h>
#include "geometry_utils.h"
#include <prometheus_msgs/UAVCommand.h>

ros::Publisher pose_cmd_pub;
ros::Publisher traj_pts_pub;
ros::Subscriber poly_traj_sub;
ros::Publisher uav_cmd_pub;

PolynomialTraj Poly_traj;
quadrotor_msgs::PositionCommand cmd;
ros::Time start_time;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

double last_yaw_, last_yaw_dot_;
bool receive_traj_ = false;
int traj_id_;
double traj_duration_, time_forward_;

double time_traj;

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
    constexpr double PI = 3.1415926;
    constexpr double YAW_DOT_MAX_PER_SEC = PI;
    // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
    std::pair<double, double> yaw_yawdot(0, 0);
    double yaw = 0;
    double yawdot = 0;

    //   Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
    Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? Poly_traj.evaluate(t_cur + time_forward_) - pos : Poly_traj.evaluate(traj_duration_) - pos;
    double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
    double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
    if (yaw_temp - last_yaw_ > PI)
    {
        if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
        {
            yaw = last_yaw_ - max_yaw_change;
            if (yaw < -PI)
                yaw += 2 * PI;

            yawdot = -YAW_DOT_MAX_PER_SEC;
        }
        else
        {
            yaw = yaw_temp;
            if (yaw - last_yaw_ > PI)
                yawdot = -YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }
    else if (yaw_temp - last_yaw_ < -PI)
    {
        if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
        {
            yaw = last_yaw_ + max_yaw_change;
            if (yaw > PI)
                yaw -= 2 * PI;

            yawdot = YAW_DOT_MAX_PER_SEC;
        }
        else
        {
            yaw = yaw_temp;
            if (yaw - last_yaw_ < -PI)
                yawdot = YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }
    else
    {
        if (yaw_temp - last_yaw_ < -max_yaw_change)
        {
            yaw = last_yaw_ - max_yaw_change;
            if (yaw < -PI)
                yaw += 2 * PI;

            yawdot = -YAW_DOT_MAX_PER_SEC;
        }
        else if (yaw_temp - last_yaw_ > max_yaw_change)
        {
            yaw = last_yaw_ + max_yaw_change;
            if (yaw > PI)
                yaw -= 2 * PI;

            yawdot = YAW_DOT_MAX_PER_SEC;
        }
        else
        {
            yaw = yaw_temp;
            if (yaw - last_yaw_ > PI)
                yawdot = -YAW_DOT_MAX_PER_SEC;
            else if (yaw - last_yaw_ < -PI)
                yawdot = YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }

    if (fabs(yaw - last_yaw_) <= max_yaw_change)
        yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
    yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
    last_yaw_ = yaw;
    last_yaw_dot_ = yawdot;

    yaw_yawdot.first = yaw;
    yaw_yawdot.second = yawdot;

    return yaw_yawdot;
}

void polyTrajCallback(quadrotor_msgs::PolynomialTrajectory::ConstPtr msg)
{
    ROS_INFO("[my Traj server]:receive poly_traj");

    Poly_traj.reset();
    int idx;
    for (int i = 0; i < msg->num_segment; i++)
    {
        vector<double> cx, cy, cz;
        for (int j = 0; j < msg->num_order + 1; j++)
        {
            idx = i * (msg->num_order + 1) + j;
            cx.push_back(msg->coef_x[idx]);
            cy.push_back(msg->coef_y[idx]);
            cz.push_back(msg->coef_z[idx]);
        }
        // 前面三个轨迹参数，后面是这段轨迹的时间
        Poly_traj.addSegment(cx, cy, cz, msg->time[i]);
    }
    Poly_traj.init();

    

    start_time = ros::Time::now();
    traj_id_ = msg->trajectory_id;
    traj_duration_ = Poly_traj.getTimeSum();
    receive_traj_ = true;

    // 打印第一段的时间
    ROS_INFO("[my traj server]:time=%f", Poly_traj.getTimes().front());
    // 打印总时间
    ROS_INFO("[my traj server]:traj_duration_=%f", traj_duration_);

}

void pub_traj(double t_cur)
{
    static int old_cnt = 0;
    int cnt = (int)((traj_duration_ - t_cur) * 2) + 1;

    if (cnt != old_cnt)
    {
        geometry_msgs::PoseArray traj_pts;
        geometry_msgs::Pose traj_pt;
        Eigen::Vector3d opt_pt(Eigen::Vector3d::Zero());

        traj_pts.header.stamp = ros::Time::now();
        for (int i = 0; i < cnt + 1; i++)
        {
            opt_pt = Poly_traj.evaluate(std::min(t_cur + i * 0.5, traj_duration_));
            traj_pt.orientation.w = 1.0;
            traj_pt.position.x = opt_pt(0);
            traj_pt.position.y = opt_pt(1);
            traj_pt.position.z = opt_pt(2);
            traj_pts.poses.push_back(traj_pt);
        }
        traj_pts_pub.publish(traj_pts);
    }
    old_cnt = cnt;
}


void pub_prometheus_command(quadrotor_msgs::PositionCommand ego_traj_cmd)
{
  prometheus_msgs::UAVCommand uav_command;
  uav_command.header.stamp = ros::Time::now();
  uav_command.source = prometheus_msgs::UAVCommand::EGO;
  uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
  if(ego_traj_cmd.velocity.x == 0.0 && ego_traj_cmd.velocity.y == 0.0)
  {
    uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
  }
  else{
    uav_command.Move_mode = prometheus_msgs::UAVCommand::TRAJECTORY;
  }
  uav_command.position_ref[0] = ego_traj_cmd.position.x;
  uav_command.position_ref[1] = ego_traj_cmd.position.y;
  uav_command.position_ref[2] = ego_traj_cmd.position.z;
  uav_command.velocity_ref[0] = ego_traj_cmd.velocity.x;
  uav_command.velocity_ref[1] = ego_traj_cmd.velocity.y;
  uav_command.velocity_ref[2] = ego_traj_cmd.velocity.z;
  uav_command.acceleration_ref[0] = ego_traj_cmd.acceleration.x;
  uav_command.acceleration_ref[1] = ego_traj_cmd.acceleration.y;
  uav_command.acceleration_ref[2] = ego_traj_cmd.acceleration.z;
  uav_command.yaw_ref = geometry_utils::normalize_angle(ego_traj_cmd.yaw);
  // uav_command.yaw_rate_ref         = ego_traj_cmd.yaw_dot;
  uav_cmd_pub.publish(uav_command);
}
void debug_cb(const ros::TimerEvent &e)
{
    if (!receive_traj_)
        return;

        
    ROS_INFO("time = %f", time_traj);
}

void cmdCallback(const ros::TimerEvent &e)
{
    if (!receive_traj_)
        return;

    Eigen::Vector3d pos(Eigen::Vector3d::Zero());
    Eigen::Vector3d vel(Eigen::Vector3d::Zero());
    Eigen::Vector3d acc(Eigen::Vector3d::Zero());
    Eigen::Vector3d jerk(Eigen::Vector3d::Zero());
    std::pair<double, double> yaw_yawdot(0, 0);

    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - start_time).toSec();
    time_traj = t_cur;
    // ROS_INFO("time = %f", t_cur);
    static ros::Time time_last = ros::Time::now();

    if (t_cur < traj_duration_ && t_cur >= 0.0)
    {
        pos = Poly_traj.evaluate(t_cur);
        vel = Poly_traj.evaluateVel(t_cur);
        acc = Poly_traj.evaluateAcc(t_cur);
        jerk = Poly_traj.evaluateJerk(t_cur);
        yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
        pub_traj(t_cur + 0.5);
    }
    else if (t_cur >= traj_duration_)
    {
        pos = Poly_traj.evaluate(traj_duration_);
        vel.setZero();
        acc.setZero();
        yaw_yawdot.first = last_yaw_;
        yaw_yawdot.second = 0;
        receive_traj_ = false;
        ROS_WARN("[my Traj server]: traj over.");
    }
    else
    {
        ROS_WARN("[my Traj server]: invalid time.");
    }
    time_last = time_now;

    cmd.header.stamp = time_now;
    cmd.header.frame_id = "world";
    cmd.trajectory_id = traj_id_;

    cmd.position.x = pos(0);
    cmd.position.y = pos(1);
    cmd.position.z = pos(2);

    cmd.velocity.x = vel(0);
    cmd.velocity.y = vel(1);
    cmd.velocity.z = vel(2);

    cmd.acceleration.x = acc(0);
    cmd.acceleration.y = acc(1);
    cmd.acceleration.z = acc(2);

    cmd.jerk.x = jerk(0);
    cmd.jerk.y = jerk(1);
    cmd.jerk.z = jerk(2);

    cmd.yaw = yaw_yawdot.first;
    cmd.yaw_dot = yaw_yawdot.second;

    last_yaw_ = cmd.yaw;

    pose_cmd_pub.publish(cmd);

    pub_prometheus_command(cmd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_server_pengyu");
    ros::NodeHandle nh("~");

    pose_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
    traj_pts_pub = nh.advertise<geometry_msgs::PoseArray>("/traj_pts", 50);
    poly_traj_sub = nh.subscribe("/poly_coefs", 10, polyTrajCallback);

    // [订阅] EGO规划结果 - to Prometheus uav_control
    uav_cmd_pub = nh.advertise<prometheus_msgs::UAVCommand>("/prometheus/command", 50);

    ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

    ros::Timer debug_timer = nh.createTimer(ros::Duration(1.0), debug_cb);

    /* control parameter */
    cmd.kx[0] = pos_gain[0];
    cmd.kx[1] = pos_gain[1];
    cmd.kx[2] = pos_gain[2];

    cmd.kv[0] = vel_gain[0];
    cmd.kv[1] = vel_gain[1];
    cmd.kv[2] = vel_gain[2];
    last_yaw_ = 0.0;
    last_yaw_dot_ = 0.0;
    time_forward_ = 1.0;

    ros::Duration(1.0).sleep();

    ROS_INFO("[my Traj server]: ready.");

    ros::spin();

    return 0;
}