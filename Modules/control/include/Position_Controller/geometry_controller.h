/***************************************************************************************************************************
* geometry_controller.h
*
* Author: Qyp
*
* Update Time: 2021.3.0
*
* Introduction:  Geometry Controller  
***************************************************************************************************************************/
#ifndef geometry_controller_H
#define geometry_controller_H

#include <math.h>
#include <command_to_mavros.h>
#include <prometheus_control_utils.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/PositionReference.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/ControlOutput.h>


#define ERROR_QUATERNION 1
#define ERROR_GEOMETRIC 2

using namespace std;

class geometry_controller
{
     //public表明该数据成员、成员函数是对全部用户开放的。全部用户都能够直接进行调用，在程序的不论什么其他地方訪问。
    public:

        //构造函数
        geometry_controller(void):
            nh("~")
        {
            /// 读取参数
            // 控制模式
            nh.param<int>("geometry_controller/ctrl_mode", ctrl_mode_, ERROR_GEOMETRIC);
            // 是否使用速度来控制偏航角
            // 是：使用速度期望值计算当前期望偏航角，否：使用系统预设的偏航角
            nh.param<bool>("geometry_controller/velocity_yaw", velocity_yaw_, false);
            // 最大加速度反馈部分
            nh.param<double>("geometry_controller/max_acc", max_fb_acc_, 9.0);
            // 预设的期望偏航角，如果velocity_yaw_设置为true，则此设置无效
            nh.param<double>("geometry_controller/yaw_heading", mavYaw_, 0.0);
            // rotor drag
            nh.param<double>("geometry_controller/drag_dx", dx_, 0.0);
            nh.param<double>("geometry_controller/drag_dy", dy_, 0.0);
            nh.param<double>("geometry_controller/drag_dz", dz_, 0.0);
            // 常数，不清楚作用
            nh.param<double>("geometry_controller/attctrl_constant", attctrl_tau_, 0.1);
            // 推力相关参数，不清楚作用
            nh.param<double>("geometry_controller/normalizedthrust_constant", norm_thrust_const_, 0.05);  // 1 / max acceleration
            nh.param<double>("geometry_controller/normalizedthrust_offset", norm_thrust_offset_, 0.1);    // 1 / max acceleration
            // 控制参数
            nh.param<double>("geometry_controller/Kp_x", Kpos_x_, 8.0);
            nh.param<double>("geometry_controller/Kp_y", Kpos_y_, 8.0);
            nh.param<double>("geometry_controller/Kp_z", Kpos_z_, 10.0);
            nh.param<double>("geometry_controller/Kv_x", Kvel_x_, 1.5);
            nh.param<double>("geometry_controller/Kv_y", Kvel_y_, 1.5);
            nh.param<double>("geometry_controller/Kv_z", Kvel_z_, 3.3);
            // 初始目标值
            nh.param<double>("geometry_controller/init_pos_x", initTargetPos_x_, 0.0);
            nh.param<double>("geometry_controller/init_pos_y", initTargetPos_y_, 0.0);
            nh.param<double>("geometry_controller/init_pos_z", initTargetPos_z_, 2.0);

            targetPos_ << initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;  // Initial Position
            targetVel_ << 0.0, 0.0, 0.0;
            mavPos_ << 0.0, 0.0, 0.0;
            mavVel_ << 0.0, 0.0, 0.0;
            g_ << 0.0, 0.0, -9.8;
            Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
            Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
            D_ << dx_, dy_, dz_;
        }

        // 控制模式
        int ctrl_mode_;
        bool velocity_yaw_;
        // 初始目标值
        double initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;
        // 目标位置、速度、加速度
        Eigen::Vector3d targetPos_, targetVel_, targetAcc_, targetJerk_, targetSnap_;
        Eigen::Vector3d targetPos_prev_, targetVel_prev_;
        // 预设的期望偏航角，如果velocity_yaw_设置为true，则此设置无效
        double mavYaw_;
        // 无人机状态信息 - 位置、速度、角速度
        Eigen::Vector3d mavPos_, mavVel_, mavRate_;
        // 重力加速度
        Eigen::Vector3d g_;
        // 无人机姿态，期望姿态
        Eigen::Vector4d mavAtt_, q_des;
        // 期望角速度及推力
        Eigen::Vector4d cmdBodyRate_;  //{wx, wy, wz, Thrust}
        // rotor drag
        double dx_, dy_, dz_;
        Eigen::Vector3d D_;
        // 控制参数
        double Kpos_x_, Kpos_y_, Kpos_z_, Kvel_x_, Kvel_y_, Kvel_z_;
        Eigen::Vector3d Kpos_, Kvel_;
        // 最大 反馈加速度
        double max_fb_acc_;

        // 不清楚作用的变量
        double attctrl_tau_;
        double norm_thrust_const_, norm_thrust_offset_;

        //Printf the PID parameter
        void printf_param();

        void printf_result();

        // Position control main function 
        Eigen::Vector4d pos_controller(const prometheus_msgs::DroneState& _DroneState, const prometheus_msgs::PositionReference& _Reference_State);

    private:
        ros::NodeHandle nh;

        static double getVelocityYaw(const Eigen::Vector3d velocity) { return atan2(velocity(1), velocity(0)); };

        void computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &target_pos,
                                       const Eigen::Vector3d &target_vel, const Eigen::Vector3d &target_acc);
        Eigen::Vector4d geometric_attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc,
                                                       Eigen::Vector4d &curr_att);
        Eigen::Vector4d attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc,
                                             Eigen::Vector4d &curr_att);
        Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw);

};

Eigen::Vector4d geometry_controller::pos_controller(
    const prometheus_msgs::DroneState& _DroneState, 
    const prometheus_msgs::PositionReference& _Reference_State)
{
    // 变量转换
    for (int i=0; i<3; i++)
    {
        // 此处可能会存在更新频率较低的问题（因为不是直接订阅的mavros消息）
        mavPos_[i] = _DroneState.position[i];
        mavVel_[i] = _DroneState.velocity[i];
        mavRate_[i] = _DroneState.attitude_rate[i];

        targetPos_[i] = _Reference_State.position_ref[i];
        targetVel_[i] = _Reference_State.velocity_ref[i];
        targetAcc_[i] = _Reference_State.acceleration_ref[i];
    }

    mavAtt_(0) = _DroneState.attitude_q.w;
    mavAtt_(1) = _DroneState.attitude_q.x;
    mavAtt_(2) = _DroneState.attitude_q.y;
    mavAtt_(3) = _DroneState.attitude_q.z;

    // 计算
    computeBodyRateCmd(cmdBodyRate_, targetPos_, targetVel_, targetAcc_);

    return cmdBodyRate_;
}

// 计算角速率指令
void geometry_controller::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &target_pos,
                                       const Eigen::Vector3d &target_vel, const Eigen::Vector3d &target_acc) 
{
    /// Compute BodyRate commands using differential flatness
    /// 该控制器基于Faessler 2017论文，论文名称：
    
    const Eigen::Vector3d a_ref = target_acc;

    if (velocity_yaw_) 
    {
        // 根据速度计算偏航角
        mavYaw_ = getVelocityYaw(mavVel_);
    }

    const Eigen::Vector4d q_ref = acc2quaternion(a_ref - g_, mavYaw_);
    const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);

    const Eigen::Vector3d pos_error = mavPos_ - target_pos;
    const Eigen::Vector3d vel_error = mavVel_ - target_vel;

    // 加速度 - 反馈部分
    // 根据位置、速度误差计算，同时设置限幅，即max_fb_acc_
    // Kpos_ 是三维向量，Kpos_.asDiagonal()变成对角矩阵
    Eigen::Vector3d a_fb =
        Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error;  // feedforward term for trajectory error
    if (a_fb.norm() > max_fb_acc_)
        a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;  // Clip acceleration if reference is too large

    /// Rotor drag
    // 默认情况下，D_为0
    const Eigen::Vector3d a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * target_vel;  

    // 期望加速度 = 加速度反馈部分 + 加速度参考值 - rotor drag - 重力加速度
    const Eigen::Vector3d a_des = a_fb + a_ref - a_rd - g_;

    // 计算期望四元数
    q_des = acc2quaternion(a_des, mavYaw_);

    /// 计算 期望姿态角速度（默认使用geometric_attcontroller）    
    if (ctrl_mode_ == ERROR_GEOMETRIC) 
    {
        bodyrate_cmd = geometric_attcontroller(q_des, a_des, mavAtt_);  // Calculate BodyRate
    } else {
        bodyrate_cmd = attcontroller(q_des, a_des, mavAtt_);  // Calculate BodyRate
    }
}



Eigen::Vector4d geometry_controller::geometric_attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc,
                                                       Eigen::Vector4d &curr_att) 
{
  // Geometric attitude controller
  // Attitude error is defined as in Lee, Taeyoung, Melvin Leok, and N. Harris McClamroch. "Geometric tracking control
  // of a quadrotor UAV on SE (3)." 49th IEEE conference on decision and control (CDC). IEEE, 2010.
  // The original paper inputs moment commands, but for offboard control angular rate commands are sent

  Eigen::Vector4d ratecmd;
  Eigen::Matrix3d rotmat;    // Rotation matrix of current atttitude
  Eigen::Matrix3d rotmat_d;  // Rotation matrix of desired attitude
  Eigen::Vector3d zb;
  Eigen::Vector3d error_att;

  rotmat = quat2RotMatrix(curr_att);
  rotmat_d = quat2RotMatrix(ref_att);

  error_att = 0.5 * matrix_hat_inv(rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat);
  ratecmd.head(3) = (2.0 / attctrl_tau_) * error_att;
  rotmat = quat2RotMatrix(mavAtt_);
  zb = rotmat.col(2);
  ratecmd(3) =
      std::max(0.0, std::min(1.0, norm_thrust_const_ * ref_acc.dot(zb) + norm_thrust_offset_));  // Calculate thrust

  return ratecmd;
}

Eigen::Vector4d geometry_controller::attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc,
                                             Eigen::Vector4d &curr_att) {
  // Geometric attitude controller
  // Attitude error is defined as in Brescianini, Dario, Markus Hehn, and Raffaello D'Andrea. Nonlinear quadrocopter
  // attitude control: Technical report. ETH Zurich, 2013.

  Eigen::Vector4d ratecmd;
  Eigen::Vector4d qe, q_inv, inverse;
  Eigen::Matrix3d rotmat;
  Eigen::Vector3d zb;

  inverse << 1.0, -1.0, -1.0, -1.0;
  q_inv = inverse.asDiagonal() * curr_att;
  qe = quatMultiplication(q_inv, ref_att);
  ratecmd(0) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);
  ratecmd(1) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);
  ratecmd(2) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3);
  rotmat = quat2RotMatrix(mavAtt_);
  zb = rotmat.col(2);
  ratecmd(3) =
      std::max(0.0, std::min(1.0, norm_thrust_const_ * ref_acc.dot(zb) + norm_thrust_offset_));  // Calculate thrust

  return ratecmd;
}

Eigen::Vector4d geometry_controller::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw) 
{
  Eigen::Vector4d quat;
  Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
  Eigen::Matrix3d rotmat;

  proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

  zb_des = vector_acc / vector_acc.norm();
  yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
  xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

  rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
  quat = rot2Quaternion(rotmat);
  return quat;
}

void geometry_controller::printf_result()
{
    //固定的浮点显示
    cout.setf(ios::fixed);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout<<setprecision(2);
}

// 【打印参数函数】
void geometry_controller::printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Geometry Control Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout <<"norm_thrust_const_ : "<< norm_thrust_const_ << endl;
    cout <<"norm_thrust_offset_ : "<< norm_thrust_offset_ << endl;
}

#endif
