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


using namespace std;

class geometry_controller
{
     //public表明该数据成员、成员函数是对全部用户开放的。全部用户都能够直接进行调用，在程序的不论什么其他地方訪问。
    public:

        //构造函数
        geometry_controller(void):
            nh("~")
        {
            nh.param<float>("Quad/mass", Quad_MASS, 1.5);

            nh.param<string>("mavname", mav_name_, "iris");
            nh.param<int>("ctrl_mode", ctrl_mode_, ERROR_QUATERNION);
            nh.param<bool>("enable_sim", sim_enable_, true);
            nh.param<bool>("velocity_yaw", velocity_yaw_, false);
            nh.param<double>("max_acc", max_fb_acc_, 9.0);
            nh.param<double>("yaw_heading", mavYaw_, 0.0);
            nh.param<double>("drag_dx", dx_, 0.0);
            nh.param<double>("drag_dy", dy_, 0.0);
            nh.param<double>("drag_dz", dz_, 0.0);
            nh.param<double>("attctrl_constant", attctrl_tau_, 0.1);
            nh.param<double>("normalizedthrust_constant", norm_thrust_const_, 0.05);  // 1 / max acceleration
            nh.param<double>("normalizedthrust_offset", norm_thrust_offset_, 0.1);    // 1 / max acceleration
            nh.param<double>("Kp_x", Kpos_x_, 8.0);
            nh.param<double>("Kp_y", Kpos_y_, 8.0);
            nh.param<double>("Kp_z", Kpos_z_, 10.0);
            nh.param<double>("Kv_x", Kvel_x_, 1.5);
            nh.param<double>("Kv_y", Kvel_y_, 1.5);
            nh.param<double>("Kv_z", Kvel_z_, 3.3);
            nh.param<int>("posehistory_window", posehistory_window_, 200);
            nh.param<double>("init_pos_x", initTargetPos_x_, 0.0);
            nh.param<double>("init_pos_y", initTargetPos_y_, 0.0);
            nh.param<double>("init_pos_z", initTargetPos_z_, 2.0);

            targetPos_ << initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;  // Initial Position
            targetVel_ << 0.0, 0.0, 0.0;
            mavPos_ << 0.0, 0.0, 0.0;
            mavVel_ << 0.0, 0.0, 0.0;
            g_ << 0.0, 0.0, -9.8;
            Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
            Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;

            D_ << dx_, dy_, dz_;

            tau << tau_x, tau_y, tau_z;
            
            nh.param<float>("Limit/pxy_error_max", pos_error_max[0], 10.0);
            nh.param<float>("Limit/pxy_error_max", pos_error_max[1], 10.0);
            nh.param<float>("Limit/pz_error_max" , pos_error_max[2], 10.0);
            nh.param<float>("Limit/vxy_error_max", vel_error_max[0], 10.0);
            nh.param<float>("Limit/vxy_error_max", vel_error_max[1], 10.0);
            nh.param<float>("Limit/vz_error_max" , vel_error_max[2], 10.0);
            nh.param<float>("Limit/pxy_int_max"  , int_max[0], 10.0);
            nh.param<float>("Limit/pxy_int_max"  , int_max[1], 10.0);
            nh.param<float>("Limit/pz_int_max"   , int_max[2], 10.0);
            nh.param<float>("Limit/tilt_max", tilt_max, 20.0);
            nh.param<float>("Limit/int_start_error"  , int_start_error, 10.0);

            integral = Eigen::Vector3f(0.0,0.0,0.0);
        }

        //Quadrotor Parameter
        float Quad_MASS;

        //PID parameter for the control law
        Eigen::Vector3f Kp;
        Eigen::Vector3f Kd;
        Eigen::Vector3f Ki;

        //Limitation
        Eigen::Vector3f pos_error_max;
        Eigen::Vector3f vel_error_max;
        Eigen::Vector3f int_max;
        float tilt_max;
        float int_start_error;

        //积分项
        Eigen::Vector3d target_pos;
        Eigen::Vector3d target_vel;
        Eigen::Vector3d target_acc;

  std::vector<geometry_msgs::PoseStamped> posehistory_vector_;
  MAV_STATE companion_state_ = MAV_STATE::MAV_STATE_ACTIVE;

  double initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;
  Eigen::Vector3d targetPos_, targetVel_, targetAcc_, targetJerk_, targetSnap_, targetPos_prev_, targetVel_prev_;
  Eigen::Vector3d mavPos_, mavVel_, mavRate_;
  double mavYaw_;
  Eigen::Vector3d g_;
  Eigen::Vector4d mavAtt_, q_des;
  Eigen::Vector4d cmdBodyRate_;  //{wx, wy, wz, Thrust}
  Eigen::Vector3d Kpos_, Kvel_, D_;
  Eigen::Vector3d a0, a1, tau;
  double tau_x, tau_y, tau_z;
  double Kpos_x_, Kpos_y_, Kpos_z_, Kvel_x_, Kvel_y_, Kvel_z_;
  int posehistory_window_;

        //输出
        prometheus_msgs::ControlOutput _ControlOutput;


        //Printf the PID parameter
        void printf_param();

        void printf_result();

        // Position control main function 
        // [Input: Current state, Reference state, sub_mode, dt; Output: AttitudeReference;]
        prometheus_msgs::ControlOutput pos_controller(const prometheus_msgs::DroneState& _DroneState, const prometheus_msgs::PositionReference& _Reference_State, float dt);

    private:
        ros::NodeHandle nh;

        static double getVelocityYaw(const Eigen::Vector3d velocity) { return atan2(velocity(1), velocity(0)); };

};

prometheus_msgs::ControlOutput geometry_controller::pos_controller(
    const prometheus_msgs::DroneState& _DroneState, 
    const prometheus_msgs::PositionReference& _Reference_State, float dt)
{
    // 变量转换
    for (int i=0; i<3; i++)
    {
        // 此处可能会存在更新频率较低的问题（因为不是直接订阅的mavros消息）
        mavPos_[i] = _DroneState.position[i];
        mavVel_[i] = _DroneState.velocity[i];
        mavRate_[i] = _DroneState.attitude_rate[i];

        target_pos[i] = _Reference_State.position_ref[i];
        target_vel[i] = _Reference_State.velocity_ref[i];
        target_acc[i] = _Reference_State.acceleration_ref[i];
    }

    mavAtt_(0) = _DroneState.attitude_q.w;
    mavAtt_(1) = _DroneState.attitude_q.x;
    mavAtt_(2) = _DroneState.attitude_q.y;
    mavAtt_(3) = _DroneState.attitude_q.z;
}

// 计算角速率指令
void geometry_controller::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &target_pos,
                                       const Eigen::Vector3d &target_vel, const Eigen::Vector3d &target_acc) 
{
  /// Compute BodyRate commands using differential flatness
  /// 该控制器基于Faessler 2017论文，论文名称：
  
  const Eigen::Vector3d a_ref = target_acc;

  //暂时不清楚这一步是为什么，锁死航向？
  bool velocity_yaw_ = false;
  if (velocity_yaw_) 
  {
    mavYaw_ = getVelocityYaw(mavVel_);
  }

  const Eigen::Vector4d q_ref = acc2quaternion(a_ref - g_, mavYaw_);
  const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);

  const Eigen::Vector3d pos_error = mavPos_ - target_pos;
  const Eigen::Vector3d vel_error = mavVel_ - target_vel;

  // 加速度反馈量
  Eigen::Vector3d a_fb =
      Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error;  // feedforward term for trajectory error
  if (a_fb.norm() > max_fb_acc_)
    a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;  // Clip acceleration if reference is too large

  const Eigen::Vector3d a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * target_vel;  // Rotor drag
  const Eigen::Vector3d a_des = a_fb + a_ref - a_rd - g_;

  q_des = acc2quaternion(a_des, mavYaw_);

  if (ctrl_mode_ == ERROR_GEOMETRIC) {
    bodyrate_cmd = geometric_attcontroller(q_des, a_des, mavAtt_);  // Calculate BodyRate

  } else {
    bodyrate_cmd = attcontroller(q_des, a_des, mavAtt_);  // Calculate BodyRate
  }
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


prometheus_msgs::ControlOutput geometry_controller::pos_controller(
    const prometheus_msgs::DroneState& _DroneState, 
    const prometheus_msgs::PositionReference& _Reference_State, float dt)
{
    Eigen::Vector3d accel_sp;
    
    // 计算误差项
    Eigen::Vector3f pos_error;
    Eigen::Vector3f vel_error;
    
    pos_error = prometheus_control_utils::cal_pos_error(_DroneState, _Reference_State);
    vel_error = prometheus_control_utils::cal_vel_error(_DroneState, _Reference_State);

    // 误差项限幅
    for (int i=0; i<3; i++)
    {
        pos_error[i] = constrain_function(pos_error[i], pos_error_max[i]);
        vel_error[i] = constrain_function(vel_error[i], vel_error_max[i]);
    }

    // 期望加速度 = 加速度前馈 + PID
    for (int i=0; i<3; i++)
    {
        accel_sp[i] = _Reference_State.acceleration_ref[i] + Kp[i] * pos_error[i] + Kd[i] * vel_error[i] + Ki[i] * integral[i];
    }
    
    accel_sp[2] = accel_sp[2] + 9.8;

    // 更新积分项
    for (int i=0; i<3; i++)
    {
        if(abs(pos_error[i]) < int_start_error)
        {
            integral[i] += pos_error[i] * dt;

            if(abs(integral[i]) > int_max[i])
            {
                cout << "Integral saturation! " << " [0-1-2] "<< i <<endl;
                cout << "[integral]: "<< integral[i]<<" [int_max]: "<<int_max[i]<<" [m/s] "<<endl;
            }

            integral[i] = constrain_function(integral[i], int_max[i]);
        }else
        {
            integral[i] = 0;
        }

        // If not in OFFBOARD mode, set all intergral to zero.
        if(_DroneState.mode != "OFFBOARD")
        {
            integral[i] = 0;
        }
    }

    // 期望推力 = 期望加速度 × 质量
    // 归一化推力 ： 根据电机模型，反解出归一化推力
    Eigen::Vector3d thrust_sp;
    Eigen::Vector3d throttle_sp;
    thrust_sp =  prometheus_control_utils::accelToThrust(accel_sp, Quad_MASS, tilt_max);
    throttle_sp = prometheus_control_utils::thrustToThrottle(thrust_sp);

    for (int i=0; i<3; i++)
    {
        _ControlOutput.Thrust[i] = thrust_sp[i];
        _ControlOutput.Throttle[i] = throttle_sp[i];
    }

    return _ControlOutput;

}

void geometry_controller::printf_result()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>  PID Position Controller  <<<<<<<<<<<<<<<<<<<<<<" <<endl;

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
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PID Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout <<"Quad_MASS : "<< Quad_MASS << endl;

    cout <<"Kp_x : "<< Kp[0] << endl;
    cout <<"Kp_y : "<< Kp[1] << endl;
    cout <<"Kp_z : "<< Kp[2] << endl;

    cout <<"Kd_x : "<< Kd[0] << endl;
    cout <<"Kd_y : "<< Kd[1] << endl;
    cout <<"Kd_z : "<< Kd[2] << endl;

    cout <<"Ki_x : "<< Ki[0] << endl;
    cout <<"Ki_y : "<< Ki[1] << endl;
    cout <<"Ki_z : "<< Ki[2] << endl;

    cout <<"Limit:  " <<endl;
    cout <<"pxy_error_max : "<< pos_error_max[0] << endl;
    cout <<"pz_error_max :  "<< pos_error_max[2] << endl;
    cout <<"vxy_error_max : "<< vel_error_max[0] << endl;
    cout <<"vz_error_max :  "<< vel_error_max[2] << endl;
    cout <<"pxy_int_max : "<< int_max[0] << endl;
    cout <<"pz_int_max : "<< int_max[2] << endl;
    cout <<"tilt_max : "<< tilt_max << endl;
    cout <<"int_start_error : "<< int_start_error << endl;

}

#endif
