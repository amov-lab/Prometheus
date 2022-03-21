#include "Quadrotor_dynamics.h"
#include "ode/boost/numeric/odeint.hpp"
#include <Eigen/Geometry>
#include <boost/bind.hpp>
#include <iostream>

#include <ros/ros.h>
namespace odeint = boost::numeric::odeint;

namespace QuadrotorSimulator
{

Quadrotor::Quadrotor(void)
{
  // 无人机参数
  alpha0     = 48; // degree
  // 重力常数
  gravity         = 9.81;
  // 质量
  quad_mass      = 0.98;
  // 螺旋桨半径
  prop_radius = 0.062;
  // 转动惯量
  double Ixx = 2.64e-3, Iyy = 2.64e-3, Izz = 4.96e-3;
  quad_J           = Eigen::Vector3d(Ixx, Iyy, Izz).asDiagonal();

  // 推力系数
  motor_kf = 8.98132e-9;
  // motor_km = 2.5e-9; // from Nate
  // km = (Cq/Ct)*Dia*kf
  // Cq/Ct for 8 inch props from UIUC prop db ~ 0.07
  motor_km = 0.07 * (3 * prop_radius) * motor_kf;

  // 机臂长度
  arm_length          = 0.26;
  // 电机时间常数
  motor_time_constant = 1.0 / 30;
  // 最小转速
  motor_min_rpm             = 1200;
  // 最大转速
  motor_max_rpm             = 35000;

  // 无人机位置
  quad_state.pos = Eigen::Vector3d::Zero();
  // 无人机速度
  quad_state.vel         = Eigen::Vector3d::Zero();
  // 无人机旋转矩阵
  quad_state.R         = Eigen::Matrix3d::Identity();
  // 无人机角速度
  quad_state.omega     = Eigen::Vector3d::Zero();
  // 无人机电机转速
  quad_state.motor_rpm = Eigen::Array4d::Zero();
  // 外部干扰力
  disturbance_force.setZero();
  // 将位置、速度、旋转矩阵、角速度、电机转速放入同一个向量中
  updateInternalState();
  // 控制输入：4个电机的控制量
  motor_rpm_cmd = Eigen::Array4d::Zero();
}

// 将位置、速度、旋转矩阵、角速度、电机转速放入同一个向量中
void Quadrotor::updateInternalState(void)
{
  for (int i = 0; i < 3; i++)
  {
    internal_state[0 + i]  = quad_state.pos(i);
    internal_state[3 + i]  = quad_state.vel(i);
    internal_state[6 + i]  = quad_state.R(i, 0);
    internal_state[9 + i]  = quad_state.R(i, 1);
    internal_state[12 + i] = quad_state.R(i, 2);
    internal_state[15 + i] = quad_state.omega(i);
  }
  internal_state[18] = quad_state.motor_rpm(0);
  internal_state[19] = quad_state.motor_rpm(1);
  internal_state[20] = quad_state.motor_rpm(2);
  internal_state[21] = quad_state.motor_rpm(3);
}

// 根据控制输入（计算得到无人机自身产生的力和力矩）、外部干扰力、外部干扰力矩更新无人机状态
void Quadrotor::step(double dt)
{
  auto save = internal_state;

  // 求解常微分方程？
  // 调用了无人机动力学模型，但不是很清楚怎么实现这个操作的
  odeint::integrate(boost::ref(*this), internal_state, 0.0, dt, dt);

  // 检查数据有效性，如果数据无效，则使用上一时刻的数据进行赋值
  for (int i = 0; i < 22; ++i)
  {
    if (std::isnan(internal_state[i]))
    {
      std::cout << "dump " << i << " << pos ";
      for (int j = 0; j < 22; ++j)
      {
        std::cout << save[j] << " ";
      }
      std::cout << std::endl;
      internal_state = save;
      break;
    }
  }

  // 将结果给quad_state赋值
  for (int i = 0; i < 3; i++)
  {
    quad_state.pos(i) = internal_state[0 + i];
    quad_state.vel(i) = internal_state[3 + i];
    quad_state.R(i, 0) = internal_state[6 + i];
    quad_state.R(i, 1) = internal_state[9 + i];
    quad_state.R(i, 2) = internal_state[12 + i];
    quad_state.omega(i) = internal_state[15 + i];
  }
  quad_state.motor_rpm(0) = internal_state[18];
  quad_state.motor_rpm(1) = internal_state[19];
  quad_state.motor_rpm(2) = internal_state[20];
  quad_state.motor_rpm(3) = internal_state[21];

  // Re-orthonormalize R (polar decomposition)
  Eigen::LLT<Eigen::Matrix3d> llt(quad_state.R.transpose() * quad_state.R);
  Eigen::Matrix3d             P = llt.matrixL();
  Eigen::Matrix3d             R = quad_state.R * P.inverse();
  quad_state.R                      = R;

  // 保证无人机高度不会低于地面
  if (quad_state.pos(2) < 0.0 && quad_state.vel(2) < 0)
  {
    quad_state.pos(2) = 0;
    quad_state.vel(2) = 0;
  }

  // 将位置、速度、旋转矩阵、角速度、电机转速放入同一个向量中
  updateInternalState();
}

// 无人机动力学模型
void Quadrotor::operator()(const Quadrotor::InternalState& internal_state,
                      Quadrotor::InternalState& dxdt, const double /* t */)
{
  State cur_state;
  for (int i = 0; i < 3; i++)
  {
    cur_state.pos(i) = internal_state[0 + i];
    cur_state.vel(i) = internal_state[3 + i];
    cur_state.R(i, 0) = internal_state[6 + i];
    cur_state.R(i, 1) = internal_state[9 + i];
    cur_state.R(i, 2) = internal_state[12 + i];
    cur_state.omega(i) = internal_state[15 + i];
  }
  for (int i = 0; i < 4; i++)
  {
    cur_state.motor_rpm(i) = internal_state[18 + i];
  }

  // Re-orthonormalize R (polar decomposition)
  Eigen::LLT<Eigen::Matrix3d> llt(cur_state.R.transpose() * cur_state.R);
  Eigen::Matrix3d             P = llt.matrixL();
  Eigen::Matrix3d             R = cur_state.R * P.inverse();

  Eigen::Vector3d x_dot, v_dot, omega_dot;
  Eigen::Matrix3d R_dot;
  Eigen::Array4d  motor_rpm_dot;
  Eigen::Vector3d vnorm;
  Eigen::Array4d  motor_rpm_sq;
  Eigen::Matrix3d omega_vee(Eigen::Matrix3d::Zero());

  omega_vee(2, 1) = cur_state.omega(0);
  omega_vee(1, 2) = -cur_state.omega(0);
  omega_vee(0, 2) = cur_state.omega(1);
  omega_vee(2, 0) = -cur_state.omega(1);
  omega_vee(1, 0) = cur_state.omega(2);
  omega_vee(0, 1) = -cur_state.omega(2);

  motor_rpm_sq = cur_state.motor_rpm.array().square();

  // //! @todo implement
  // Eigen::Array4d blade_linear_velocity;
  // Eigen::Array4d motor_linear_velocity;
  // Eigen::Array4d AOA;
  // blade_linear_velocity = 0.104719755 // rpm to rad/s
  //                         * cur_state.motor_rpm.array() * prop_radius;
  // for (int i = 0; i < 4; ++i)
  //   AOA[i]   = alpha0 - atan2(motor_linear_velocity[i], blade_linear_velocity[i]) * 180 / 3.14159265;
  // //! @todo end

  // double totalF = motor_kf * motor_rpm_sq.sum();
  double thrust = motor_kf * motor_rpm_sq.sum();

  Eigen::Vector3d moments;
  moments(0) = motor_kf * (motor_rpm_sq(2) - motor_rpm_sq(3)) * arm_length;
  moments(1) = motor_kf * (motor_rpm_sq(1) - motor_rpm_sq(0)) * arm_length;
  moments(2) = motor_km * (motor_rpm_sq(0) + motor_rpm_sq(1) - motor_rpm_sq(2) -
                      motor_rpm_sq(3));

  double resistance = 0.1 *                                        // C
                      3.14159265 * (arm_length) * (arm_length) * // S
                      cur_state.vel.norm() * cur_state.vel.norm();

  //  ROS_INFO("resistance: %lf, Thrust: %lf%% ", resistance,
  //           motor_rpm_sq.sum() / (4 * motor_max_rpm * motor_max_rpm) * 100.0);

  vnorm = cur_state.vel;
  if (vnorm.norm() != 0)
  {
    vnorm.normalize();
  }
  x_dot = cur_state.vel;
  v_dot = -Eigen::Vector3d(0, 0, gravity) + thrust * R.col(2) / quad_mass +
          disturbance_force / quad_mass /*; //*/ - resistance * vnorm / quad_mass;

  quad_acc = v_dot;
  //  quad_acc[2] = -quad_acc[2]; // to NED

  R_dot = R * omega_vee;
  omega_dot =
    quad_J.inverse() *
    (moments - cur_state.omega.cross(quad_J * cur_state.omega) + disturbance_moment);
  motor_rpm_dot = (motor_rpm_cmd - cur_state.motor_rpm) / motor_time_constant;

  for (int i = 0; i < 3; i++)
  {
    dxdt[0 + i]  = x_dot(i);
    dxdt[3 + i]  = v_dot(i);
    dxdt[6 + i]  = R_dot(i, 0);
    dxdt[9 + i]  = R_dot(i, 1);
    dxdt[12 + i] = R_dot(i, 2);
    dxdt[15 + i] = omega_dot(i);
  }
  for (int i = 0; i < 4; i++)
  {
    dxdt[18 + i] = motor_rpm_dot(i);
  }
  for (int i = 0; i < 22; ++i)
  {
    if (std::isnan(dxdt[i]))
    {
      dxdt[i] = 0;
      //      std::cout << "nan apply to 0 for " << i << std::endl;
    }
  }
}

// 设置电机控制量（即电机转速期望值）
void Quadrotor::setInput(double u1, double u2, double u3, double u4)
{
  // Inputs are desired RPM for the motors
  // Rotor numbering is:
  //   *1*    Front
  // 3     4
  //    2
  motor_rpm_cmd(0) = u1;
  motor_rpm_cmd(1) = u2;
  motor_rpm_cmd(2) = u3;
  motor_rpm_cmd(3) = u4;
  
  // 限幅处理
  for (int i = 0; i < 4; i++)
  {
    if (std::isnan(motor_rpm_cmd(i)))
    {
      motor_rpm_cmd(i) = (motor_max_rpm + motor_min_rpm) / 2;
      std::cout << "NAN input ";
    }
    if (motor_rpm_cmd(i) > motor_max_rpm)
      motor_rpm_cmd(i) = motor_max_rpm;
    else if (motor_rpm_cmd(i) < motor_min_rpm)
      motor_rpm_cmd(i) = motor_min_rpm;
  }
}

const Quadrotor::State& Quadrotor::getState(void) const
{
  return quad_state;
}

void Quadrotor::setState(const Quadrotor::State& state)
{
  quad_state.pos         = state.pos;
  quad_state.vel         = state.vel;
  quad_state.R         = state.R;
  quad_state.omega     = state.omega;
  quad_state.motor_rpm = state.motor_rpm;

  updateInternalState();
}

void Quadrotor::setStatePos(const Eigen::Vector3d& Pos)
{
  quad_state.pos = Pos;

  updateInternalState();
}

double Quadrotor::getMass(void) const
{
  return quad_mass;
}

void Quadrotor::setMass(double mass)
{
  quad_mass = mass;
}

double Quadrotor::getGravity(void) const
{
  return gravity;
}

void Quadrotor::setGravity(double g)
{
  gravity = g;
}

const Eigen::Matrix3d& Quadrotor::getInertia(void) const
{
  return quad_J;
}

void Quadrotor::setInertia(const Eigen::Matrix3d& inertia)
{
  if (inertia != inertia.transpose())
  {
    std::cerr << "Inertia matrix not symmetric, not setting" << std::endl;
    return;
  }
  quad_J = inertia;
}

double Quadrotor::getArmLength(void) const
{
  return arm_length;
}

void Quadrotor::setArmLength(double d)
{
  if (d <= 0)
  {
    std::cerr << "Arm length <= 0, not setting" << std::endl;
    return;
  }

  arm_length = d;
}

double Quadrotor::getPropRadius(void) const
{
  return prop_radius;
}

void Quadrotor::setPropRadius(double r)
{
  if (r <= 0)
  {
    std::cerr << "Prop radius <= 0, not setting" << std::endl;
    return;
  }
  prop_radius = r;
}

double Quadrotor::getPropellerThrustCoefficient(void) const
{
  return motor_kf;
}

void Quadrotor::setPropellerThrustCoefficient(double kf)
{
  if (kf <= 0)
  {
    std::cerr << "Thrust coefficient <= 0, not setting" << std::endl;
    return;
  }

  motor_kf = kf;
}

double Quadrotor::getPropellerMomentCoefficient(void) const
{
  return motor_km;
}

void Quadrotor::setPropellerMomentCoefficient(double km)
{
  if (km <= 0)
  {
    std::cerr << "Moment coefficient <= 0, not setting" << std::endl;
    return;
  }

  motor_km = km;
}

double Quadrotor::getMotorTimeConstant(void) const
{
  return motor_time_constant;
}

void Quadrotor::setMotorTimeConstant(double k)
{
  if (k <= 0)
  {
    std::cerr << "Motor time constant <= 0, not setting" << std::endl;
    return;
  }

  motor_time_constant = k;
}

const Eigen::Vector3d& Quadrotor::getExternalForce(void) const
{
  return disturbance_force;
}

// 设置外部干扰力（惯性系）
void Quadrotor::setExternalForce(const Eigen::Vector3d& force)
{
  disturbance_force = force;
}

const Eigen::Vector3d& Quadrotor::getExternalMoment(void) const
{
  return disturbance_moment;
}

// 设置外部干扰力矩（惯性系）
void Quadrotor::setExternalMoment(const Eigen::Vector3d& moment)
{
  disturbance_moment = moment;
}

double Quadrotor::getMaxRPM(void) const
{
  return motor_max_rpm;
}

void Quadrotor::setMaxRPM(double max_rpm)
{
  if (max_rpm <= 0)
  {
    std::cerr << "Max rpm <= 0, not setting" << std::endl;
    return;
  }
  motor_max_rpm = max_rpm;
}

double Quadrotor::getMinRPM(void) const
{
  return motor_min_rpm;
}
void Quadrotor::setMinRPM(double min_rpm)
{
  if (min_rpm < 0)
  {
    std::cerr << "Min rpm < 0, not setting" << std::endl;
    return;
  }
  motor_min_rpm = min_rpm;
}

Eigen::Vector3d Quadrotor::getAcc() const
{
  return quad_acc;
}
}
