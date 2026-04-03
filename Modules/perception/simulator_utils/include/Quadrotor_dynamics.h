#ifndef __QUADROTOR_SIMULATOR_QUADROTOR_H__
#define __QUADROTOR_SIMULATOR_QUADROTOR_H__

#include <Eigen/Core>
#include <boost/array.hpp>

namespace QuadrotorSimulator
{
  class Quadrotor
  {
  public:
    // 无人机状态
    struct State
    {
      Eigen::Vector3d pos;      // 位置
      Eigen::Vector3d vel;      // 速度
      Eigen::Matrix3d R;        // 旋转矩阵
      Eigen::Vector3d omega;    // 角速度
      Eigen::Array4d motor_rpm; // 电机转速
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    Quadrotor();

    const Quadrotor::State &getState(void) const;

    void setState(const Quadrotor::State &state);

    void setStatePos(const Eigen::Vector3d &Pos);

    double getMass(void) const;
    void setMass(double mass);

    double getGravity(void) const;
    void setGravity(double g);

    const Eigen::Matrix3d &getInertia(void) const;
    void setInertia(const Eigen::Matrix3d &inertia);

    double getArmLength(void) const;
    void setArmLength(double d);

    double getPropRadius(void) const;
    void setPropRadius(double r);

    double getPropellerThrustCoefficient(void) const;
    void setPropellerThrustCoefficient(double kf);

    double getPropellerMomentCoefficient(void) const;
    void setPropellerMomentCoefficient(double km);

    double getMotorTimeConstant(void) const;
    void setMotorTimeConstant(double k);

    const Eigen::Vector3d &getExternalForce(void) const;
    void setExternalForce(const Eigen::Vector3d &force);

    const Eigen::Vector3d &getExternalMoment(void) const;
    void setExternalMoment(const Eigen::Vector3d &moment);

    double getMaxRPM(void) const;
    void setMaxRPM(double max_rpm);

    double getMinRPM(void) const;
    void setMinRPM(double min_rpm);

    // Inputs are desired RPM for the motors
    // Rotor numbering is:
    //   *1*    Front
    // 3     4
    //    2
    // with 1 and 2 clockwise and 3 and 4 counter-clockwise (looking from top)
    // 设置电机转速 要改构型 todo
    void setInput(double u1, double u2, double u3, double u4);

    // Runs the actual dynamics simulation with a time step of dt
    void step(double dt);

    // For internal use, but needs to be public for odeint
    typedef boost::array<double, 22> InternalState;
    void operator()(const Quadrotor::InternalState &internal_state, Quadrotor::InternalState &dxdt, const double /* t */);

    Eigen::Vector3d getAcc() const;

  private:
    void updateInternalState(void);

    double alpha0; // AOA
    double gravity;     // gravity
    double quad_mass;
    Eigen::Matrix3d quad_J; // Inertia
    double motor_kf;
    double motor_km;
    double prop_radius;
    double arm_length;
    double motor_time_constant; // unit: sec
    double motor_max_rpm;
    double motor_min_rpm;

    Quadrotor::State quad_state;

    Eigen::Vector3d quad_acc;

    Eigen::Array4d motor_rpm_cmd;
    Eigen::Vector3d disturbance_force;
    Eigen::Vector3d disturbance_moment;

    InternalState internal_state;
  };
} // namespace QuadrotorSimulator
#endif
