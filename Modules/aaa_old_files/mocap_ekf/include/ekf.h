#ifndef MotionEstimation_EKF_H
#define MotionEstimation_EKF_H

#include <iostream>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <queue>

using namespace std;
using namespace Eigen;
//! @brief Common variables
const double PI = 3.141592653589793;
const double TAU = 6.283185307179587;

void system_pub(ros::Time stamp);
void cam_system_pub(ros::Time stamp);
void acc_f_pub(Vector3d acc, ros::Time stamp);

//process model
void initsys();
// void getState(Vector3d& p, Vector3d& q, Vector3d& v);//p q v
void getState(Vector3d& p, Vector3d& q, Vector3d& v, Vector3d& bg, Vector3d& ba); //p q v bias
VectorXd get_filtered_acc(Vector3d acc);
VectorXd F_model(Vector3d gyro, Vector3d acc);
VectorXd g_model();
// MatrixXd diff_f_diff_x(Vector3d q_last, Vector3d gyro, Vector3d acc); // p q v
MatrixXd diff_f_diff_x(Vector3d q_last, Vector3d gyro, Vector3d acc, Vector3d bg_last, Vector3d ba_last); //  p q v bias
MatrixXd diff_f_diff_n(Vector3d q_last);
MatrixXd diff_g_diff_x();
MatrixXd diff_g_diff_v();

#define Jacobian_diff_f_diff_x 0, 0, 0, 0, 0, 0, 1, 0, 0,  0, 0, 0, 0,  0,0, 0, 1, 0,  0, 0, 0, 0,  0,0, 0, 0, 1,  0, 0, 0, (sp*(Wy*cr - Wz*sr))/cp, (Wz*cr + Wy*sr)/(cp*cp),0, 0, 0, 0,  0, 0, 0,  - Wz*cr - Wy*sr,  0,0, 0, 0, 0,0, 0, 0, (Wy*cr - Wz*sr)/cp, (sp*(Wz*cr + Wy*sr))/(cp*cp),0, 0, 0, 0,0, 0, 0,Ay*(sr*sy + cr*cy*sp) + Az*(cr*sy - cy*sp*sr), Az*cp*cr*cy - Ax*cy*sp + Ay*cp*cy*sr, Az*(cy*sr - cr*sp*sy) - Ay*(cr*cy + sp*sr*sy) - Ax*cp*sy, 0, 0, 0, 0, 0, 0, - Ay*(cy*sr - cr*sp*sy) - Az*(cr*cy + sp*sr*sy), Az*cp*cr*sy - Ax*sp*sy + Ay*cp*sr*sy, Az*(sr*sy + cr*cy*sp) - Ay*(cr*sy - cy*sp*sr) + Ax*cp*cy, 0, 0, 0,0, 0, 0,  Ay*cp*cr - Az*cp*sr,  - Ax*cp - Az*cr*sp - Ay*sp*sr,0, 0, 0, 0

// state for kalman filter
// 0-2 Px Py Pz
// 3-5 euler
// 6-8 Vx Vy Vz

class ekf_estimation
{
public:
	ekf_estimation();
	~ekf_estimation();	
	void predict(Vector3d gyro, Vector3d acc, double t);
	void correct(Vector3d pos, Vector3d vel, Vector3d mag, double t);
	void process(Vector3d gyro, Vector3d acc, VectorXd& xdot, MatrixXd& F, MatrixXd& G);
	MatrixXd computeF(Vector3d gyro, Vector3d acc);
	
	VectorXd measurement(VectorXd x, Vector3d mag);
	MatrixXd computeH(Vector3d mag);

	void measurement_fix(Vector2d& position, MatrixXd &H);
	void measurement_fix_velocity(Vector3d& velocity, MatrixXd& H);
	void measurement_sonar_height(VectorXd& sonar_height, MatrixXd& H);
	void measurement_magnetic_field(Vector3d& magnetic_field, MatrixXd& H);
	void measurement_gravity(Vector3d& acc, MatrixXd& H);

	void correct(VectorXd z, VectorXd zhat, MatrixXd H, MatrixXd R);
	void correct_fix(Vector3d position, double t);
	void correct_fix_velocity(Vector3d velocity, double t);
	void correct_sonar_height(double sonar_height, double t);//todo, without considering the roll and pitch
	void correct_magnetic_field(Vector3d mag, double t);
	void correct_gravity(Vector3d acc, double t);
	// void measurement_altimeter(double& altimeter_height, MatrixXd H);
	void getState(Quaterniond& q, Vector3d& position, Vector3d& velocity, Vector3d & bw, Vector3d&  ba);
	double get_time() { return current_t;}
private:
	VectorXd x;//state 
	MatrixXd P;//covariance


	const Vector3d GRAVITY = Vector3d(0, 0, 9.8);
	//covariance parameter
	const double fix_cov = 2.0;
	const double sonar_height_cov = 0.2;
	const double fix_velocity_cov = 2.0;
	
	const double gyro_cov = 0.01;
	const double acc_cov = 0.1;

	const double gravity_cov = 5.0;
	const double mag_cov = 5.0;

	const int n_state = 16;
	MatrixXd Q;//imu observation noise
	const MatrixXd R_fix = Matrix2d::Identity()*fix_cov;
	const MatrixXd R_fix_velocity = Matrix3d::Identity()*fix_velocity_cov;
	const MatrixXd R_sonar_height = MatrixXd::Identity(1, 1)*sonar_height_cov;
	const MatrixXd R_magnetic = Matrix3d::Identity()*mag_cov;
	const MatrixXd R_gravity = Matrix3d::Identity()*gravity_cov;

	Vector3d acc;
	Vector3d gyro;

	Vector3d referenceMagneticField_;
	double current_t;
	bool initialized;

	bool fix_initialized;
	bool imu_initialized;
	bool altimeter_initialized;
	bool sonar_initialized;
	bool magnetic_initialized;
	
};


#endif

