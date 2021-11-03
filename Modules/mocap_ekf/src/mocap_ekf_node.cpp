#include "ekf.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include "conversion.h"

using namespace std;
using namespace Eigen;

//X_state: p q v gb ab   with time stamp aligned between imu and img
/*
    EKF model
    prediction:
    xt~ = xt-1 + dt*f(xt-1, ut, 0)
    sigmat~ = Ft*sigmat-1*Ft' + Vt*Qt*Vt'
    Update:
    Kt = sigmat~*Ct'*(Ct*sigmat~*Ct' + Wt*Rt*Wt')^-1
    xt = xt~ + Kt*(zt - g(xt~,0))
    sigmat = sigmat~ - Kt*Ct*sigmat~
*/
/*
   -pi ~ pi crossing problem:
   1. the model prpagation: X_state should be limited to [-pi,pi] after predicting and updating
   2. inovation crossing: (measurement - g(X_state)) should also be limited to [-pi,pi] when getting the inovation.
   z_measurement is normally in [-pi~pi]
*/

// imu frame is imu body frame

//odom: pose px,py pz orientation qw qx qy qz
//imu: acc: x y z gyro: wx wy wz

ros::Publisher odom_pub;
ros::Publisher cam_odom_pub;

//state
geometry_msgs::Pose pose;
Vector3d position, orientation, velocity;

// Now set up the relevant matrices
//states X [p q pdot]  [px,py,pz, wx,wy,wz, vx,vy,vz]
size_t stateSize;                                // x = [p q pdot bg ba]
size_t stateSize_pqv;                                // x = [p q pdot]
size_t measurementSize;                          // z = [p q]
size_t inputSize;                                // u = [w a]
VectorXd X_state(stateSize);                     // x (in most literature)
VectorXd u_input;
VectorXd Z_measurement;                          // z
MatrixXd StateCovariance;                        // sigma
MatrixXd Kt_kalmanGain;                          // Kt
VectorXd X_state_correct(stateSize);                     // x (in most literature)
MatrixXd StateCovariance_correct;                        // sigma
MatrixXd Qt;
MatrixXd Rt;
Vector3d u_gyro;
Vector3d u_acc;
Vector3d gravity(0., 0., -9.8); //need to estimate the bias 9.8099
Vector3d bg_0(0., 0., 0); //need to estimate the bias
Vector3d ba_0(0., 0., 0); //need to estimate the bias  0.1
Vector3d ng(0., 0., 0.);
Vector3d na(0., 0., 0.);
Vector3d nbg(0., 0., 0.);
Vector3d nba(0., 0., 0.);

Vector3d q_last;
Vector3d bg_last;
Vector3d ba_last;

//Qt imu covariance matrix  smaller believe system(imu) more
double gyro_cov = 0.01;
double acc_cov = 0.01;
//Rt visual odomtry covariance smaller believe measurement more
double position_cov = 0.1;
double q_rp_cov = 0.1;
double q_yaw_cov = 0.1;

double dt = 0.005; //second
double t_last, t_now;  
bool first_frame_imu = true;
bool first_frame_tag_odom = true;
bool test_odomtag_call = false;
bool odomtag_call = false;

double time_now, time_last;
double time_odom_tag_now;
double diff_time;
//world frame points velocity
vector< pair<VectorXd, sensor_msgs::Imu> > sys_seq;  // keep a sequence of sys imu and X_state(before imu system prediction)
vector<MatrixXd> cov_seq;

//Rotation from the camera frame to the IMU frame
Matrix3d Rc_i;     
Vector3d tc_i;  //  cam in imu frame
int cnt = 0;
Vector3d INNOVATION_;
Matrix3d Rr_i;     
Vector3d tr_i;  //  rigid body in imu frame

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // first_frame_tag_odom 初始为true，收到vicon后设置为false
    if(!first_frame_tag_odom)
    { 
        // 第一次回调：发布初始值
        if(first_frame_imu)
        {
            first_frame_imu = false;
            time_now = msg->header.stamp.toSec();
            time_last = time_now;
            // 发布初始值
            system_pub(msg->header.stamp);
        }
        else
        {
            time_now = msg->header.stamp.toSec();
            // 时间差
            dt = time_now - time_last;

            if(odomtag_call)
            {
                odomtag_call = false;
                diff_time = time_now - time_odom_tag_now;
                if(diff_time<0)
                {
                    cout << "diff time: " << diff_time << endl;  //???!!! exist !!!???
                    cout << "timeimu: " << time_now - 1.60889e9 << " time_odom: " << time_odom_tag_now - 1.60889e9 << endl;
                    // cout << "diff time: " << diff_time << endl;  //about 30ms
                }
            }
            MatrixXd Ft;
            MatrixXd Vt;

            // 将imu存为输入值
            u_gyro(0) = msg->angular_velocity.x;
            u_gyro(1) = msg->angular_velocity.y;
            u_gyro(2) = msg->angular_velocity.z;
            u_acc(0)  = msg->linear_acceleration.x;
            u_acc(1)  = msg->linear_acceleration.y;
            u_acc(2)  = msg->linear_acceleration.z;

            // 上一时刻姿态
            q_last = X_state.segment<3>(3);  // last X2
            bg_last = X_state.segment<3>(9);  //last X4
            ba_last = X_state.segment<3>(12);  //last X5
            Ft = MatrixXd::Identity(stateSize, stateSize) + dt*diff_f_diff_x(q_last, u_gyro, u_acc, bg_last, ba_last);
         
            Vt = dt*diff_f_diff_n(q_last);

            // 使用输入更新状态
            X_state += dt*F_model(u_gyro, u_acc);
            // 欧拉角限制幅度
            if(X_state(3) > PI)  X_state(3) -= 2*PI;
            if(X_state(3) < -PI) X_state(3) += 2*PI;
            if(X_state(4) > PI)  X_state(4) -= 2*PI;
            if(X_state(4) < -PI) X_state(4) += 2*PI;
            if(X_state(5) > PI)  X_state(5) -= 2*PI;
            if(X_state(5) < -PI) X_state(5) += 2*PI;
            // 更新COV
            StateCovariance = Ft*StateCovariance*Ft.transpose() + Vt*Qt*Vt.transpose();
 
            time_last = time_now;
            
            system_pub(msg->header.stamp);
        }
    }
  
}

VectorXd get_pose_from_mocap(const geometry_msgs::PoseStamped::ConstPtr &msg) 
{
    Matrix3d Rr_w;    //rigid body in world
    Vector3d tr_w;
    Matrix3d Ri_w;  
    Vector3d ti_w;
    Vector3d p_temp;
    // 位置
    p_temp(0) = msg->pose.position.x;
    p_temp(1) = msg->pose.position.y;
    p_temp(2) = msg->pose.position.z;
    //quaternion2euler:  ZYX  roll pitch yaw
    Quaterniond q;
    // 姿态
    q.w() = msg->pose.orientation.w;
    q.x() = msg->pose.orientation.x;
    q.y() = msg->pose.orientation.y;
    q.z() = msg->pose.orientation.z;
    
    // 刚体的位置和姿态
    Rr_w = q.toRotationMatrix();
    tr_w = p_temp;
    // imu的位置和姿态
    Ri_w = Rr_w * Rr_i.inverse();
    ti_w = tr_w - Ri_w*tr_i;
    Vector3d euler = mat2euler(Ri_w);

    VectorXd pose = VectorXd::Random(6);
    // imu的位置和姿态
    pose.segment<3>(0) = ti_w;
    pose.segment<3>(3) = euler;

    return pose;
}

void mocap_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // 第一次回调
    if(first_frame_tag_odom)
    {
        first_frame_tag_odom = false;
        time_odom_tag_now = msg->header.stamp.toSec();

        VectorXd odom_pose = get_pose_from_mocap(msg);
        X_state.segment<3>(0) = odom_pose.segment<3>(0);
        X_state.segment<3>(3) = odom_pose.segment<3>(3);
    }
    else
    {
        time_odom_tag_now = msg->header.stamp.toSec();
        MatrixXd Ct;
        MatrixXd Wt;

        // 获取得到imu的位置姿态（imu和质心存在偏差）
        VectorXd odom_pose = get_pose_from_mocap(msg);
        // 更新测量值
        Z_measurement.segment<3>(0) = odom_pose.segment<3>(0);
        Z_measurement.segment<3>(3) = odom_pose.segment<3>(3);
        // 发布测量值（即mocap的原始值）
        cam_system_pub(msg->header.stamp);

        Ct = diff_g_diff_x();
        Wt = diff_g_diff_v();

        // 更新kalman增益
        Kt_kalmanGain = StateCovariance*Ct.transpose() * (Ct*StateCovariance*Ct.transpose() + Wt*Rt*Wt.transpose()).inverse();
        VectorXd gg = g_model();
        VectorXd innovation = Z_measurement - gg;
    
        //Prevent innovation changing suddenly when euler from -Pi to Pi
        if(innovation(3) > 6)  innovation(3) -= 2*PI;
        if(innovation(3) < -6) innovation(3) += 2*PI;
        if(innovation(4) > 6)  innovation(4) -= 2*PI;
        if(innovation(4) < -6) innovation(4) += 2*PI;
        if(innovation(5) > 6)  innovation(5) -= 2*PI;
        if(innovation(5) < -6) innovation(5) += 2*PI;
        INNOVATION_ = innovation.segment<3>(3);
        // 使用测量值更新状态
        X_state += Kt_kalmanGain*(innovation);
        if(X_state(3) > PI)  X_state(3) -= 2*PI;
        if(X_state(3) < -PI) X_state(3) += 2*PI;
        if(X_state(4) > PI)  X_state(4) -= 2*PI;
        if(X_state(4) < -PI) X_state(4) += 2*PI;
        if(X_state(5) > PI)  X_state(5) -= 2*PI;
        if(X_state(5) < -PI) X_state(5) += 2*PI;
        StateCovariance = StateCovariance - Kt_kalmanGain*Ct*StateCovariance;
        
        test_odomtag_call = true;
        odomtag_call = true;
        
        if(cnt == 10||cnt==50||cnt==90)
        {
            // cout << "Ct: \n" << Ct << "\nWt:\n" << Wt << endl; 
            // cout << "Kt_kalmanGain: \n" << Kt_kalmanGain << endl; 
            // cout << "\ninnovation: \n" << Kt_kalmanGain*innovation  << "\ndt:\n" << dt << endl;
            // cout << "\ninnovation: \n" << Kt_kalmanGain*innovation  << endl;
            // cout << "\ninnovation: \n" << INNOVATION_ << endl; 
        }
        cnt++;
        if(cnt>100) cnt=101;
    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mocap_ekf_node");
    ros::NodeHandle n("~");
    // 【订阅】 IMU数据，来自PX4
    ros::Subscriber s1 = n.subscribe("imu", 100, imu_callback, ros::TransportHints().tcpNoDelay());
    // 【订阅】 VICON数据
    ros::Subscriber s2 = n.subscribe("pose", 100, mocap_callback, ros::TransportHints().tcpNoDelay());
    // 【发布】 融合后的odom(发布频率为imu的频率)
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);  
    // 【发布】 相机odom？
    cam_odom_pub = n.advertise<nav_msgs::Odometry>("cam_ekf_odom", 100);  
    
    n.getParam("gyro_cov", gyro_cov);
    n.getParam("acc_cov", acc_cov);
    n.getParam("position_cov", position_cov);
    n.getParam("q_rp_cov", q_rp_cov);
    n.getParam("q_yaw_cov", q_yaw_cov);

    cout << "Q:" << gyro_cov << " " << acc_cov << " R: " << position_cov << " " << q_rp_cov << " " << q_yaw_cov << endl;

    // 初始化
    initsys();
    cout << "initsys" << endl;

    cout << "======================" << endl;
    double r = atan2(1,-100);
    double p = asin(-0.707);
    double y = atan2(-1, -100);   
    cout << "r: " << r << " p: " << p << " y: " << y << endl;
    cout << "======================" << endl;

    ros::spin();
}

void system_pub(ros::Time stamp)
{
    nav_msgs::Odometry odom_fusion;
    odom_fusion.header.stamp = stamp;
    odom_fusion.header.frame_id = "world";
    // odom_fusion.header.frame_id = "imu";
    odom_fusion.pose.pose.position.x = X_state(0);
    odom_fusion.pose.pose.position.y = X_state(1);
    odom_fusion.pose.pose.position.z = X_state(2);
    Quaterniond q;
    q = euler2quaternion(X_state.segment<3>(3));
    odom_fusion.pose.pose.orientation.w = q.w();
    odom_fusion.pose.pose.orientation.x = q.x();
    odom_fusion.pose.pose.orientation.y = q.y();
    odom_fusion.pose.pose.orientation.z = q.z();
    odom_fusion.twist.twist.linear.x = X_state(6);
    odom_fusion.twist.twist.linear.y = X_state(7);
    odom_fusion.twist.twist.linear.z = X_state(8);
    odom_pub.publish(odom_fusion);
}

void cam_system_pub(ros::Time stamp)
{
    // 测量值
    nav_msgs::Odometry odom_fusion;
    odom_fusion.header.stamp = stamp;
    odom_fusion.header.frame_id = "world";

    odom_fusion.pose.pose.position.x = Z_measurement(0);
    odom_fusion.pose.pose.position.y = Z_measurement(1);
    odom_fusion.pose.pose.position.z = Z_measurement(2);
    Quaterniond q;
    q = euler2quaternion(Z_measurement.segment<3>(3));
    odom_fusion.pose.pose.orientation.w = q.w();
    odom_fusion.pose.pose.orientation.x = q.x();
    odom_fusion.pose.pose.orientation.y = q.y();
    odom_fusion.pose.pose.orientation.z = q.z();

    odom_fusion.twist.twist.angular.x = diff_time;
    odom_fusion.twist.twist.angular.y = dt;
    cam_odom_pub.publish(odom_fusion);
}

//process model
void initsys()
{
    //  camera position in the IMU frame = (0.05, 0.05, 0)
    // camera orientaion in the IMU frame = Quaternion(0, 1, 0, 0); w x y z, respectively
    //					   RotationMatrix << 1, 0, 0,
    //							             0, -1, 0,
    //                                       0, 0, -1;
    //set the cam2imu params
    Rc_i = Quaterniond(0, 1, 0, 0).toRotationMatrix();
    // cout << "R_cam" << endl << Rc_i << endl;
    tc_i << 0.05, 0.05, 0; 

    Rr_i = Quaterniond(1, 0, 0, 0).toRotationMatrix();
    tr_i << 0, 0, 0.055;
    cout << "Rr_i: " << endl << Rr_i << endl;
    cout << "tr_i: " << endl << tr_i << endl;
    //  rigid body position in the IMU frame = (0, 0, 0.04)
    // rigid body orientaion in the IMU frame = Quaternion(1, 0, 0, 0); w x y z, respectively
    //					   RotationMatrix << 1, 0, 0,
    //							             0, 1, 0,
    //                                       0, 0, 1; 

    //states X 
    // 状态维度 [p q pdot bg ba]  [px,py,pz, wx,wy,wz（欧拉角）, vx,vy,vz bgx,bgy,bgz bax,bay,baz]
    stateSize = 15;                                                                 // x = [p q pdot bg ba]
    stateSize_pqv = 9;                                                              // x = [p q pdot]
    // 测量维度 位置+姿态
    measurementSize = 6;                                                            // z = [p q]
    // 输入维度 w是角速度 a是加速度
    inputSize = 6;                                                                  // u = [w a]
    X_state = VectorXd::Zero(stateSize);                                            // x 
    //velocity
    X_state(6) = 0;
    X_state(7) = 0;
    X_state(8) = 0; 
    // bias
    X_state.segment<3>(9) = bg_0;
    X_state.segment<3>(12) = ba_0;
    // 输入
    u_input = VectorXd::Zero(inputSize);
    // 测量
    Z_measurement = VectorXd::Zero(measurementSize);                                // z
    // 状态cov
    StateCovariance = MatrixXd::Identity(stateSize, stateSize);                     // sigma
    // kalman增益
    Kt_kalmanGain = MatrixXd::Identity(stateSize, measurementSize);                 // Kt
    // Ct_stateToMeasurement = MatrixXd::Identity(stateSize, measurementSize);         // Ct
    // ？
    X_state_correct = X_state;
    // ？
    StateCovariance_correct = StateCovariance;

    Qt = MatrixXd::Identity(inputSize, inputSize);  //6x6 input [gyro acc]covariance
    Rt = MatrixXd::Identity(measurementSize, measurementSize); //6x6 measurement [p q]covariance

    // You should also tune these parameters
    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    // //Rt visual odomtry covariance smaller believe measurement more
    Qt.topLeftCorner(3, 3) = gyro_cov * Qt.topLeftCorner(3, 3);
    Qt.bottomRightCorner(3, 3) = acc_cov * Qt.bottomRightCorner(3, 3);
    Rt.topLeftCorner(3, 3) = position_cov * Rt.topLeftCorner(3, 3);
    Rt.bottomRightCorner(3, 3) = q_rp_cov * Rt.bottomRightCorner(3, 3);
    Rt.bottomRightCorner(1, 1) = q_yaw_cov * Rt.bottomRightCorner(1, 1);
}

void getState(Vector3d& p, Vector3d& q, Vector3d& v, Vector3d& bg, Vector3d& ba)
{
	p = X_state.segment<3>(0);
	q = X_state.segment<3>(3);
	v = X_state.segment<3>(6);
    bg = X_state.segment<3>(9);
    ba = X_state.segment<3>(12);
}

VectorXd F_model(Vector3d gyro, Vector3d acc)
{
    // IMU is in FLU frame
    // Transform IMU frame into "world" frame whose original point is FLU's original point and the XOY plain is parallel with the ground and z axis is up
    VectorXd f(VectorXd::Zero(stateSize));
    Vector3d p, q, v, bg, ba;
    getState(p, q, v, bg, ba);
    f.segment<3>(0) = v;
    f.segment<3>(3) = w_Body2Euler(q)*(gyro-bg-ng);
    f.segment<3>(6) = gravity + euler2mat(q)*(acc-ba-na);
    f.segment<3>(9) = nbg;
    f.segment<3>(12) = nba;
    return f;
}

VectorXd g_model()
{
    VectorXd g(VectorXd::Zero(measurementSize));

    g.segment<6>(0) = X_state.segment<6>(0);

    return g;
}

//F_model G_model Jocobian
//diff_f()/diff_x (x_t-1  ut  noise=0)   At     Ft = I+dt*At
MatrixXd diff_f_diff_x(Vector3d q_last, Vector3d gyro, Vector3d acc, Vector3d bg_last, Vector3d ba_last)
{
    double cr = cos( q_last(0));
    double sr = sin( q_last(0));
    double cp = cos( q_last(1));
    double sp = sin( q_last(1));
    double cy = cos( q_last(2));
    double sy = sin( q_last(2));
 
    // ng na = 0 nbg nba = 0
    double Ax = acc(0) - ba_last(0);
    double Ay = acc(1) - ba_last(1);
    double Az = acc(2) - ba_last(2);
    // double Wx = gyro(0) - bg_last(0);
    double Wy = gyro(1) - bg_last(1);
    double Wz = gyro(2) - bg_last(2);

    MatrixXd diff_f_diff_x_jacobian(MatrixXd::Zero(stateSize, stateSize));
    MatrixXd diff_f_diff_x_jacobian_pqv(MatrixXd::Zero(stateSize_pqv, stateSize_pqv));

    diff_f_diff_x_jacobian_pqv <<  0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1, 0, 
      0, 0, 0, 0, 0, 0, 0, 0, 1,  
      0, 0, 0, (sp*(Wy*cr - Wz*sr))/cp, (Wz*cr + Wy*sr)/(cp*cp), 0, 0, 0, 0,  
      0, 0, 0, (- Wz*cr - Wy*sr), 0, 0, 0, 0, 0, 
      0, 0, 0, (Wy*cr - Wz*sr)/cp, (sp*(Wz*cr + Wy*sr))/(cp*cp), 0, 0, 0, 0, 
      0, 0, 0, (Ay*(sr*sy + cr*cy*sp) + Az*(cr*sy - cy*sp*sr)), (Az*cp*cr*cy - Ax*cy*sp + Ay*cp*cy*sr), (Az*(cy*sr - cr*sp*sy) - Ay*(cr*cy + sp*sr*sy) - Ax*cp*sy), 0, 0, 0, 
      0, 0, 0, (- Ay*(cy*sr - cr*sp*sy) - Az*(cr*cy + sp*sr*sy)), (Az*cp*cr*sy - Ax*sp*sy + Ay*cp*sr*sy), (Az*(sr*sy + cr*cy*sp) - Ay*(cr*sy - cy*sp*sr) + Ax*cp*cy), 0, 0, 0,
      0, 0, 0, (Ay*cp*cr - Az*cp*sr), (- Ax*cp - Az*cr*sp - Ay*sp*sr), 0, 0, 0, 0 ;
    
    diff_f_diff_x_jacobian.block<9, 9>(0, 0) = diff_f_diff_x_jacobian_pqv;
    diff_f_diff_x_jacobian.block<3, 3>(3, 9) = -w_Body2Euler(q_last);
    diff_f_diff_x_jacobian.block<3, 3>(6, 12) = -euler2mat(q_last);

    return diff_f_diff_x_jacobian;

    // cp != 0 pitch != 90° !!!!!!!!!
}
//diff_f()/diff_n (x_t-1  ut  noise=0)  Ut    Vt = dt*Ut
MatrixXd diff_f_diff_n(Vector3d q_last)
{
    MatrixXd diff_f_diff_n_jacobian(MatrixXd::Zero(stateSize, inputSize));
    diff_f_diff_n_jacobian.block<3,3>(3,0) = -w_Body2Euler(q_last);
    diff_f_diff_n_jacobian.block<3,3>(6,3) = -euler2mat(q_last);

    return diff_f_diff_n_jacobian;
}
//diff_g()/diff_x  (xt~ noise=0)  Ct 
MatrixXd diff_g_diff_x()
{
    MatrixXd diff_g_diff_x_jacobian(MatrixXd::Zero(measurementSize, stateSize));
    diff_g_diff_x_jacobian.block<3,3>(0,0) = MatrixXd::Identity(3,3);
    diff_g_diff_x_jacobian.block<3,3>(3,3) = MatrixXd::Identity(3,3);

    return diff_g_diff_x_jacobian;
}
//diff_g()/diff_v  (xt~ noise=0) Wt
MatrixXd diff_g_diff_v()
{
    MatrixXd diff_g_diff_v_jacobian(MatrixXd::Identity(measurementSize, measurementSize));

    return diff_g_diff_v_jacobian;
}
