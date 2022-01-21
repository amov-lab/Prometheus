/***************************************************************************************************************************
 * ukf_ncv.h
 *
 * Author: Qyp
 *
 * Update Time: 2020.2.12
 *
 * 说明: UKF类,用于目标状态估计
 *      
 *      系统状态方程：
 *      观测方程：
***************************************************************************************************************************/
#ifndef UKF_H
#define UKF_H

#include <Eigen/Eigen>
#include <math.h>
#include <prometheus_msgs/DetectionInfo.h>

using namespace std;
using namespace Eigen;

class UKF_NCV
{
    public:

        //构造函数
        UKF_NCV(void):
            UKF_nh("~")
        {
            is_initialized = false;

            // Read the noise std
            UKF_nh.param<double>("UKF_NCV/std_ax_", NCV_proc_noise.std_ax_, 0.0);
            UKF_nh.param<double>("UKF_NCV/std_ay_", NCV_proc_noise.std_ay_, 0.0);
            UKF_nh.param<double>("UKF_NCV/std_az_", NCV_proc_noise.std_az_, 0.0);
            UKF_nh.param<double>("UKF_NCV/std_px_", NCV_meas_noise.std_px_, 0.0);
            UKF_nh.param<double>("UKF_NCV/std_py_", NCV_meas_noise.std_py_, 0.0);
            UKF_nh.param<double>("UKF_NCV/std_pz_", NCV_meas_noise.std_pz_, 0.0); 

            cout<<"NCV_proc_noise.std_ax_="<<NCV_proc_noise.std_ax_<<endl;
            cout<<"NCV_proc_noise.std_ay_="<<NCV_proc_noise.std_ay_<<endl;
            cout<<"NCV_proc_noise.std_az_="<<NCV_proc_noise.std_az_<<endl;
            cout<<"NCV_meas_noise.std_px_="<<NCV_meas_noise.std_px_<<endl;
            cout<<"NCV_meas_noise.std_py_="<<NCV_meas_noise.std_py_<<endl;
            cout<<"NCV_meas_noise.std_pz_="<<NCV_meas_noise.std_pz_<<endl;
            
            n_x_ = 6;
            n_noise_ = 3;
            n_aug_ = n_x_ + n_noise_;
            n_z_ = 3;
            x_ = VectorXd(n_x_);
            x_pre = VectorXd(n_x_);
            z_ = VectorXd(n_z_);

            P_pre = MatrixXd(n_x_,n_x_);
            
            P_ = MatrixXd(n_x_,n_x_);
            Q_ = MatrixXd(2,2);
            R_ = MatrixXd(n_z_,n_z_);

            P_ <<   NCV_meas_noise.std_px_*NCV_meas_noise.std_px_, 0, 0, 0, 0, 0,
                    0, NCV_meas_noise.std_py_*NCV_meas_noise.std_py_, 0, 0, 0, 0,
                    0, 0, NCV_meas_noise.std_pz_*NCV_meas_noise.std_pz_, 0, 0, 0,
                    0, 0, 0, 1, 0, 0,
                    0, 0, 0, 0, 1, 0,
                    0, 0, 0, 0, 0, 1;

            Q_ <<   NCV_proc_noise.std_ax_*NCV_proc_noise.std_ax_, 0, 0,
                    0, NCV_proc_noise.std_ay_*NCV_proc_noise.std_ay_, 0,
                    0, 0, NCV_proc_noise.std_az_*NCV_proc_noise.std_az_;

            R_ <<   NCV_meas_noise.std_px_*NCV_meas_noise.std_px_, 0, 0,
                    0, NCV_meas_noise.std_py_*NCV_meas_noise.std_py_, 0,
                    0, 0, NCV_meas_noise.std_pz_*NCV_meas_noise.std_pz_;

            cout<<"P_="<<endl<<P_<<endl<<endl;
            cout<<"Q_="<<endl<<Q_<<endl<<endl;
            cout<<"R_="<<endl<<R_<<endl<<endl;
            
            kamma_ = 3 - n_aug_;
            
            //set weights
            W_s = VectorXd(2*n_aug_+1);
            W_s(0) = kamma_/(kamma_+n_aug_);
            for(int i=1; i<2*n_aug_+1; ++i)
            {
                W_s(i) = 1/(2*kamma_+2*n_aug_);
            }

            Xsig_pred_ = MatrixXd(n_x_,2*n_aug_+1);    

            cout<<"[UKF]: "<<"NCV model selected."<<endl;
        }

        //initially set to false, set to ture in first call of Run()
        bool is_initialized;

        int n_x_;       //系统状态维数
        int n_noise_;   //过程噪声维数
        int n_aug_;     //增广维数 = 系统状态维数 + 过程噪声维数
        int n_z_;       //测量状态维数

        VectorXd x_;        //系统状态变量 即 x(k)
        VectorXd x_pre;     //预测的系统状态变量 即 x(k|k-1)
        VectorXd z_;        //测量值

        MatrixXd Q_;        //过程噪声协方差矩阵
        MatrixXd R_;        //测量噪声协方差矩阵
        MatrixXd P_pre;     //预测状态误差协方差矩阵 即 P(k|k-1)
        MatrixXd P_;        //状态后验协方差矩阵

        double kamma_;          //sigma点缩放系数
        VectorXd W_s;           //sigma点权重
        MatrixXd Xsig_pred_;    //sigma点预测矩阵

        // process noise standard deviation
        // NCV model
        struct
        {
            double std_ax_;
            double std_ay_;
            double std_az_;
        }NCV_proc_noise;        

        // measurement noise standard deviation
        // Vision measurement
        struct
        {
            double std_px_;
            double std_py_;
            double std_pz_;
        }NCV_meas_noise;

        // Process Measurement
        void ProcessMeasurement();

        // Prediction 
        // delta_t in s
        void Prediction(double delta_t);

        // Update Vision Measurement
        void UpdateVision(const prometheus_msgs::DetectionInfo& mesurement);

        // UKF main function
        VectorXd Run(const prometheus_msgs::DetectionInfo& mesurement, double delta);

    private:

        ros::NodeHandle UKF_nh;

};

VectorXd UKF_NCV::Run(const prometheus_msgs::DetectionInfo& mesurement, double delta_t)
{
    if(!is_initialized)
    {
        is_initialized = true;

        x_[0] = mesurement.position[0];
        x_[1] = mesurement.position[1];
        x_[2] = mesurement.position[2];
        x_[3] = 0.0;
        x_[4] = 0.0;
        x_[5] = 0.0;
        cout<<"[UKF]: "<<"NCV model is initialized."<<endl;

    }    
    // 预测
    Prediction(delta_t);

    UpdateVision(mesurement);

    return x_;
}

void UKF_NCV::Prediction(double delta_t)
{
    // 【UKF第一步】 构造sigma点
    
    // x_aug为x_的增广状态向量 维度 = 原系统维度+系统噪声维度
    VectorXd x_aug = VectorXd(n_aug_);
    x_aug.head(6) = x_;
    x_aug[7] = 0.0;
    x_aug[8] = 0.0;
    x_aug[9] = 0.0;

    // P_aug为P_阵的增广矩阵
    MatrixXd P_aug = MatrixXd(n_aug_,n_aug_);
    P_aug.fill(0.0);
    P_aug.topLeftCorner(6,6) = P_;
    P_aug.bottomRightCorner(3,3) = Q_;

    cout<<"P_aug="<<endl<<P_aug<<endl<<endl;

    //Xsig_aug为产生的2na+1个sigma点
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
    Xsig_aug.fill(0.0);

    //llt()是Cholesky 分解
    //Cholesky分解是把一个对称正定的矩阵表示成一个下三角矩阵L和其转置的乘积的分解
    //即 P_aug = L*L^t;
    MatrixXd L = P_aug.llt().matrixL();
    for(int i=0; i<2*n_aug_+1; ++i)
    {
        //第i列
        Xsig_aug.col(i) = x_aug;
    }
    //matrix.block<p,q>(i,j) : 从 (i,j) 开始，大小为 (p,q) 矩阵块
    Xsig_aug.block<9,9>(0,1) += sqrt(kamma_+n_aug_)*L;
    Xsig_aug.block<9,9>(0,n_aug_+1) -= sqrt(kamma_+n_aug_)*L;

    //cout<<"Xsig_aug="<<endl<<Xsig_aug<<endl<<endl;

    // 【UKF第二步】 时间更新（预测） - 利用系统方程对状态预测
    for(int i=0; i<2*n_aug_+1; ++i)
    {
        double p_x            = Xsig_aug(0,i);
        double p_y            = Xsig_aug(1,i);
        double p_z            = Xsig_aug(2,i);
        double v_x            = Xsig_aug(3,i);
        double v_y            = Xsig_aug(4,i);
        double v_z            = Xsig_aug(5,i);
        double nu_a_x         = Xsig_aug(6,i);
        double nu_a_y         = Xsig_aug(7,i);
        double nu_a_z         = Xsig_aug(8,i);

        double px_pred, py_pred, pz_pred;
        double vx_pred, vy_pred, vz_pred;
        px_pred = p_x + v_x*delta_t;
        py_pred = p_y + v_y*delta_t;
        pz_pred = p_z + v_z*delta_t; 
        vx_pred = v_x;
        vy_pred = v_y;
        vz_pred = v_z;

        //add noise
        px_pred += 0.5*nu_a_x*delta_t*delta_t;
        py_pred += 0.5*nu_a_y*delta_t*delta_t;
        pz_pred += 0.5*nu_a_z*delta_t*delta_t; 
        vx_pred += nu_a_x*delta_t;
        vy_pred += nu_a_y*delta_t;
        vz_pred += nu_a_z*delta_t; 
        
        // Xsig_pred_为 sigma点经过系统方程的非线性变化后得到
        Xsig_pred_(0,i) = px_pred;
        Xsig_pred_(1,i) = py_pred;
        Xsig_pred_(2,i) = pz_pred;
        Xsig_pred_(3,i) = vx_pred;
        Xsig_pred_(4,i) = vy_pred;
        Xsig_pred_(5,i) = vz_pred;
    }
    //cout<<"Xsig_pred_="<<endl<<Xsig_pred_<<endl<<endl;

    // 预测状态
    x_pre.fill(0.0);
    for (int i=0; i<2*n_aug_+1; ++i)
    {
        x_pre += W_s(i)*Xsig_pred_.col(i);
    } 

    cout<<"x_pre="<<endl<<x_pre<<endl<<endl;

    // 预测协方差矩阵
    P_pre.fill(0.0);
    for (int i=0; i<2*n_aug_+1; ++i)
    {
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_pre;

        P_pre +=  W_s(i)*x_diff*x_diff.transpose();
    }

    cout<<"P_pre="<<endl<<P_pre<<endl<<endl;
}

void UKF_NCV::UpdateVision(const prometheus_msgs::DetectionInfo& mesurement)
{  
    // 【UKF第三步】 测量更新
    z_[0] = mesurement.position[0];
    z_[1] = mesurement.position[1];
    z_[2] = mesurement.attitude[2];

    // Zsig为 Xsig_pred_经过测量方程的非线性变化后得到 
    MatrixXd Zsig = MatrixXd(n_z_, 2*n_aug_+1);
    //观测预测值 - 观测方程
    Zsig.fill(0.0);
    for (int i=0; i<2*n_aug_+1; i++)
    {
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        double p_z = Xsig_pred_(2,i);

        Zsig(0,i) = p_x;                      
        Zsig(1,i) = p_y;                               
        Zsig(2,i) = p_z;  
    }
    // z_pred为预测观测值
    MatrixXd z_pred = VectorXd(n_z_);
    z_pred.fill(0.0);
    for (int i=0; i < 2*n_aug_+1; ++i) 
    {
        z_pred = z_pred + W_s(i) * Zsig.col(i);
    }  

    MatrixXd S_ = MatrixXd(n_z_,n_z_); //预测测量误差协方差矩阵
    MatrixXd T_ = MatrixXd(n_x_,n_z_);  //状态与测量空间相关函数
    S_.fill(0.0);
    T_.fill(0.0);

    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        S_ = S_ + W_s(i) * z_diff * z_diff.transpose();
        
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        T_ = T_ + W_s(i) * x_diff * z_diff.transpose();
    }  

    S_ = S_ + R_;  // add measurement noise covariance matrix

    MatrixXd K_= MatrixXd(n_x_,n_z_);       //卡尔曼增益K_ 
    K_ = T_ * S_.inverse();   // Kalman gain K;

    VectorXd z_diff = z_ - z_pred;    // residual

    // 【UKF第四步】 更新状态及P_阵
    //zheli duima ?
    x_ = x_pre + K_*z_diff;
    P_ = P_pre - K_*S_*K_.transpose();

    cout<<"x_="<<endl<<x_<<endl<<endl;
    cout<<"P_="<<endl<<P_<<endl<<endl;
}


#endif
