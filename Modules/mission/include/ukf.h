/***************************************************************************************************************************
 * ukf.h
 *
 * Author: Qyp
 *
 * Update Time: 2020.1.12
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

class UKF
{
    public:

        //构造函数
        UKF(int _model_type):
            UKF_nh("~")
        {
            model_type = _model_type;

            is_initialized = false;

            // NCA model
            if(model_type == NCA)
            {
                cout<<"[UKF]: "<<"NCA model selected."<<endl;
                
                n_x_ = 9;
                n_noise_ = 3;
                n_aug_ = n_x_ + n_noise_;
                x_ = VectorXd(n_x_);
                Xsig_pred_ = MatrixXd(n_x_,2*n_aug_+1);
                kamma_ = 3 - n_aug_;
                P_ = MatrixXd(n_x_, n_x_);
                
                //set weights
                W_s = VectorXd(2*n_aug_+1);
                W_s(0) = kamma_/(kamma_+n_aug_);
                for(int i=1; i<2*n_aug_+1; ++i)
                {
                    W_s(i) = 1/(2*kamma_+2*n_aug_);
                }

                // Read the noise std
                UKF_nh.param<double>("UKF/NCA/std_ax_", NCA_proc_noise.std_ax_, 0.0);
                UKF_nh.param<double>("UKF/NCA/std_ay_", NCA_proc_noise.std_ay_, 0.0);
                UKF_nh.param<double>("UKF/NCA/std_az_", NCA_proc_noise.std_az_, 0.0);
                UKF_nh.param<double>("UKF/NCA/std_px_", NCA_meas_noise.std_px_, 0.0);
                UKF_nh.param<double>("UKF/NCA/std_py_", NCA_meas_noise.std_py_, 0.0);
                UKF_nh.param<double>("UKF/NCA/std_pz_", NCA_meas_noise.std_pz_, 0.0);
            }            
            // car model,for autonomous landing
            else if (model_type == CAR)
            {

                // Read the noise std
                UKF_nh.param<double>("UKF/std_a_", CAR_proc_noise.std_a_, 0.0);
                UKF_nh.param<double>("UKF/std_yaw_dotdot_", CAR_proc_noise.std_yaw_dotdot_, 0.0);
                UKF_nh.param<double>("UKF/std_px_", CAR_meas_noise.std_px_, 0.0);
                UKF_nh.param<double>("UKF/std_py_", CAR_meas_noise.std_py_, 0.0);
                UKF_nh.param<double>("UKF/std_yaw_", CAR_meas_noise.std_yaw_, 0.0);

                n_x_ = 5;
                n_noise_ = 2;
                n_aug_ = n_x_ + n_noise_;
                n_z_ = 3;
                x_ = VectorXd(n_x_);
                z_ = VectorXd(n_z_);

                P_pre = MatrixXd(n_x_,n_x_);
                S_ = MatrixXd(n_z_,n_z_);
                T_ = MatrixXd(n_x_,n_z_);
                K_ = MatrixXd(n_x_,n_z_);
                P_ = MatrixXd(n_x_,n_x_);
                Q_ = MatrixXd(2,2);
                R_ = MatrixXd(n_z_,n_z_);

                P_ <<   CAR_meas_noise.std_px_*CAR_meas_noise.std_px_, 0, 0, 0, 0,
                        0, CAR_meas_noise.std_py_*CAR_meas_noise.std_py_, 0, 0, 0,
                        0, 0, 1, 0, 0,
                        0, 0, 0, CAR_meas_noise.std_yaw_*CAR_meas_noise.std_yaw_, 0,
                        0, 0, 0, 0, 1;
                Q_ <<   CAR_proc_noise.std_a_*CAR_proc_noise.std_a_, 0, 
                        0,CAR_proc_noise.std_yaw_dotdot_*CAR_proc_noise.std_yaw_dotdot_;
                               
                R_ <<   CAR_meas_noise.std_px_*CAR_meas_noise.std_px_, 0, 0,
                        0, CAR_meas_noise.std_py_*CAR_meas_noise.std_py_, 0,
                        0, 0,CAR_meas_noise.std_yaw_*CAR_meas_noise.std_yaw_;

                
                
                kamma_ = 3 - n_aug_;
                
                //set weights
                W_s = VectorXd(2*n_aug_+1);
                W_s(0) = kamma_/(kamma_+n_aug_);
                for(int i=1; i<2*n_aug_+1; ++i)
                {
                    W_s(i) = 1/(2*kamma_+2*n_aug_);
                }

                Xsig_pred_ = MatrixXd(n_x_,2*n_aug_+1);        
                


                cout<<"[UKF]: "<<"CAR model selected."<<endl;
            }
        }

        int model_type;

        enum Model
        {
            NCA,
            NCV,
            CAR,
        };

        //initially set to false, set to ture in first call of Run()
        bool is_initialized;

        int n_x_;       //系统状态维数
        int n_noise_;   //过程噪声维数
        int n_aug_;     //增广维数 = 系统状态维数 + 过程噪声维数
        int n_z_;       //测量状态维数

        VectorXd x_;   //系统状态变量
        VectorXd z_;   //测量值
        
        MatrixXd P_pre;     //预测状态误差协方差矩阵
        MatrixXd S_;        //预测测量误差协方差矩阵
        MatrixXd T_;        //状态与测量空间相关函数
        MatrixXd K_;        //卡尔曼增益
        MatrixXd P_;        //状态后验协方差矩阵
        MatrixXd Q_;        //过程噪声协方差矩阵
        MatrixXd R_;        //测量噪声协方差矩阵

        double kamma_;          //sigma点缩放系数
        VectorXd W_s;           //sigma点权重
        MatrixXd Xsig_pred_;    //sigma点预测矩阵

        // process noise standard deviation
        // NCA model
        struct
        {
            double std_ax_;
            double std_ay_;
            double std_az_;
        }NCA_proc_noise;        
        // car model
        struct
        {
            double std_a_;
            double std_yaw_dotdot_;
        }CAR_proc_noise;

        // measurement noise standard deviation
        // Vision measurement
        struct
        {
            double std_px_;
            double std_py_;
            double std_yaw_;
        }CAR_meas_noise;

        struct
        {
            double std_px_;
            double std_py_;
            double std_pz_;
        }NCA_meas_noise;

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

VectorXd UKF::Run(const prometheus_msgs::DetectionInfo& mesurement, double delta_t)
{
    if(!is_initialized)
    {
        if(model_type == NCA)
        {
            x_[0] = mesurement.position[0];
            x_[1] = mesurement.position[1];
            x_[2] = mesurement.position[2];
            x_[3] = 0.0;
            x_[4] = 0.0;
            x_[5] = 0.0;
            x_[6] = 0.0;
            x_[7] = 0.0;
            x_[8] = 0.0;

            P_ <<   NCA_meas_noise.std_px_*NCA_meas_noise.std_px_, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, NCA_meas_noise.std_py_*NCA_meas_noise.std_py_, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, NCA_meas_noise.std_pz_*NCA_meas_noise.std_pz_, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 1, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 1, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 1, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 1, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 1, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 1;
        }else if(model_type == CAR)
        {
            x_[0] = mesurement.position[0];
            x_[1] = mesurement.position[1];
            x_[2] = 0.0;
            x_[3] = mesurement.attitude[2];
            x_[4] = 0.0;
                    
            cout<<"[UKF]: "<<"CAR model is initialized."<<endl;
        }
        is_initialized = true;
    }    
    // 预测
    Prediction(delta_t);

    //UpdateVision(mesurement);

    return x_;
}

void UKF::Prediction(double delta_t)
{
    if (model_type == NCA)
    {
        // Augmentation     

    }
    // car model
    else if (model_type == CAR)
    {
        // 构造sigma点
        VectorXd x_aug = VectorXd(n_aug_);
        MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
        MatrixXd P_aug = MatrixXd(n_aug_,n_aug_);

        x_aug.head(5) = x_;
        x_aug[5] = 0.0;
        x_aug[6] = 0.0;

        P_aug.fill(0.0);
        P_aug.topLeftCorner(5,5) = P_;
        P_aug.bottomRightCorner(2,2) = Q_;

        cout<<"P_aug="<<endl<<P_aug<<endl<<endl;

        //llt()是Cholesky 分解
        //Cholesky分解是把一个对称正定的矩阵表示成一个下三角矩阵L和其转置的乘积的分解
        //即 P_aug = L*L^t;
        MatrixXd L = P_aug.llt().matrixL();
        Xsig_aug.fill(0.0);
        for(int i=0; i<2*n_aug_+1; ++i)
        {
            //第i列
            Xsig_aug.col(i) = x_aug;
        }
        //matrix.block<p,q>(i,j) : 从 (i,j) 开始，大小为 (p,q) 矩阵块
        Xsig_aug.block<7,7>(0,1) += sqrt(kamma_+n_aug_)*L;
        Xsig_aug.block<7,7>(0,n_aug_+1) -= sqrt(kamma_+n_aug_)*L;

        //时间更新（预测） - 利用系统方程对状态预测
        for(int i=0; i<2*n_aug_+1; ++i)
        {
            double p_x            = Xsig_aug(0,i);
            double p_y            = Xsig_aug(1,i);
            double v              = Xsig_aug(2,i);
            double yaw            = Xsig_aug(3,i);
            double yaw_dot        = Xsig_aug(4,i);
            double nu_a           = Xsig_aug(5,i);
            double nu_yaw_dotdot  = Xsig_aug(6,i);

            double px_pred, py_pred;
            if (fabs(yaw_dot) > 0.001)
            {
                px_pred = p_x + v/yaw_dot * (sin(yaw+yaw_dot*delta_t) - sin(yaw));
                py_pred = p_y - v/yaw_dot * (cos(yaw+yaw_dot*delta_t) - cos(yaw));
            }
            else 
            {
                px_pred = p_x + v*cos(yaw)*delta_t;
                py_pred = p_y + v*sin(yaw)*delta_t;
            }
            double v_pred = v;
            double yaw_pred = yaw + yaw_dot*delta_t;
            double yaw_dot_pred = yaw_dot;

            //add noise
            px_pred      += 0.5*nu_a*delta_t*delta_t * cos(yaw);
            py_pred      += 0.5*nu_a*delta_t*delta_t * sin(yaw);
            v_pred       += nu_a*delta_t;
            yaw_pred     += 0.5*nu_yaw_dotdot*delta_t*delta_t;
            yaw_dot_pred += nu_yaw_dotdot*delta_t;

            Xsig_pred_(0,i) = px_pred;
            Xsig_pred_(1,i) = py_pred;
            Xsig_pred_(2,i) = v_pred;
            Xsig_pred_(3,i) = yaw_pred;
            Xsig_pred_(4,i) = yaw_dot_pred; 
        }

        // 预测状态
        x_.fill(0.0);
        for (int i=0; i<2*n_aug_+1; ++i)
        {
            x_ += W_s(i)*Xsig_pred_.col(i);
        } 
        // 预测协方差矩阵
        P_.fill(0.0);
        for (int i=0; i<2*n_aug_+1; ++i)
        {
            // state difference
            VectorXd x_diff = Xsig_pred_.col(i) - x_;
            // angle normalization （偏航角）
            while (x_diff(3)>M_PI) x_diff(3)-=2.*M_PI;
            while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

            P_ = P_ + W_s(i)*x_diff*x_diff.transpose();
        }

        cout<<"P_="<<endl<<P_<<endl<<endl;
    }
}

void UKF::UpdateVision(const prometheus_msgs::DetectionInfo& mesurement)
{  
    if (model_type == NCA)
    {

    }
    // car model
    else if (model_type == CAR)
    {
        z_[0] = mesurement.position[0];
        z_[1] = mesurement.position[1];
        z_[2] = mesurement.attitude[2];

        MatrixXd Zsig = MatrixXd(n_z_, 2*n_aug_+1);
        MatrixXd z_pred = VectorXd(n_z_);
        MatrixXd T_ = MatrixXd(n_x_,n_z_);
        
        //观测预测值 - 观测方程
        Zsig.fill(0.0);
        for (int i=0; i<2*n_aug_+1; i++)
        {
            double p_x = Xsig_pred_(0,i);
            double p_y = Xsig_pred_(1,i);
            double yaw = Xsig_pred_(3,i);

            Zsig(0,i) = p_x;                      
            Zsig(1,i) = p_y;                               
            Zsig(2,i) = yaw;  
        }
        z_pred.fill(0.0);
        for (int i=0; i < 2*n_aug_+1; ++i) 
        {
            z_pred = z_pred + W_s(i) * Zsig.col(i);
        }  
        
        // Mean predicted measurement
        S_.fill(0.0);
        T_.fill(0.0);
        for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
            // residual
            VectorXd z_diff = Zsig.col(i) - z_pred;

            // angle normalization
            while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
            while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

            S_ = S_ + W_s(i) * z_diff * z_diff.transpose();
            VectorXd x_diff = Xsig_pred_.col(i) - x_;
            // angle normalization
            while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
            while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

            T_ = T_ + W_s(i) * x_diff * z_diff.transpose();
        }  
        // Innovation covariance matrix S

        S_ = S_ + R_;  // add measurement noise covariance matrix

        K_ = T_ * S_.inverse();   // Kalman gain K;
        VectorXd z_diff = z_ - z_pred;    // residual

        while(z_diff(1)>M_PI) z_diff(1) -= 2.*M_PI;
        while(z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI; // angle normalization

        //zheli duima ?
        x_ = x_ + K_*z_diff;
        P_ = P_ - K_*S_*K_.transpose();
    }
    
    
}


#endif
