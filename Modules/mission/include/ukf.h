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
        UKF(int model_num):
            UKF_nh("~")
        {
            if(model_num == 1)
            {
                cout<<"[object_tracking]: "<<"Takeoff."<<endl;
            }
            UKF_nh.param<int>("UKF/model", model, 1.0);
            
            is_initialized = false;

            // NCA model
            if (model == 1)
            {
                n_x_ = 9;
                n_noise_ = 3;
                n_aug_ = n_x_ + n_noise_;
                x_ = VectorXd(n_x_);
                Xsig_pred_ = MatrixXd(n_x_,2*n_aug_+1);
                kamma_ = 3 - n_aug_;
                
                //set weights
                W_s = VectorXd(2*n_aug_+1);
                W_s(0) = kamma_/(kamma_+n_aug_);
                for(int i=0; i<2*n_aug_+1; ++i)
                {
                    W_s(i) = 1/(2*kamma_+2*n_aug_);
                }

                // Read the noise std
                UKF_nh.param<double>("UKF/std_ax_", std_ax_, 0.0);
                UKF_nh.param<double>("UKF/std_ay_", std_ay_, 0.0);
                UKF_nh.param<double>("UKF/std_az_", std_az_, 0.0);
                UKF_nh.param<double>("UKF/std_px_", std_px_, 0.0);
                UKF_nh.param<double>("UKF/std_py_", std_py_, 0.0);
                UKF_nh.param<double>("UKF/std_pz_", std_pz_, 0.0);
            }
            // car model,for autonomous landing
            else if (model == 2)
            {
                n_x_ = 5;
                n_noise_ = 2;
                n_aug_ = n_x_ + n_noise_;
                x_ = VectorXd(n_x_);
                Xsig_pred_ = MatrixXd(n_x_,2*n_aug_+1);
                kamma_ = 3 - n_aug_;
                
                //set weights
                W_s = VectorXd(2*n_aug_+1);
                W_s(0) = kamma_/(kamma_+n_aug_);
                for(int i=0; i<2*n_aug_+1; ++i)
                {
                    W_s(i) = 1/(2*kamma_+2*n_aug_);
                }

                // Read the noise std
                UKF_nh.param<double>("UKF/std_a_", std_a_, 0.0);
                UKF_nh.param<double>("UKF/std_yaw_dotdot_", std_yaw_dotdot_, 0.0);
                UKF_nh.param<double>("UKF/std_px_", std_px_, 0.0);
                UKF_nh.param<double>("UKF/std_py_", std_py_, 0.0);
                UKF_nh.param<double>("UKF/std_yaw_", std_yaw_, 0.0);
            }
        }

        // model
        int model;

        //initially set to false, set to ture in first call of Run()
        bool is_initialized;

        // state dimension
        int n_x_;

        // process noise dimension
        int n_noise_;

        // augmented state dimension = state dimension + process noise dimension
        int n_aug_;

        //系统状态变量
        VectorXd x_;
        // state covariance matrix
        MatrixXd P_;
        // predicted sigma points matrix
        VectorXd Xsig_pred_;

        //weights of sigma point
        VectorXd W_s;

        // sigma point spreading parameter
        double kamma_;

        // process noise standard deviation
        // car model
        double std_a_;
        double std_yaw_dotdot_;

        // NCA model
        double std_ax_;
        double std_ay_;
        double std_az_;

        // measurement noise standard deviation
        // Vision measurement
        double std_px_;
        double std_py_;
        double std_pz_;
        double std_yaw_;

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
        if(model == 1)
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

            P_ <<   1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 1, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 1, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 1, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 1, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 1, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 1, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 1, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 1;
        }else if(model == 2)
        {
            x_[0] = mesurement.position[0];
            x_[1] = mesurement.position[1];
            x_[2] = 0.0;
            x_[3] = mesurement.attitude[2];
            x_[4] = 0.0;

            P_ <<   1, 0, 0, 0, 0,
                    0, 1, 0, 0, 0,
                    0, 0, 1, 0, 0,
                    0, 0, 0, 1, 0,
                    0, 0, 0, 0, 1;
        }
        is_initialized = true;
    }
    
    // 预测
    Prediction(delta_t);

    UpdateVision(mesurement);

    return x_;
}

void UKF::Prediction(double delta_t)
{
    if (model == 1)
    {
        // Augmentation
        VectorXd x_aug = VectorXd(n_aug_);
        MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
        MatrixXd P_aug = MatrixXd(11,11);

        x_aug.head(9) = x_;
        x_aug[9] = 0.0;
        x_aug[10] = 0.0;
    }
    // car model
    else if (model == 2)
    {
        // Augmentation (dimension = state dimension + process noise dimension)
        VectorXd x_aug = VectorXd(n_aug_);
        MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
        MatrixXd P_aug = MatrixXd(n_aug_,n_aug_);

        x_aug.head(5) = x_;
        x_aug[5] = 0.0;
        x_aug[6] = 0.0;

        P_aug.fill(0.0);
        P_aug.topLeftCorner(5,5) = P_;
        P_aug(5,5) = std_a_*std_a_;
        P_aug(6,6) = std_yaw_dotdot_*std_yaw_dotdot_;

        // Create Augmented Sigma Points
        MatrixXd L = P_aug.llt().matrixL();
        Xsig_aug.fill(0.0);
        for(int i=0; i<2*n_aug_+1; ++i)
        {
            Xsig_aug.col(i) = x_aug;
        }
        Xsig_aug.block<7,7>(0,1) += sqrt(kamma_+n_aug_)*L;
        Xsig_aug.block<7,7>(0,n_aug_+1) -= sqrt(kamma_+n_aug_)*L;

        //Sigma Point Prediction
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

        // Predict Mean and Covariance
        x_.fill(0.0);
        for (int i=0; i<2*n_aug_+1; ++i)
        {
            x_ += W_s(i)*Xsig_pred_.col(i);
        } // predict state mean
        P_.fill(0.0);
        for (int i=0; i<2*n_aug_+1; ++i)
        {
            // state difference
            VectorXd x_diff = Xsig_pred_.col(i) - x_;
            // angle normalization
            while (x_diff(3)>M_PI) x_diff(3)-=2.*M_PI;
            while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

            P_ = P_ + W_s(i)*x_diff*x_diff.transpose();
        }
    }
}

void UKF::UpdateVision(const prometheus_msgs::DetectionInfo& mesurement)
{
    VectorXd z;
    if (model == 1)
    {
        z[0] = mesurement.position[0];
        z[1] = mesurement.position[1];
        z[2] = mesurement.position[2];
    }
    // car model
    else if (model == 2)
    {
        z[0] = mesurement.position[0];
        z[1] = mesurement.position[1];
        z[2] = mesurement.attitude[2];

        int n_z = 3;
        MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);
        MatrixXd z_pred = VectorXd(n_z);
        MatrixXd S = MatrixXd(n_z,n_z);
        MatrixXd Tc = MatrixXd(n_x_,n_z);
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
        S.fill(0.0);
        Tc.fill(0.0);
        for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
            // residual
            VectorXd z_diff = Zsig.col(i) - z_pred;

            // angle normalization
            while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
            while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

            S = S + W_s(i) * z_diff * z_diff.transpose();
            VectorXd x_diff = Xsig_pred_.col(i) - x_;
            // angle normalization
            while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
            while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

            Tc = Tc + W_s(i) * x_diff * z_diff.transpose();
        }  
        // Innovation covariance matrix S
        MatrixXd R = MatrixXd(n_z,n_z);
        R <<  std_px_*std_px_, 0, 0,
                0, std_py_*std_py_, 0,
                0, 0,std_yaw_*std_yaw_;
        S = S + R;  // add measurement noise covariance matrix

        MatrixXd K = Tc * S.inverse();   // Kalman gain K;
        VectorXd z_diff = z - z_pred;    // residual

        while(z_diff(1)>M_PI) z_diff(1) -= 2.*M_PI;
        while(z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI; // angle normalization

        x_ = x_ + K*z_diff;
        P_ = P_ - K*S*K.transpose();
    }
    
    
}


#endif
