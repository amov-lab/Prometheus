#ifndef __CONTROLLER_UTILS_H__
#define __CONTROLLER_UTILS_H__

#include <Eigen/Eigen>
#include <math.h>
#include <numeric>

using namespace std;

struct Desired_State
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    Eigen::Quaterniond q;
    double yaw;
};


struct Current_State
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Quaterniond q;
    double yaw;
};


struct Ctrl_Param_PID
{
    float quad_mass;
    float tilt_angle_max;
    float hov_percent;
    Eigen::Vector3d g;
    Eigen::Vector3f int_max;
    Eigen::Matrix3d Kp;
    Eigen::Matrix3d Kv;
    Eigen::Matrix3d Kvi;
    Eigen::Matrix3d Ka;
};

struct Ctrl_Param_UDE
{
    double T_ude;
    float tilt_angle_max;
    float quad_mass;
    float hov_percent;
    Eigen::Vector3d g;
    Eigen::Vector3f int_max;
    Eigen::Matrix3d Kp;
    Eigen::Matrix3d Kd;
};

struct Ctrl_Param_NE
{
    Eigen::Matrix3d Kp;
    Eigen::Matrix3d Kd;
    double T_ude;
    double T_ne;
    float tilt_angle_max;
    float quad_mass;
    float hov_percent;
    Eigen::Vector3d g;
    Eigen::Vector3f int_max;
};

// 追踪误差评估
class Tracking_Error_Evaluation
{
    public:
        Tracking_Error_Evaluation(){};

        std::vector<double> pos_error_vector;
        std::vector<double> vel_error_vector;

        double pos_error_mean{0};
        double vel_error_mean{0};

        void input_error(Eigen::Vector3d pos_error,Eigen::Vector3d vel_error)
        {
            double track_error_pos = pos_error.norm();
            double track_error_vel = vel_error.norm();
            pos_error_vector.insert(pos_error_vector.begin(), track_error_pos);
            vel_error_vector.insert(vel_error_vector.begin(), track_error_vel);
            
            if (pos_error_vector.size() > Slide_windouw)
            {
                pos_error_vector.pop_back();
            }
            if (vel_error_vector.size() > Slide_windouw)
            {
                vel_error_vector.pop_back();
            }
            
            vel_error_mean = std::accumulate(vel_error_vector.begin(), vel_error_vector.end(), 0.0) / pos_error_vector.size();
            pos_error_mean = std::accumulate(pos_error_vector.begin(), pos_error_vector.end(), 0.0) / pos_error_vector.size();
        }

    private:

        int Slide_windouw=100;
};


namespace controller_utils 
{



}
#endif