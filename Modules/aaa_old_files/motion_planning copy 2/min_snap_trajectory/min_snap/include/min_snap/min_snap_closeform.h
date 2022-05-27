#ifndef _MIN_SNAP_CLOSEFORM_H_
#define _MIN_SNAP_CLOSEFORM_H_

#include <Eigen/Eigen>
#include <iostream>

using std::vector;
using namespace std;

namespace my_planner
{
    class minsnapCloseform
    {
    private:
        vector<Eigen::Vector3d> wps;
        int n_order, n_seg, n_per_seg;
        double mean_vel;
        Eigen::VectorXd ts;
        Eigen::VectorXd poly_coef_x, poly_coef_y, poly_coef_z;
        Eigen::VectorXd dec_vel_x, dec_vel_y, dec_vel_z;
        Eigen::MatrixXd Q, M, Ct;

        int fact(int n);
        void init_ts(int init_type);
        std::pair<Eigen::VectorXd, Eigen::VectorXd> MinSnapCloseFormServer(const Eigen::VectorXd &wp);
        Eigen::VectorXd calDecVel(const Eigen::VectorXd decvel);
        void calQ();
        void calM();
        void calCt();

    public:
        minsnapCloseform(){};
        ~minsnapCloseform(){};
        minsnapCloseform(const vector<Eigen::Vector3d> &waypoints, double meanvel = 1.0);
        void Init(const vector<Eigen::Vector3d> &waypoints, double meanvel = 1.0);
        void calMinsnap_polycoef();
        Eigen::MatrixXd getPolyCoef();
        Eigen::MatrixXd getDecVel();
        Eigen::VectorXd getTime();
    };
}
#endif