#include <min_snap/min_snap_closeform.h>

namespace my_planner
{
    minsnapCloseform::minsnapCloseform(const vector<Eigen::Vector3d> &waypoints, double meanvel)
    {
        n_order = 7;
        wps = waypoints;
        n_seg = int(wps.size()) - 1;
        n_per_seg = n_order + 1;
        mean_vel = meanvel;
    }

    void minsnapCloseform::Init(const vector<Eigen::Vector3d> &waypoints, double meanvel)
    {
        n_order = 7;
        wps = waypoints;
        n_seg = int(wps.size()) - 1;
        n_per_seg = n_order + 1;
        mean_vel = meanvel;
    }

    // 0 means each segment time is one second.
    void minsnapCloseform::init_ts(int init_type)
    {
        const double dist_min = 2.0;
        ts = Eigen::VectorXd::Zero(n_seg);
        if (init_type)
        {
            Eigen::VectorXd dist(n_seg);
            double dist_sum = 0, t_sum = 0;
            for (int i = 0; i < n_seg; i++)
            {
                dist(i) = 0;
                for (int j = 0; j < 3; j++)
                {
                    dist(i) += pow(wps[i + 1](j) - wps[i](j), 2);
                }
                dist(i) = sqrt(dist(i));
                if ((dist(i)) < dist_min)
                {
                    dist(i) = sqrt(dist(i)) * 2;
                }
                dist_sum += dist(i);
            }
            dist(0) += 1;
            dist(n_seg - 1) += 1;
            dist_sum += 2;
            double T = dist_sum / mean_vel;
            for (int i = 0; i < n_seg - 1; i++)
            {
                ts(i) = dist(i) / dist_sum * T;
                t_sum += ts(i);
            }
            ts(n_seg - 1) = T - t_sum;
        }
        else
        {
            for (int i = 0; i < n_seg; i++)
            {
                ts(i) = 1;
            }
        }
        cout << "ts: " << ts.transpose() << endl;
    }

    int minsnapCloseform::fact(int n)
    {
        if (n < 0)
        {
            cout << "ERROR fact(" << n << ")" << endl;
            return 0;
        }
        else if (n == 0)
        {
            return 1;
        }
        else
        {
            return n * fact(n - 1);
        }
    }

    Eigen::VectorXd minsnapCloseform::calDecVel(const Eigen::VectorXd decvel)
    {
        Eigen::VectorXd temp(Eigen::VectorXd::Zero((n_seg + 1) * 4));
        for (int i = 0; i < (n_seg + 1) / 2; i++)
        {
            temp.segment(i * 8, 8) = decvel.segment((i * 2) * 8, 8);
        }
        if (n_seg % 2 != 1)
        {
            temp.tail(4) = decvel.tail(4);
        }

        return temp;
    }

    void minsnapCloseform::calQ()
    {
        Q = Eigen::MatrixXd::Zero(n_seg * (n_order + 1), n_seg * (n_order + 1));
        for (int k = 0; k < n_seg; k++)
        {
            Eigen::MatrixXd Q_k(Eigen::MatrixXd::Zero(n_order + 1, n_order + 1));
            for (int i = 5; i <= n_order + 1; i++)
            {
                for (int j = 5; j <= n_order + 1; j++)
                {
                    Q_k(i - 1, j - 1) = fact(i) / fact(i - 4) *
                                        fact(j) / fact(j - 4) /
                                        (i + j - 7) * pow(ts(k), i + j - 7);
                }
            }
            Q.block(k * (n_order + 1), k * (n_order + 1), n_order + 1, n_order + 1) = Q_k;
        }
    }

    void minsnapCloseform::calM()
    {
        M = Eigen::MatrixXd::Zero(n_seg * (n_order + 1), n_seg * (n_order + 1));
        for (int k = 0; k < n_seg; k++)
        {
            Eigen::MatrixXd M_k(Eigen::MatrixXd::Zero(n_order + 1, n_order + 1));
            M_k(0, 0) = 1;
            M_k(1, 1) = 1;
            M_k(2, 2) = 2;
            M_k(3, 3) = 6;
            for (int i = 0; i <= n_order; i++)
            {
                for (int j = 0; j <= 3; j++)
                {
                    if (i >= j)
                    {
                        M_k(j + 4, i) = fact(i) / fact(i - j) * pow(ts(k), i - j);
                    }
                }
            }
            M.block(k * (n_order + 1), k * (n_order + 1), n_order + 1, n_order + 1) = M_k;
        }
    }

    void minsnapCloseform::calCt()
    {
        int m = n_seg * (n_order + 1);
        int n = 4 * (n_seg + 1);
        Ct = Eigen::MatrixXd::Zero(m, n);
        for (int i = 0; i < n_seg; i++)
        {
            Ct.block(i * (n_order + 1), i * 4, n_order + 1, n_order + 1) =
                Eigen::MatrixXd::Identity(n_order + 1, n_order + 1);
        }

        Eigen::MatrixXd dF_Ct = Eigen::MatrixXd::Zero(m, 8 + (n_seg - 1));
        Eigen::MatrixXd dP_Ct = Eigen::MatrixXd::Zero(m, (n_seg - 1) * 3);
        dF_Ct.leftCols(4) = Ct.leftCols(4);
        for (int i = 0; i < n_seg - 1; i++)
        {
            dF_Ct.col(i + 4) = Ct.col(i * 4 + 4);
            dP_Ct.middleCols(i * 3, 3) = Ct.middleCols(i * 4 + 5, 3);
        }
        dF_Ct.rightCols(4) = Ct.rightCols(4);
        Ct << dF_Ct, dP_Ct;
    }

    std::pair<Eigen::VectorXd, Eigen::VectorXd> minsnapCloseform::MinSnapCloseFormServer(const Eigen::VectorXd &wp)
    {
        std::pair<Eigen::VectorXd, Eigen::MatrixXd> return_vel;
        Eigen::VectorXd poly_coef, dec_vel;
        Eigen::VectorXd dF(n_seg + 7), dP(3 * (n_seg - 1));
        Eigen::VectorXd start_cond(4), end_cond(4), d(4 * n_seg + 4);
        Eigen::MatrixXd R, R_pp, R_fp;
        start_cond << wp.head(1), 0, 0, 0;
        end_cond << wp.tail(1), 0, 0, 0;
        init_ts(1);

        calQ();
        calM();
        calCt();

        
        R = Ct.transpose() * M.inverse().transpose() * Q * M.inverse() * Ct;
        R_pp = R.bottomRightCorner(3 * (n_seg - 1), 3 * (n_seg - 1));
        R_fp = R.topRightCorner(n_seg + 7, 3 * (n_seg - 1));

        dF << start_cond, wp.segment(1, n_seg - 1), end_cond;
        dP = -R_pp.inverse() * R_fp.transpose() * dF;
        d << dF, dP;

        dec_vel = Ct * d;
        poly_coef = M.inverse() * dec_vel;
        
        return_vel.first = poly_coef;
        return_vel.second = dec_vel;
        return return_vel;
    }

    void minsnapCloseform::calMinsnap_polycoef()
    {
        Eigen::VectorXd wps_x(n_seg + 1), wps_y(n_seg + 1), wps_z(n_seg + 1);
        for (int i = 0; i < n_seg + 1; i++)
        {
            wps_x(i) = wps[i](0);
            wps_y(i) = wps[i](1);
            wps_z(i) = wps[i](2);
        }
        std::pair<Eigen::VectorXd, Eigen::VectorXd> return_vel;
        return_vel = MinSnapCloseFormServer(wps_x);
        poly_coef_x = return_vel.first;
        dec_vel_x = calDecVel(return_vel.second);
        return_vel = MinSnapCloseFormServer(wps_y);
        poly_coef_y = return_vel.first;
        dec_vel_y = calDecVel(return_vel.second);
        return_vel = MinSnapCloseFormServer(wps_z);
        poly_coef_z = return_vel.first;
        dec_vel_z = calDecVel(return_vel.second);
    }

    Eigen::MatrixXd minsnapCloseform::getPolyCoef()
    {
        Eigen::MatrixXd poly_coef(poly_coef_x.size(), 3);
        poly_coef << poly_coef_x, poly_coef_y, poly_coef_z;
        return poly_coef;
    }

    Eigen::MatrixXd minsnapCloseform::getDecVel()
    {
        Eigen::MatrixXd dec_vel(dec_vel_x.size(), 3);
        dec_vel << dec_vel_x, dec_vel_y, dec_vel_z;
        return dec_vel;
    }

    Eigen::VectorXd minsnapCloseform::getTime()
    {
        return ts;
    }

}