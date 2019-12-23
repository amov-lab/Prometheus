//
// Created by taojiang on 2019/12/9.
//

#ifndef SRC_POTENTIAL_FIELD_PLANNING_H
#define SRC_POTENTIAL_FIELD_PLANNING_H
#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include "plan_env/edt_environment.h"


using namespace std;

namespace dyn_planner{

class PotentialFiledPlanner{

private:
    EDTEnvironment::Ptr edt_env_;

public:
    Eigen::Matrix<double, 3, 1> push_force;
    Eigen::Matrix<double, 3, 1> attractive_force;

    double obs_distance;
    double att_distance;

    Eigen::Matrix<double, 3, 1> total_force;

    PotentialFiledPlanner()
    {
    }
    ~PotentialFiledPlanner();

    bool compute_force(Eigen::Matrix<double, 3, 1> goal, Eigen::Matrix<double, 3, 1> current_odom);

    /* get */

    /* set */


    /* check feasibility*/


    /* for evaluation */


    // typedef std::shared_ptr<PotentialFiledPlanner> Ptr;
};



}






#endif //SRC_POTENTIAL_FIELD_PLANNING_H
