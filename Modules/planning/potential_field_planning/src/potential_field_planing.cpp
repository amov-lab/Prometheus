//
// Created by taojiang on 2019/12/9.
//
//#include "potential_field_planning.h"
#include "../include/potential_field_planning.h"
#include <ros/ros.h>
namespace dyn_planner
{
    PotentialFiledPlanner::~PotentialFiledPlanner(){

    }

    bool PotentialFiledPlanner::compute_force(Eigen::Matrix<double, 3, 1> goal, Eigen::Matrix<double, 3, 1> current_odom){

        double dist;

//        edt_env_->evaluateEDTWithGrad(current_odom, -1.0, dist, dist_grad);
        Eigen::Matrix<double, 3, 1> push_force_edt;
        edt_env_->get_push_force(current_odom, 3.0, push_force_edt);
        double alpha_push_force = 1.0;
        push_force = alpha_push_force * push_force_edt;

        double cost;
        double dist0_;
        cost += dist < obs_distance ? pow(dist - dist0_, 2) : 0.0;


        double distance_to_goal = (goal - current_odom).norm();
        double alpha_goal_force = 1.0;
        if(distance_to_goal<3.0){
            attractive_force = alpha_goal_force * (goal-current_odom);
        }


        total_force = attractive_force + push_force;

        return true;
    }
}

int main(void){
    printf("hello world!\n");
    return 0;
}
