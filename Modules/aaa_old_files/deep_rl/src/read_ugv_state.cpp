#include <ros/ros.h>
#include <Eigen/Eigen>

#include <prometheus_drl/ugv_state.h>

#include "printf_utils.h"

prometheus_drl::ugv_state ugv_state;
Eigen::MatrixXd obstacle_matrix;
Eigen::MatrixXd agent_matrix;
Eigen::MatrixXd goal_matrix;
Eigen::MatrixXd nei_goal_matrix;

void state_cb(const prometheus_drl::ugv_stateConstPtr& msg)
{
    ugv_state = *msg;

    obstacle_matrix.resize(ugv_state.matrix_size,ugv_state.matrix_size);
    agent_matrix.resize(ugv_state.matrix_size,ugv_state.matrix_size);
    goal_matrix.resize(ugv_state.matrix_size,ugv_state.matrix_size);
    nei_goal_matrix.resize(ugv_state.matrix_size,ugv_state.matrix_size);

    for(int i = 0; i < ugv_state.matrix_size; i++)
    {
        for(int j = 0; j < ugv_state.matrix_size; j++)
        {
            obstacle_matrix(i,j) = ugv_state.obstacle_matrix[i*ugv_state.matrix_size+j];
            agent_matrix(i,j) = ugv_state.agent_matrix[i*ugv_state.matrix_size+j];
            goal_matrix(i,j) = ugv_state.goal_matrix[i*ugv_state.matrix_size+j];
            nei_goal_matrix(i,j) = ugv_state.nei_goal_matrix[i*ugv_state.matrix_size+j];
        }
    }
}

void printf_cb(const ros::TimerEvent &e)
{
    cout << GREEN << "obstacle_matrix: " << TAIL <<endl;
    cout << GREEN <<  obstacle_matrix  << TAIL <<endl;
    cout << GREEN << "agent_matrix: " << TAIL <<endl;
    cout << GREEN <<  agent_matrix  << TAIL <<endl;
    cout << GREEN << "goal_matrix: " << TAIL <<endl;
    cout << GREEN <<  goal_matrix  << TAIL <<endl;
    cout << GREEN << "nei_goal_matrix: " << TAIL <<endl;
    cout << GREEN <<  nei_goal_matrix  << TAIL <<endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_odom_node");
    ros::NodeHandle nh("~");

    string ugv_name = "/ugv" + std::to_string(1);

    ros::Subscriber ugv_state_sub = nh.subscribe<prometheus_drl::ugv_state>(ugv_name + "/ugv_state", 1, state_cb);

    ros::Timer printf_timer = nh.createTimer(ros::Duration(5.0), printf_cb);

    ros::spin();

    return 0;
}