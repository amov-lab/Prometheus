#include <ros/ros.h>
#include <Eigen/Eigen>

#include <prometheus_drl/agent_state.h>

#include "printf_utils.h"

prometheus_drl::agent_state agent_state;
Eigen::MatrixXd obstacle_matrix;
Eigen::MatrixXd agent_matrix;
Eigen::MatrixXd goal_matrix;
Eigen::MatrixXd nei_goal_matrix;

void state_cb(const prometheus_drl::agent_stateConstPtr& msg)
{
    agent_state = *msg;

    obstacle_matrix.resize(agent_state.matrix_size,agent_state.matrix_size);
    agent_matrix.resize(agent_state.matrix_size,agent_state.matrix_size);
    goal_matrix.resize(agent_state.matrix_size,agent_state.matrix_size);
    nei_goal_matrix.resize(agent_state.matrix_size,agent_state.matrix_size);

    for(int i = 0; i < agent_state.matrix_size; i++)
    {
        for(int j = 0; j < agent_state.matrix_size; j++)
        {
            obstacle_matrix(i,j) = agent_state.obstacle_matrix[i*agent_state.matrix_size+j];
            agent_matrix(i,j) = agent_state.agent_matrix[i*agent_state.matrix_size+j];
            goal_matrix(i,j) = agent_state.goal_matrix[i*agent_state.matrix_size+j];
            nei_goal_matrix(i,j) = agent_state.nei_goal_matrix[i*agent_state.matrix_size+j];
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
    ros::init(argc, argv, "read_agent_state");
    ros::NodeHandle nh("~");

    string agent_prefix;
    // 模型前缀 - 默认为 ugv，无人机则设置为 uav
    nh.param<string>("agent_prefix", agent_prefix, "/ugv");

    string agent_name = agent_prefix + std::to_string(1);

    ros::Subscriber agent_state_sub = nh.subscribe<prometheus_drl::agent_state>(agent_name + "/agent_state", 1, state_cb);

    ros::Timer printf_timer = nh.createTimer(ros::Duration(5.0), printf_cb);

    ros::spin();

    return 0;
}