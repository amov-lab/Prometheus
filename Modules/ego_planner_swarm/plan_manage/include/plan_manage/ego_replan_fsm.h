#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <string>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <traj_utils/Bspline.h>
#include <traj_utils/MultiBsplines.h>
#include <geometry_msgs/PoseStamped.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <prometheus_msgs/ParamSettings.h>
#include <prometheus_msgs/UAVCommand.h>
#include <std_msgs/Bool.h>
#include <unordered_set>
#include <angles/angles.h> 
#include <ros/callback_queue.h>
#define PI 3.14159265358979323846
using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    //规划器状态
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP,
      SEQUENTIAL_START
    };
    // 目标点类型
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::DataDisp data_disp_;
    traj_utils::MultiBsplines multi_bspline_msgs_buf_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[10][3];

    int waypoint_num_, wp_id_,Last_Command_ID;
    double planning_horizen_, planning_horizen_time_,target_yaw;
    double emergency_time_;
    bool flag_realworld_experiment_;
    bool enable_fail_safe_;
    bool waypointCallback_status;
    bool goal_flag_;// 发布终点的标志位
    /* planning data */
    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    std::vector<Eigen::Vector3d> wps_;
    int current_wp_;
    Eigen::Vector3d end_wp;
    bool flag_escape_emergency_,has_last_bspline_;
    std_msgs::Bool stop_control_state;
    traj_utils::Bspline last_bspline_;
    /* ROS utils */
    ros::NodeHandle node_;
    ros::CallbackQueue callback_queue_;
    ros::Timer exec_timer_, safety_timer_,goal_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_, swarm_trajs_sub_, broadcast_bspline_sub_, trigger_sub_ ,param_sub_,stop_control_state_sub,sub_callback;
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, data_disp_pub_, swarm_trajs_pub_, broadcast_bspline_pub_,new_goal_pub_,yaw_local_pub_,ego_command_pub_;

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromCurrentTraj(const int trial_times = 1);
    void enu_yaw_pub(const float yaw_p,Eigen::Vector3d init_pt);
    void ego_command_stop_pub(bool flag_stop);
    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void readGivenWps();
    void planNextWaypoint(const Eigen::Vector3d next_wp);
    void getLocalTarget();

    /* ROS functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);
    void GoalCollisionCallback(const ros::TimerEvent &e);
    void waypointCallback(const geometry_msgs::PoseStampedPtr &msg);
    void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void odometryCallback_opt(const nav_msgs::OdometryConstPtr &msg);
    void swarmTrajsCallback(const traj_utils::MultiBsplinesPtr &msg);
    void BroadcastBsplineCallback(const traj_utils::BsplinePtr &msg);
    void paramCallback(const prometheus_msgs::ParamSettingsConstPtr &msg);
    bool checkCollision();
    void publishSwarmTrajs(bool startup_pub);
    void stop_control_state_cb(const std_msgs::Bool::ConstPtr &msg);
    std::vector<std::string> fsm_params_compare={ "flight_type","waypoint_num",
                                                  "emergency_time","planning_horizon","thresh_no_replan_meter",
                                                  "thresh_replan_time","waypoint0_x","waypoint0_y","waypoint0_z",
                                                  "waypoint1_x","waypoint1_y","waypoint1_z","waypoint2_x","waypoint2_y",
                                                  "waypoint2_z","waypoint3_x","waypoint3_y","waypoint3_z","fail_safe","realworld_experiment"
                                                }; 
    std::vector<std::string> fsm_params_compare_all;
    std::vector<int*> fsm_params_get_i;
    std::vector<bool*> fsm_params_get_b;
    std::vector<double*> fsm_params_get_d;
    inline void pre_fsm_params_compare(std::vector<std::string>& fsm_params_compare, std::vector<std::string>& fsm_params_compare_all)
    {
        // 确保 fsm_params_compare_all 的大小足够大
        fsm_params_compare_all.resize(fsm_params_compare.size());
        
        // 遍历 fsm_params_compare，将每个元素添加前缀，并赋值给 fsm_params_compare_all
        for (size_t i = 0; i < fsm_params_compare.size(); ++i) 
        {
            fsm_params_compare_all[i] = "/uav1_ego_planner_node/fsm/" + fsm_params_compare[i]; 
        }
    }
  public:
    EGOReplanFSM(/* args */)
    {
    }
    ~EGOReplanFSM()
    {
    }

    void init(ros::NodeHandle &nh);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif