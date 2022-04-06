#pragma once
#include <iostream>
#include <vector>
// ROS
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
// synchronize topic
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_srvs/Trigger.h>

//include opencv and eigen
#include <Eigen/Eigen>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>

namespace detect {

const int max_drone_num_ = 3;

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class DroneDetector
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  DroneDetector(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~DroneDetector();

  void test();
 private:
  void readParameters();

  // inline functions
  double getDist2(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2);
  double getDist2(const Eigen::Vector4d &p1, const Eigen::Vector4d &p2);
  Eigen::Vector4d depth2Pos(int u, int v, float depth);
  Eigen::Vector4d depth2Pos(const Eigen::Vector2i &pixel, float depth) ;
  Eigen::Vector2i pos2Depth(const Eigen::Vector4d &pose_in_camera) ;
  bool isInSensorRange(const Eigen::Vector2i &pixel);

  bool countPixel(int drone_id, Eigen::Vector2i &true_pixel, Eigen::Vector4d &true_pose_cam);
  void detect(int drone_id, Eigen::Vector2i &true_pixel);
  
  // subscribe callback function
  void rcvDepthColorCamPoseCallback(const sensor_msgs::ImageConstPtr& depth_img, \
                                const sensor_msgs::ImageConstPtr& color_img, \
                                const geometry_msgs::PoseStampedConstPtr& camera_pose);

  void rcvDepthCamPoseCallback(const sensor_msgs::ImageConstPtr& depth_img, \
                                const geometry_msgs::PoseStampedConstPtr& camera_pose);
  
  void rcvMyOdomCallback(const nav_msgs::Odometry& odom);
  void rcvDepthImgCallback(const sensor_msgs::ImageConstPtr& depth_img);


  void rcvDroneOdomCallbackBase(const nav_msgs::Odometry& odom, const int drone_id);

  void rcvDrone0OdomCallback(const nav_msgs::Odometry& odom);
  void rcvDrone1OdomCallback(const nav_msgs::Odometry& odom);
  void rcvDrone2OdomCallback(const nav_msgs::Odometry& odom);
  void rcvDroneXOdomCallback(const nav_msgs::Odometry& odom);
  
  //! ROS node handle.
  ros::NodeHandle& nh_;

  //! ROS topic subscriber.
  // depth, colordepth, camera_pos subscriber
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::PoseStamped> SyncPolicyDepthColorImagePose;
  typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyDepthColorImagePose>> SynchronizerDepthColorImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped> SyncPolicyDepthImagePose;
  typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyDepthImagePose>> SynchronizerDepthImagePose;
  
  // std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_img_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> colordepth_img_sub_;
  std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> camera_pos_sub_;

  SynchronizerDepthColorImagePose sync_depth_color_img_pose_;
  SynchronizerDepthImagePose sync_depth_img_pose_;
  // other drones subscriber
  ros::Subscriber drone0_odom_sub_, drone1_odom_sub_, drone2_odom_sub_, droneX_odom_sub_;
  std::string drone1_odom_topic_, drone2_odom_topic_;

  ros::Subscriber my_odom_sub_, depth_img_sub_;
  bool has_odom_;
  nav_msgs::Odometry my_odom_;
  // ROS topic publisher
  // new_depth_img: erase the detected drones
  // new_colordepth_img: for debug
  ros::Publisher new_depth_img_pub_;
  ros::Publisher debug_depth_img_pub_;

  // parameters
  //camera param
  int img_width_, img_height_;
  double fx_,fy_,cx_,cy_;

  double max_pose_error_;
  double max_pose_error2_;
  double drone_width_, drone_height_;
  double pixel_ratio_;
  int pixel_threshold_;

  // for debug
  bool debug_flag_;
  int debug_detect_result_[max_drone_num_];
  std::stringstream debug_img_text_[max_drone_num_];
  ros::Time debug_start_time_, debug_end_time_;

  ros::Publisher debug_info_pub_;
  ros::Publisher drone_pose_err_pub_[max_drone_num_];

  int my_id_;
  cv::Mat depth_img_, color_img_;

  Eigen::Matrix4d cam2body_;
  Eigen::Matrix4d cam2world_;
  Eigen::Quaterniond cam2world_quat_;
  Eigen::Vector4d my_pose_world_;
  Eigen::Quaterniond my_attitude_world_;
  Eigen::Vector4d my_last_pose_world_;
  ros::Time my_last_odom_stamp_ = ros::TIME_MAX;
  ros::Time my_last_camera_stamp_ = ros::TIME_MAX;

  Eigen::Matrix4d drone2world_[max_drone_num_];
  Eigen::Vector4d drone_pose_world_[max_drone_num_];
  Eigen::Quaterniond drone_attitude_world_[max_drone_num_];
  Eigen::Vector4d drone_pose_cam_[max_drone_num_];
  Eigen::Vector2i drone_ref_pixel_[max_drone_num_];

  std::vector<Eigen::Vector2i> hit_pixels_[max_drone_num_];
  int valid_pixel_cnt_[max_drone_num_];
  
  bool in_depth_[max_drone_num_] = {false};
  cv::Point searchbox_lu_[max_drone_num_], searchbox_rd_[max_drone_num_];
  cv::Point boundingbox_lu_[max_drone_num_], boundingbox_rd_[max_drone_num_];
};

} /* namespace */
