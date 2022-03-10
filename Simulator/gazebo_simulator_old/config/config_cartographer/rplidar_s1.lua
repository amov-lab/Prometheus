-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
-- map_frame发布submaps的坐标系,也是位姿的父坐标系
--和odom最开始的时候是一个原点，但时间累计对产生累积误差
  map_frame = "map",
-- tracking_frame是SLAM算法的坐标系,如果使用IMU,则一般设为imu_link
  tracking_frame = "base_link",
-- published_frame是位姿的子坐标系，发布published_frame坐标系在map_frame坐标系的位姿
  published_frame = "base_link",
-- The frame between published_frame and map_frame to be used for publishing the (non-loop-closed) local SLAM result.
-- 发布odom_frame坐标系在map坐标系的位姿，发布published_frame在odom_frame的位姿
  odom_frame = "odom_cartographer",
  --如果启用，则local-slam估计的连续的姿态（不包括回环）将作为map_frame中 odom_frame发布
  provide_odom_frame = false,
  --publish_frame是否投影到XY平面
  publish_frame_projected_to_2d = true,
  use_odometry = false,
  --是否使用GPS数据
  use_nav_sat = false,
  --是否使用landmark
  use_landmarks = false,
  --sensor_msgs/LaserScan类型激光雷达的数量
  num_laser_scans = 1,
  --sensor_msgs/MultiEchoLaserScan类型激光雷达的数量
  num_multi_echo_laser_scans = 0,
  --每帧激光分成几份
  num_subdivisions_per_laser_scan = 1,
  --sensor_msgs/PointCloud2类型激光雷达的数量
  num_point_clouds = 0,
  --检测tf变换超时时间
  lookup_transform_timeout_sec = 0.2,
-- 发布submap的频率
  submap_publish_period_sec = 0.3,
-- 位姿发布频率,发布机器人位子及匹配点云频率
  pose_publish_period_sec = 0.01,
-- 轨迹发布频率
  trajectory_publish_period_sec = 0.2,
  --激光雷达采样因子
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 15.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65
return options
