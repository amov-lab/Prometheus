#roscore & sleep 5;
roslaunch min_snap real.launch & sleep 5;
roslaunch vicon_bridge vicon.launch & sleep 3;
roslaunch mavros px4.launch & sleep 3;
roslaunch ekf PX4_vicon.launch & sleep 3;
roslaunch px4ctrl run_ctrl.launch 
#rosbag record /vicon_imu_ekf_odom /debugPx4ctrl
