gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash && roscore"  

sleep 3

gnome-terminal -x bash -c "cd ~/maplab_original_ws/ && source devel/setup.bash && rosrun rovioli tutorial_sineva_localization /home/colin/datasets/test_results_2.0/d1000-sineva/d1000-imu /home/colin/datasets/Localization_maps_maplab/d1000-maplab" &

gnome-terminal -x bash -c "cd ~/code_repos/Prometheus/Modules/map_building/ && source devel/setup.bash && rosrun sineva_stereo sineva_stereo_node"

sleep 10

gnome-terminal -x bash -c "cd ~/datasets/mapping_1h_1+10min+30min/ && rosbag play vinsdataset_mapping.bag /mynteye/left/image_mono:=/cam0/image_raw /mynteye/right/image_mono:=/cam1/image_raw /mynteye/imu/data_raw:=/imu0"


