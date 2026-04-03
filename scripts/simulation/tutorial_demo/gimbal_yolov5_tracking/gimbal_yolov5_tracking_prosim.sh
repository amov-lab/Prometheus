#!/bin/bash
# 脚本描述：点击目标吊舱跟踪
# 1.启动从仿真环境获取视频的节点
# 2.启动推流服务
# 3.启动吊舱跟踪节点
# 4.启动点击跟踪视频识别、跟踪算法节点
# 5.启动推流节点
gnome-terminal --window  -e 'bash -c "roslaunch airsim_ros_pkgs airsim_node.launch output:=screen host:=$PX4_SIM_HOST_ADDR; exit; exec bash"' \
--tab -e 'bash -c "sleep 1.5s; /home/amov/SpireCV/scripts/common/startMediaServer.sh; exit; exec bash"' \
--tab -e 'bash -c "sleep 1.5s; source ~/spirecv-ros/devel/setup.bash; roslaunch spirecv_ros gimbal_server_prosim_gx40.launch; exit; exec bash"' \
--tab -e 'bash -c "sleep 1.5s; export LD_LIBRARY_PATH=/usr/local/cuda/lib64:/home/amov/TensorRT-8.6.1.6/lib:/opt/intel/openvino_2022/runtime/lib/intel64:$LD_LIBRARY_PATH; source ~/spirecv-ros/devel/setup.bash; roslaunch spirecv_ros prosim_detection_tracking_with_d435i.launch; exit; exec bash"' \
--tab -e 'bash -c "sleep 1.5s; export LD_LIBRARY_PATH=/usr/local/cuda/lib64:/home/amov/TensorRT-8.6.1.6/lib:/opt/intel/openvino_2022/runtime/lib/intel64:$LD_LIBRARY_PATH; source ~/spirecv-ros/devel/setup.bash; roslaunch spirecv_ros prosim_video_streaming_d435i.launch; exit; exec bash"' \
--tab -e 'bash -c "sleep 1.5s;roslaunch prometheus_demo gimbal_yolov5_tracking_prosim.launch output:=screen; exit; exec bash"' \





