###
 # @Author: your name
 # @Date: 2022-03-22 11:03:28
 # @LastEditTime: 2022-03-22 12:26:36
 # @LastEditors: your name
 # @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 # @FilePath: /Prometheus/compile_new.sh
### 
catkin_make --source Modules/common/msgs --build build/msgs
catkin_make --source Simulator/gazebo_simulator --build build/prometheus_gazebo
catkin_make --source Modules/uav_control --build build/uav_control
catkin_make --source Modules/matlab_bridge --build build/matlab_bridge
catkin_make --source Modules/tutorial_demo --build build/tutorial_demo
