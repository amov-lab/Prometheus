echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=:$(pwd)/gazebo_simulator/models:~/gazebo_models" >> ~/.bashrc
echo "source /home/colin/code_repos/some/Firmware/Tools/setup_gazebo.bash /home/colin/code_repos/some/Firmware /home/colin/code_repos/some/Firmware/build/px4_sitl_default" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/home/colin/code_repos/some/Firmware" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/home/colin/code_repos/some/Firmware/Tools/sitl_gazebo" >> ~/.bashrc
source ~/.bashrc
echo "SUCCEED,  No need to repeat it."
