echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=:$(pwd)/src/simulation/models:~/gazebo_models" >> ~/.bashrc
echo "source $(pwd)/Firmware/Tools/setup_gazebo.bash $(pwd)/Firmware $(pwd)/Firmware/build/px4_sitl_default" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$(pwd)/Firmware" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$(pwd)/Firmware/Tools/sitl_gazebo" >> ~/.bashrc
source ~/.bashrc
echo "SUCCEED,  No need to repeat it."
