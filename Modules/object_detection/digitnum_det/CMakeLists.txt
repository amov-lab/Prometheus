cmake_minimum_required(VERSION 2.8.3)
project(digitnum_det_ros)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

# add_message_files(DIRECTORY msg FILES Floats.msg HeaderString.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

# do not wildcard install files since the root folder of the package will contain a debian folder for releasing
catkin_install_python(PROGRAMS
  py/pytorch_mnist_camera.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/py
)
install(FILES
  py/README
  py/num_det.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/py
)


