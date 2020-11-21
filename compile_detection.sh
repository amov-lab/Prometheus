# compile common message
catkin_make --source Modules/common/msgs --build build/msgs
# compile object_detection
catkin_make --source Modules/object_detection --build build/object_detection

if [ ! -f "Modules/object_detection_landing/CMakeLists.txt" ]; then
  # submodule object_detection_landing not exist, skip it
  echo -e "\e[32m[INFO] SUBMODULE\e[0m \e[33mobject_detection_landing\e[0m \e[32mNOT EXIST, Skip it!\e[0m"
else
  echo -e "\e[32m[INFO] COMPILE \e[33mobject_detection_landing\e[0m \e[32m...\e[0m"
  # compile object_detection_landing
  catkin_make --source Modules/object_detection_landing --build build/object_detection_landing
fi
