# download submodule: object_detection_yolov5openvino
git clone https://gitee.com/jario-jin/prometheus_detection_circlex.git Modules/object_detection_circlex
cd Modules/object_detection_circlex

sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler
sudo apt-get install --no-install-recommends libboost-all-dev
sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libopenblas-dev

cd spire_caffe
mkdir build
cd build
cmake ..
make -j6

cd ..
cd ..
cd ..

catkin_make --source Modules/object_detection_circlex --build build/object_detection_circlex
