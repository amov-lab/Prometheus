# download submodule: object_detection_yolov5openvino
git clone --branch x86 https://gitee.com/amovlab/prometheus-yolov5-tensorrt.git Modules/object_detection_yolov5tensorrt
cd Modules/object_detection_yolov5tensorrt
pip3 install -r requirements.txt -i http://mirrors.aliyun.com/pypi/simple/ --trusted-host mirrors.aliyun.com
mkdir build
cd build
cmake ..
make
cd .. && ./weights/download_weights.sh
python3 gen_wts.py
sudo build/yolov5 -s
