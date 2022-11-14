cd Modules/object_detection/py_nodes/yolov5_tensorrt_server
pip3 install -r requirements.txt -i http://mirrors.aliyun.com/pypi/simple/ --trusted-host mirrors.aliyun.com
mkdir build
cd build
cmake ..
make
cd ..

python3 -c "
from utils.downloads import *;
attempt_download('build/yolov5s.pt')
"

python3 gen_wts.py -w build/yolov5s.pt -o build/yolov5s.wts

pip3 install cuda-python

cd build
sudo ./yolov5 -s yolov5s.wts yolov5s.engine s
