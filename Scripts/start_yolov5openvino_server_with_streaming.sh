source /opt/intel/openvino_2021/bin/setupvars.sh
cd Modules/object_detection_yolov5openvino
python3 yolo_openvino.py -m weights/yolov5s.xml -i cam -at yolov5 --rtmp_stream
