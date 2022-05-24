# import os
# prometheus_root_path = os.path.dirname(os.path.abspath(__file__)) + "/../../../../object_detection_yolov5tensorrt"
# import sys

# sys.path.insert(0, prometheus_root_path)

# print(os.listdir(prometheus_root_path))

import argparse
parser = argparse.ArgumentParser()
parser.add_argument(
    "--rtsp", type=str, default=0
)

args = parser.parse_args()

print(type(args.rtsp))