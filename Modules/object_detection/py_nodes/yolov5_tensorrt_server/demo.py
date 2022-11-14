import torch
import time

# Model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # or yolov5n - yolov5x6, custom

# Images
img = 'samples/bus.jpg'  # or file, Path, PIL, OpenCV, numpy, list

T1 = time.time()
# Inference
results = model(img)
T2 =time.time()
print('infer time: %s ms' % ((T2 - T1)*1000))

img = 'samples/zidane.jpg'  # or file, Path, PIL, OpenCV, numpy, list

T1 = time.time()
# Inference
results = model(img)
T2 =time.time()
print('infer time: %s ms' % ((T2 - T1)*1000))

# Results
# results.show()  # or .show(), .save(), .crop(), .pandas(), etc.

