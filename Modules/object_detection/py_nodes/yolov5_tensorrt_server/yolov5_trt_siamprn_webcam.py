"""
An example that uses TensorRT's Python api to make inferences.
"""
import sys
path = sys.path[0]
path = path + '/../../src/siam_rpn_lib/'
print(path)
sys.path.append(path)
import ctypes
import os
import shutil
import random
import threading
import time
import cv2
import socket
import numpy as np
from cuda import cudart
import tensorrt as trt
import torch
import torchvision
import math

from net import SiamRPNvot
from run_SiamRPN import SiamRPN_init, SiamRPN_track
from siam_utils import get_axis_aligned_bbox, cxy_wh_2_rect


ROS_SERVER_ON = True
NAMES_TXT = 'coco'

CONF_THRESH = 0.2
IOU_THRESHOLD = 0.4

global g_x, g_y, g_clicked, track_start
g_x = 0
g_y = 0
g_clicked = False
track_start = False


if ROS_SERVER_ON:
    socket_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    socket_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    socket_server.bind(('', 9091))
    socket_server.listen(5)
    print('Start listening on port 9091 ...')
    client_socket, client_address = socket_server.accept()
    print('Got my client')


def on_mouse(event, x, y, flags, param):
    global g_x, g_y, g_clicked, track_start
    if event == cv2.EVENT_LBUTTONDOWN:
        # print("x: {}, y: {}".format(x, y))
        g_x = x
        g_y = y
        g_clicked = True
    if event == cv2.EVENT_RBUTTONDOWN:
        track_start = False


WINDOW_NAME = 'tensorrt-yolov5'
cv2.namedWindow(WINDOW_NAME)  # flags = cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO
cv2.setMouseCallback(WINDOW_NAME, on_mouse)


def load_class_desc(dataset='coco'):
    """
    载入class_desc文件夹中的类别信息，txt文件的每一行代表一个类别
    :param dataset: str 'coco'
    :return: list ['cls1', 'cls2', ...]
    """
    desc_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'class_desc')
    desc_names = []
    for f in os.listdir(desc_dir):
        if f.endswith('.txt'):
            desc_names.append(os.path.splitext(f)[0])
    # 如果类别描述文件存在，则返回所有类别名称，否则会报错
    cls_names = []
    if dataset in desc_names:
        with open(os.path.join(desc_dir, dataset + '.txt')) as f:
            for line in f.readlines():
                if len(line.strip()) > 0:
                    cls_names.append(line.strip())
    else:
        raise NameError('{}.txt not exist in "class_desc"'.format(dataset))
    # 类别描述文件不能为空，否则会报错
    if len(cls_names) > 0:
        return cls_names
    else:
        raise RuntimeError('{}.txt is EMPTY'.format(dataset))


def get_img_path_batches(batch_size, img_dir):
    ret = []
    batch = []
    for root, dirs, files in os.walk(img_dir):
        for name in files:
            if len(batch) == batch_size:
                ret.append(batch)
                batch = []
            batch.append(os.path.join(root, name))
    if len(batch) > 0:
        ret.append(batch)
    return ret


def plot_one_box(x, img, color=None, label=None, line_thickness=None):
    """
    description: Plots one bounding box on image img,
                 this function comes from YoLov5 project.
    param:
        x:      a box likes [x1,y1,x2,y2]
        img:    a opencv image object
        color:  color to draw rectangle, such as (0,255,0)
        label:  str
        line_thickness: int
    return:
        no return

    """
    tl = (
            line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1
    )  # line/font thickness
    color = color or [0, 255, 0]  # [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(
            img,
            label,
            (c1[0], c1[1] - 2),
            0,
            tl / 3,
            [225, 255, 255],
            thickness=tf,
            lineType=cv2.LINE_AA,
        )


class YoLov5TRT(object):
    """
    description: A YOLOv5 class that warps TensorRT ops, preprocess and postprocess ops.
    """

    def __init__(self, engine_file_path):
        TRT_LOGGER = trt.Logger(trt.Logger.INFO)
        runtime = trt.Runtime(TRT_LOGGER)

        # Deserialize the engine from file
        with open(engine_file_path, "rb") as f:
            engine = runtime.deserialize_cuda_engine(f.read())
        context = engine.create_execution_context()
        # Create a Stream on this device,
        _, stream = cudart.cudaStreamCreate()
        host_inputs = []
        cuda_inputs = []
        host_outputs = []
        cuda_outputs = []
        bindings = []

        for binding in engine:
            print('bingding:', binding, engine.get_binding_shape(binding))
            size = trt.volume(engine.get_binding_shape(binding)) * engine.max_batch_size
            dtype = trt.nptype(engine.get_binding_dtype(binding))
            # Allocate host and device buffers
            host_mem = np.empty(size, dtype=dtype)
            _, cuda_mem = cudart.cudaMallocAsync(host_mem.nbytes, stream)
            # Append the device buffer to device bindings.
            bindings.append(int(cuda_mem))
            # Append to the appropriate list.
            if engine.binding_is_input(binding):
                self.input_w = engine.get_binding_shape(binding)[-1]
                self.input_h = engine.get_binding_shape(binding)[-2]
                host_inputs.append(host_mem)
                cuda_inputs.append(cuda_mem)
            else:
                host_outputs.append(host_mem)
                cuda_outputs.append(cuda_mem)

        # Store
        self.stream = stream
        self.context = context
        self.engine = engine
        self.host_inputs = host_inputs
        self.cuda_inputs = cuda_inputs
        self.host_outputs = host_outputs
        self.cuda_outputs = cuda_outputs
        self.bindings = bindings
        self.batch_size = engine.max_batch_size
        self.fps = 0
        self.tc = 0.

        print('loading model...........')
        self.siamnet = SiamRPNvot()
        self.siamnet.load_state_dict(torch.load(path + 'SiamRPNVOT.model'))
        self.siamnet.eval().cuda()
        z = torch.Tensor(1, 3, 127, 127)
        #self.siamnet.temple(z.cuda())
        x = torch.Tensor(1, 3, 271, 271)
        #self.siamnet(x.cuda())
        self.cls_names = load_class_desc(NAMES_TXT)
        self.frame_cnt = 0

    def infer(self, raw_image_generator):
        global g_x, g_y, g_clicked, track_start, g_label
        t0 = time.time()
        
        threading.Thread.__init__(self)
        # Restore
        stream = self.stream
        context = self.context
        engine = self.engine
        host_inputs = self.host_inputs
        cuda_inputs = self.cuda_inputs
        host_outputs = self.host_outputs
        cuda_outputs = self.cuda_outputs
        bindings = self.bindings
        
        start = time.time()
        sent_it = False
        self.frame_cnt += 1
        input_image_frame = raw_image_generator[0]
        if not track_start:
            # Do image preprocess
            batch_image_raw = []
            batch_origin_h = []
            batch_origin_w = []
            batch_input_image = np.empty(shape=[self.batch_size, 3, self.input_h, self.input_w])
            image_raw = raw_image_generator[0]
            input_image, image_raw, origin_h, origin_w = self.preprocess_image(image_raw)
            batch_image_raw.append(image_raw)
            batch_origin_h.append(origin_h)
            batch_origin_w.append(origin_w)
            np.copyto(batch_input_image[0], input_image)
            batch_input_image = np.ascontiguousarray(batch_input_image)

            # Copy input image to host buffer
            np.copyto(host_inputs[0], batch_input_image.ravel())
            
            # Transfer input data  to the GPU.
            cudart.cudaMemcpyAsync(cuda_inputs[0], host_inputs[0].ctypes.data, host_inputs[0].nbytes,
                                   cudart.cudaMemcpyKind.cudaMemcpyHostToDevice, stream)
            # Run inference.
            context.execute_async(batch_size=self.batch_size, bindings=bindings, stream_handle=stream)
            # Transfer predictions back from the GPU.
            cudart.cudaMemcpyAsync(host_outputs[0].ctypes.data, cuda_outputs[0], host_outputs[0].nbytes,
                                   cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost, stream)
            # Synchronize the stream
            cudart.cudaStreamSynchronize(stream)
            
            # Here we use the first row of output in that batch_size = 1
            output = host_outputs[0]
            # Do postprocess

            result_boxes, result_scores, result_classid = self.post_process(
                output[0: 6001], batch_origin_h[0], batch_origin_w[0]
            )

            box_clicked = []
            dist = 1e9
            # Draw rectangles and labels on the original image
            for j in range(len(result_boxes)):
                box = result_boxes[j]
                if box[0] < 0: box[0] = 0
                if box[1] < 0: box[1] = 0
                if box[2] < 0: box[2] = 0
                if box[3] < 0: box[3] = 0
                plot_one_box(
                    box,
                    batch_image_raw[0],
                    label="{}:{:.2f}".format(
                        categories[int(result_classid[j])], result_scores[j]
                    ),
                )
                if ROS_SERVER_ON:
                    # FrameID, 是否检测到目标(0/1,>1:num-of-objs), obj-order, 类别, x (0-1), y (0-1), w (0-1), h (0-1), 置信度, 0:detecting-mode
                    client_socket.send('{:08d},{:03d},{:03d},{:03d},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:04d},{:04d},0'.format(
                        self.frame_cnt,
                        len(result_boxes),
                        j,
                        int(result_classid[j]),
                        box[0] / image_raw.shape[1],
                        box[1] / image_raw.shape[0],
                        (box[2]-box[0]) / image_raw.shape[1],
                        (box[3]-box[1]) / image_raw.shape[0],
                        result_scores[j],
                        int((box[0]+box[2])/2),
                        int((box[1]+box[3])/2)).encode('utf-8'))
                    sent_it = True
                
                if g_clicked and (g_x>box[0] and g_y>box[1] and g_x<box[2] and g_y<box[3]):
                    cx = (box[0] + box[2]) / 2
                    cy = (box[1] + box[3]) / 2
                    d = math.sqrt((cx-g_x)**2 + (cy-g_y)**2)
                    if d < dist:
                        dist = d
                        box_clicked = box
                        g_label = int(result_classid[j])
            
            if g_clicked:
                if dist < 1e8:
                    cx = (box_clicked[0] + box_clicked[2]) / 2
                    cy = (box_clicked[1] + box_clicked[3]) / 2
                    print("x: {}, y: {}, box-cx: {}, box-cy: {}".format(g_x, g_y, cx, cy))
                    target_pos = np.array([int(cx), int(cy)])
                    target_sz = np.array([int(box_clicked[2]-box_clicked[0]), int(box_clicked[3]-box_clicked[1])])
                    self.state = SiamRPN_init(input_image_frame, target_pos, target_sz, self.siamnet)
                    track_start = True
                g_clicked = False

            if ROS_SERVER_ON and not sent_it:  # 发送无检测信息标志位
                client_socket.send('{:08d},000,000,000,0.000,0.000,0.000,0.000,0.000,0000,0000,0'.format(self.frame_cnt).encode('utf-8'))
                
        else:
            self.state = SiamRPN_track(self.state, input_image_frame)  # track
            res = cxy_wh_2_rect(self.state['target_pos'], self.state['target_sz'])
            res = [int(l) for l in res]
            image_raw = input_image_frame
            cv2.rectangle(image_raw, (res[0], res[1]), (res[0] + res[2], res[1] + res[3]), (0, 255, 255), 2)
            batch_image_raw = [image_raw]
            if ROS_SERVER_ON:
                # FrameID, 是否检测到目标(0/1), 类别, x (0-1), y (0-1), w (0-1), h (0-1), 置信度, 1:tracking-mode
                client_socket.send('{:08d},001,000,{:03d},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:04d},{:04d},1'.format(
                    self.frame_cnt,
                    int(g_label),
                    res[0] / image_raw.shape[1],
                    res[1] / image_raw.shape[0],
                    res[2] / image_raw.shape[1],
                    res[3] / image_raw.shape[0],
                    self.state['score'],
                    int(res[0]+res[2]/2),
                    int(res[1]+res[3]/2)).encode('utf-8'))
                sent_it = True

            if ROS_SERVER_ON and not sent_it:  # 发送无检测信息标志位
                client_socket.send('{:08d},000,000,000,0.000,0.000,0.000,0.000,0.000,0000,0000,1'.format(self.frame_cnt).encode('utf-8'))

        end = time.time()
        return batch_image_raw, end - start

    def destroy(self):
        # Remove any stream and cuda mem
        cudart.cudaStreamDestroy(self.stream)
        cudart.cudaFree(self.cuda_inputs[0])
        cudart.cudaFree(self.cuda_outputs[0])

    def get_raw_image(self, image_path_batch):
        """
        description: Read an image from image path
        """
        for img_path in image_path_batch:
            yield cv2.imread(img_path)

    def get_raw_image_zeros(self, image_path_batch=None):
        """
        description: Ready data for warmup
        """
        for _ in range(self.batch_size):
            yield np.zeros([self.input_h, self.input_w, 3], dtype=np.uint8)

    def preprocess_image(self, raw_bgr_image):
        """
        description: Convert BGR image to RGB,
                     resize and pad it to target size, normalize to [0,1],
                     transform to NCHW format.
        param:
            input_image_path: str, image path
        return:
            image:  the processed image
            image_raw: the original image
            h: original height
            w: original width
        """
        image_raw = raw_bgr_image
        
        h, w, c = image_raw.shape
        image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2RGB)
        # Calculate widht and height and paddings
        r_w = self.input_w / w
        r_h = self.input_h / h
        if r_h > r_w:
            tw = self.input_w
            th = int(r_w * h)
            tx1 = tx2 = 0
            ty1 = int((self.input_h - th) / 2)
            ty2 = self.input_h - th - ty1
        else:
            tw = int(r_h * w)
            th = self.input_h
            tx1 = int((self.input_w - tw) / 2)
            tx2 = self.input_w - tw - tx1
            ty1 = ty2 = 0
        # Resize the image with long side while maintaining ratio
        image = cv2.resize(image, (tw, th))
        # Pad the short side with (128,128,128)
        image = cv2.copyMakeBorder(
            image, ty1, ty2, tx1, tx2, cv2.BORDER_CONSTANT, None, (128, 128, 128)
        )
        image = image.astype(np.float32)
        # Normalize to [0,1]
        image /= 255.0
        # HWC to CHW format:
        image = np.transpose(image, [2, 0, 1])
        # CHW to NCHW format
        image = np.expand_dims(image, axis=0)
        # Convert the image to row-major order, also known as "C order":
        image = np.ascontiguousarray(image)
        return image, image_raw, h, w

    def xywh2xyxy(self, origin_h, origin_w, x):
        """
        description:    Convert nx4 boxes from [x, y, w, h] to [x1, y1, x2, y2] where xy1=top-left, xy2=bottom-right
        param:
            origin_h:   height of original image
            origin_w:   width of original image
            x:          A boxes numpy, each row is a box [center_x, center_y, w, h]
        return:
            y:          A boxes numpy, each row is a box [x1, y1, x2, y2]
        """
        y = np.zeros_like(x)
        r_w = self.input_w / origin_w
        r_h = self.input_h / origin_h
        if r_h > r_w:
            y[:, 0] = x[:, 0] - x[:, 2] / 2
            y[:, 2] = x[:, 0] + x[:, 2] / 2
            y[:, 1] = x[:, 1] - x[:, 3] / 2 - (self.input_h - r_w * origin_h) / 2
            y[:, 3] = x[:, 1] + x[:, 3] / 2 - (self.input_h - r_w * origin_h) / 2
            y /= r_w
        else:
            y[:, 0] = x[:, 0] - x[:, 2] / 2 - (self.input_w - r_h * origin_w) / 2
            y[:, 2] = x[:, 0] + x[:, 2] / 2 - (self.input_w - r_h * origin_w) / 2
            y[:, 1] = x[:, 1] - x[:, 3] / 2
            y[:, 3] = x[:, 1] + x[:, 3] / 2
            y /= r_h

        return y

    def post_process(self, output, origin_h, origin_w):
        """
        description: postprocess the prediction
        param:
            output:     A numpy likes [num_boxes,cx,cy,w,h,conf,cls_id, cx,cy,w,h,conf,cls_id, ...]
            origin_h:   height of original image
            origin_w:   width of original image
        return:
            result_boxes: finally boxes, a boxes numpy, each row is a box [x1, y1, x2, y2]
            result_scores: finally scores, a numpy, each element is the score correspoing to box
            result_classid: finally classid, a numpy, each element is the classid correspoing to box
        """
        # Get the num of boxes detected
        num = int(output[0])
        # Reshape to a two dimentional ndarray
        pred = np.reshape(output[1:], (-1, 6))[:num, :]
        # Do nms
        boxes = self.non_max_suppression(pred, origin_h, origin_w, conf_thres=CONF_THRESH, nms_thres=IOU_THRESHOLD)
        result_boxes = boxes[:, :4] if len(boxes) else np.array([])
        result_scores = boxes[:, 4] if len(boxes) else np.array([])
        result_classid = boxes[:, 5] if len(boxes) else np.array([])
        return result_boxes, result_scores, result_classid

    def bbox_iou(self, box1, box2, x1y1x2y2=True):
        """
        description: compute the IoU of two bounding boxes
        param:
            box1: A box coordinate (can be (x1, y1, x2, y2) or (x, y, w, h))
            box2: A box coordinate (can be (x1, y1, x2, y2) or (x, y, w, h))
            x1y1x2y2: select the coordinate format
        return:
            iou: computed iou
        """
        if not x1y1x2y2:
            # Transform from center and width to exact coordinates
            b1_x1, b1_x2 = box1[:, 0] - box1[:, 2] / 2, box1[:, 0] + box1[:, 2] / 2
            b1_y1, b1_y2 = box1[:, 1] - box1[:, 3] / 2, box1[:, 1] + box1[:, 3] / 2
            b2_x1, b2_x2 = box2[:, 0] - box2[:, 2] / 2, box2[:, 0] + box2[:, 2] / 2
            b2_y1, b2_y2 = box2[:, 1] - box2[:, 3] / 2, box2[:, 1] + box2[:, 3] / 2
        else:
            # Get the coordinates of bounding boxes
            b1_x1, b1_y1, b1_x2, b1_y2 = box1[:, 0], box1[:, 1], box1[:, 2], box1[:, 3]
            b2_x1, b2_y1, b2_x2, b2_y2 = box2[:, 0], box2[:, 1], box2[:, 2], box2[:, 3]

        # Get the coordinates of the intersection rectangle
        inter_rect_x1 = np.maximum(b1_x1, b2_x1)
        inter_rect_y1 = np.maximum(b1_y1, b2_y1)
        inter_rect_x2 = np.minimum(b1_x2, b2_x2)
        inter_rect_y2 = np.minimum(b1_y2, b2_y2)
        # Intersection area
        inter_area = np.clip(inter_rect_x2 - inter_rect_x1 + 1, 0, None) * \
                     np.clip(inter_rect_y2 - inter_rect_y1 + 1, 0, None)
        # Union Area
        b1_area = (b1_x2 - b1_x1 + 1) * (b1_y2 - b1_y1 + 1)
        b2_area = (b2_x2 - b2_x1 + 1) * (b2_y2 - b2_y1 + 1)

        iou = inter_area / (b1_area + b2_area - inter_area + 1e-16)

        return iou

    def non_max_suppression(self, prediction, origin_h, origin_w, conf_thres=0.5, nms_thres=0.4):
        """
        description: Removes detections with lower object confidence score than 'conf_thres' and performs
        Non-Maximum Suppression to further filter detections.
        param:
            prediction: detections, (x1, y1, x2, y2, conf, cls_id)
            origin_h: original image height
            origin_w: original image width
            conf_thres: a confidence threshold to filter detections
            nms_thres: a iou threshold to filter detections
        return:
            boxes: output after nms with the shape (x1, y1, x2, y2, conf, cls_id)
        """
        # Get the boxes that score > CONF_THRESH
        boxes = prediction[prediction[:, 4] >= conf_thres]
        # Trandform bbox from [center_x, center_y, w, h] to [x1, y1, x2, y2]
        boxes[:, :4] = self.xywh2xyxy(origin_h, origin_w, boxes[:, :4])
        # clip the coordinates
        boxes[:, 0] = np.clip(boxes[:, 0], 0, origin_w - 1)
        boxes[:, 2] = np.clip(boxes[:, 2], 0, origin_w - 1)
        boxes[:, 1] = np.clip(boxes[:, 1], 0, origin_h - 1)
        boxes[:, 3] = np.clip(boxes[:, 3], 0, origin_h - 1)
        # Object confidence
        confs = boxes[:, 4]
        # Sort by the confs
        boxes = boxes[np.argsort(-confs)]
        # Perform non-maximum suppression
        keep_boxes = []
        while boxes.shape[0]:
            large_overlap = self.bbox_iou(np.expand_dims(boxes[0, :4], 0), boxes[:, :4]) > nms_thres
            label_match = boxes[0, -1] == boxes[:, -1]
            # Indices of boxes with lower confidence scores, large IOUs and matching labels
            invalid = large_overlap & label_match
            keep_boxes += [boxes[0]]
            boxes = boxes[~invalid]
        boxes = np.stack(keep_boxes, 0) if len(keep_boxes) else np.array([])
        return boxes


class warmUpThread(threading.Thread):
    def __init__(self, yolov5_wrapper):
        threading.Thread.__init__(self)
        self.yolov5_wrapper = yolov5_wrapper

    def run(self):
        batch_image_raw, use_time = self.yolov5_wrapper.infer([np.zeros((640, 640, 3), np.uint8)])
        print('warm_up->{}, time->{:.2f}ms'.format(batch_image_raw[0].shape, use_time * 1000))



if __name__ == "__main__":
    # load custom plugin and engine
    PLUGIN_LIBRARY = "build/libmyplugins.so"
    engine_file_path = "build/yolov5s.engine"

    if len(sys.argv) > 1:
        engine_file_path = sys.argv[1]
    if len(sys.argv) > 2:
        PLUGIN_LIBRARY = sys.argv[2]

    ctypes.CDLL(PLUGIN_LIBRARY)
    cudart.cudaDeviceSynchronize()

    # load coco labels
    categories = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
            "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
            "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
            "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
            "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
            "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
            "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
            "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
            "hair drier", "toothbrush"]

    # a  YoLov5TRT instance
    yolov5_wrapper = YoLov5TRT(engine_file_path)

    # from https://github.com/ultralytics/yolov5/tree/master/inference/images
    # input_image_paths = ["T_20210424205620_000.jpg"]
    for i in range(10):
        # create a new thread to do warm_up
        thread1 = warmUpThread(yolov5_wrapper)
        thread1.start()
        thread1.join()

    # for input_image_path in input_image_paths:
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        # create a new thread to do inference
        ret, input_image = cap.read()
        # print(input_image.shape)
        batch_image_raw, use_time = yolov5_wrapper.infer([input_image])
        cv2.imshow(WINDOW_NAME, batch_image_raw[0])
        cv2.waitKey(10)

    # destroy the instance
    yolov5_wrapper.destroy()
