#!/usr/bin/env python3
import sys
import os
path = sys.path[0]
path = path + '/../../src/siam_rpn_lib/'
print(path)
sys.path.append(path)
sys.path.append("/../../../../devel/lib/python2.7/dist-packages")
sys.path.insert(0,'/opt/ros/' + os.environ['ROS_DISTRO'] + '/lib/python3/dist-packages/')
import rospy
import cv2
import torch
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
from geometry_msgs.msg import Pose
from net import SiamRPNvot
from run_SiamRPN import SiamRPN_init, SiamRPN_track
from utils import get_axis_aligned_bbox, cxy_wh_2_rect
from yaml import load, dump
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

from threading import Lock
from prometheus_msgs.msg import DetectionInfo, MultiDetectionInfo,WindowPosition
import math
import time


image_lock = Lock()

camera_matrix = np.zeros((3, 3), np.float32)
distortion_coefficients = np.zeros((5,), np.float32)
kcf_tracker_h = 1.0

rospy.init_node('siamrpn_tracker', anonymous=True)


"""
def draw_circle(event, x, y, flags, param):
    global x1, y1, x2, y2, drawing, init, flag, g_image, start

    if 1:
        if event == cv2.EVENT_LBUTTONDOWN and flag == 1:
            drawing = True
            x1, y1 = x, y
            x2, y2 = -1, -1
            flag = 2
            
            init = False    
            
        x2, y2 = x, y
        if event == cv2.EVENT_LBUTTONUP and flag == 2:
            w = x2-x1
            h = y2 -y1
            if w>0 and w*h>50:
                init = True   
                start = False   
                flag = 1
                drawing = False
                # print(init)
                # print([x1,y1,x2,y2])
            else:
                x1, x2, y1, y2 = -1, -1, -1, -1
        if drawing is True:
            x2, y2 = x, y
            cv2.rectangle(g_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

    if event == cv2.EVENT_MBUTTONDOWN:
        flag = 1
        init = False
        x1, x2, y1, y2 = -1, -1, -1, -1
"""
class MouseInfo:
    def __init__(self):
        self.flag = False
        self.down_xy = [0, 0]
        self.up_xy = [0, 0]
        self.cur_draw_xy = [0, 0]
        self.show_draw = False
        self.finish = False
        self.past_ts = time.time()
        self._r_double_button = False

    def __call__(self, event, x, y, flags, params):
        self.cur_draw_xy = [x, y]
        if event == cv2.EVENT_LBUTTONUP:
            if time.time() - self.past_ts < 0.5:
                self._r_double_button = True
                # print("Double Right Button")
            self.past_ts = time.time()

        if event == cv2.EVENT_LBUTTONDOWN and self.flag == False:
            self.down_xy = [x, y]
            self.up_xy = [0, 0]
            self.flag = not self.flag
            self.show_draw = True

        if event == cv2.EVENT_LBUTTONUP and self.flag == True:
            self.up_xy = [x, y]
            self.flag = not self.flag
            self.show_draw = False
            self.finish = True

    def r_double_event(self) -> bool:
        # 是否完成双击
        tmp = self._r_double_button
        self._r_double_button = False
        return tmp

    def finish_event(self) -> bool:
        # 是否完成框选
        tmp = self.finish
        self.finish = False
        return tmp

def callback(data):
    global g_image, getim
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    with image_lock:
        g_image = cv_image   
    getim = True

def winpos_callback(data):
    global x1, y1, x2, y2, init, start
    x1=data.origin_x
    y1=data.origin_y
    x2=x1+data.width
    y2=y1+data.height
    if data.mode==1:
        init = True
        start = False
    else:
        init = False
        start = False
    print(data)
    

def showImage(subscriber, camera_matrix, kcf_tracker_h, uav_id):
    global g_image, getim

    start = False
    getim = False
    flag_lose = False
    count_lose = 0

    print('loading model...........')
    net = SiamRPNvot()
    net.load_state_dict(torch.load(path + 'SiamRPNVOT.model'))
    net.eval().cuda()
    z = torch.Tensor(1, 3, 127, 127)
    net.temple(z.cuda())
    x = torch.Tensor(1, 3, 271, 271)
    net(x.cuda())
    print('ready for starting!')

    rospy.Subscriber(subscriber, Image, callback)
    rospy.Subscriber("/detection/bbox_draw",WindowPosition,winpos_callback)
    pub = rospy.Publisher("/uav" + str(uav_id) + '/prometheus/object_detection/siamrpn_tracker', DetectionInfo, queue_size=10)

    cv2.namedWindow('image')
    draw_bbox = MouseInfo()
    cv2.setMouseCallback('image', draw_bbox)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if getim:
            getim = False
            ## ! 
            d_info = DetectionInfo()
            d_info.frame = 0
            ## ! 
            with image_lock:
                image = g_image.copy()

            if start is False and draw_bbox.finish_event():
                mouse_bbox = [
                    min(draw_bbox.down_xy[0], draw_bbox.up_xy[0]),
                    min(draw_bbox.down_xy[1], draw_bbox.up_xy[1]),
                    max(draw_bbox.down_xy[0], draw_bbox.up_xy[0]),
                    max(draw_bbox.down_xy[1], draw_bbox.up_xy[1]),
                ]
                target_pos = np.array([(mouse_bbox[0] + mouse_bbox[2]) / 2, (mouse_bbox[1] + mouse_bbox[3]) / 2])
                target_sz = np.array([mouse_bbox[2] - mouse_bbox[0], mouse_bbox[3] - mouse_bbox[1]])
                if (target_sz[0]**2 + target_sz[1]**2) < 100:
                    continue
                state = SiamRPN_init(image, target_pos, target_sz, net)
                start = True
                flag_lose = False
                continue

            # 双击取消框选
            if draw_bbox.r_double_event():
                d_info.detected = False
                start = False
                continue

            if start is True:
                state = SiamRPN_track(state, image)  # track
                res = cxy_wh_2_rect(state['target_pos'], state['target_sz'])
                res = [int(l) for l in res]
                cv2.rectangle(image, (res[0], res[1]), (res[0] + res[2], res[1] + res[3]), (0, 255, 255), 2)

                ## ! 
                depth = kcf_tracker_h / state['target_sz'][1] * camera_matrix[1,1]
                cx = state['target_pos'][0] - image.shape[1] / 2
                cy = state['target_pos'][1] - image.shape[0] / 2
                d_info.position[0] = depth * cx / camera_matrix[0,0]
                d_info.position[1] = depth * cy / camera_matrix[1,1]
                d_info.position[2] = depth
                d_info.sight_angle[0] = cx / (image.shape[1] / 2) * math.atan((image.shape[1] / 2) / camera_matrix[0,0])
                d_info.sight_angle[1] = cy / (image.shape[0] / 2) * math.atan((image.shape[0] / 2) / camera_matrix[1,1])
                d_info.detected = True
                ## ! 

                cv2.putText(image, str(state['score']), (res[0] + res[2], res[1] + res[3]), cv2.FONT_HERSHEY_SIMPLEX , 0.5, (255,255,0), 1)

                if state['score'] < 0.5:
                    count_lose = count_lose + 1
                else:
                    count_lose = 0
                if count_lose > 4:
                    flag_lose = True

            cv2.putText(image, 'Double click to cancel the selection', (10,20), cv2.FONT_HERSHEY_SIMPLEX , 0.5, (0,0,255), 1)
            if flag_lose is True:
                cv2.putText(image, 'target lost', (20,40), cv2.FONT_HERSHEY_SIMPLEX , 1, (0,0,255), 2)
                ## ! 
                d_info.detected = False

            if draw_bbox.show_draw:
                cv2.rectangle(
                    image, draw_bbox.down_xy, draw_bbox.cur_draw_xy, (0, 255, 0), 2
                )

            cx = int(image.shape[1]/2)
            cy = int(image.shape[0]/2)
            cv2.line(image,(cx-20, cy), (cx+20, cy), (255, 255, 255), 2)
            cv2.line(image,(cx, cy-20), (cx, cy+20), (255, 255, 255), 2)
            ## ! 
            pub.publish(d_info)
            cv2.imshow('image', image)
            cv2.waitKey(10)

        rate.sleep()

if __name__ == '__main__':
    subscriber = rospy.get_param('~camera_topic', '/prometheus/sensor/monocular_front/image_raw')
    config = rospy.get_param('~camera_info', '/home/onx/Code/Prometheus/Simulator/gazebo_simulator/config/camera_config/camera_param_gazebo_monocular.yaml')
    uav_id = rospy.get_param('~uav_id', 1)

    yaml_config_fn = config
    print('Input config file: {}'.format(config))

    # yaml_config = yaml.load(open(yaml_config_fn))
    yaml_config = load(open(yaml_config_fn), Loader=Loader)

    camera_matrix[0,0] = yaml_config['fx']
    camera_matrix[1,1] = yaml_config['fy']
    camera_matrix[2,2] = 1
    camera_matrix[0,2] = yaml_config['x0']
    camera_matrix[1,2] = yaml_config['y0']
    print(camera_matrix)

    distortion_coefficients[0] = yaml_config['k1']
    distortion_coefficients[1] = yaml_config['k2']
    distortion_coefficients[2] = yaml_config['p1']
    distortion_coefficients[3] = yaml_config['p2']
    distortion_coefficients[4] = yaml_config['k3']
    print(distortion_coefficients)

    kcf_tracker_h = yaml_config['kcf_tracker_h']
    print(kcf_tracker_h)

    showImage(subscriber, camera_matrix, kcf_tracker_h, uav_id)
