#!/usr/bin/env python
import sys
import os
path = sys.path[0]
path = path + '/../../src/siam_rpn_lib/'
print(path)
sys.path.append(path)
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
import yaml
from prometheus_msgs.msg import DetectionInfo, MultiDetectionInfo
import math


camera_matrix = np.zeros((3, 3), np.float32)
distortion_coefficients = np.zeros((5,), np.float32)
kcf_tracker_h = 1.0

rospy.init_node('siamrpn_tracker', anonymous=True)
pub = rospy.Publisher('/prometheus/object_detection/siamrpn_tracker', DetectionInfo, queue_size=10)


'''
def draw_circle(event, x, y, flags, param):
    global x1, y1, x2, y2, drawing, init, flag, iamge

    if init is False:
        #print(init)
        if event == cv2.EVENT_LBUTTONDOWN and flag == 2:
            if drawing is True:
                drawing = False
                x2, y2 = x, y
                init = True
                #flag = 1
                print(init)
                print([x1,y1,x2,y2])

        if event == cv2.EVENT_LBUTTONDOWN and flag == 1:

            drawing = True
            x1, y1 = x, y
            x2, y2 = -1, -1
            flag = 2
        if drawing is True:
            x2, y2 = x, y
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

    if event == cv2.EVENT_MBUTTONDOWN:
        flag = 1
        init = False
        x1, x2, y1, y2 = -1, -1, -1, -1
'''


def draw_circle(event, x, y, flags, param):
    global x1, y1, x2, y2, drawing, init, flag, iamge, start

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
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

    if event == cv2.EVENT_MBUTTONDOWN:
        flag = 1
        init = False
        x1, x2, y1, y2 = -1, -1, -1, -1


def callback(data):
    global image, getim
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    image = cv_image   
    getim = True


def showImage(subscriber, camera_matrix, kcf_tracker_h):
    global x1, y1, x2, y2, drawing, init, flag, image, getim, start

    flag=1
    init = False
    drawing = False
    getim = False
    start = False
    x1, x2, y1, y2 = -1, -1, -1, -1
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

    cv2.namedWindow('image')
    cv2.setMouseCallback('image', draw_circle)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if getim:
            getim = False
            ## ! 
            d_info = DetectionInfo()
            d_info.frame = 0
            ## ! 

            if start is False and init is True:
                target_pos = np.array([int((x1+x2)/2), int((y1+y2)/2)])
                target_sz = np.array([int(x2-x1), int(y2-y1)])
                state = SiamRPN_init(image, target_pos, target_sz, net)
                start = True
                flag_lose = False
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
            if flag_lose is True:
                cv2.putText(image, 'target lost', (20,40), cv2.FONT_HERSHEY_SIMPLEX , 1, (0,0,255), 2)
                ## ! 
                d_info.detected = False
            if drawing is True:
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            cx = int(image.shape[1]/2)
            cy = int(image.shape[0]/2)
            cv2.line(image,(cx-20, cy), (cx+20, cy), (255, 255, 255), 2)
            cv2.line(image,(cx, cy-20), (cx, cy+20), (255, 255, 255), 2)
            ## ! 
            pub.publish(d_info)
            cv2.imshow('image', image)
            cv2.waitKey(1)

        rate.sleep()

if __name__ == '__main__':
    subscriber = rospy.get_param('~camera_topic', '/prometheus/camera/rgb/image_raw')
    config = rospy.get_param('~camera_info', 'camera_param.yaml')

    yaml_config_fn = config
    print('Input config file: {}'.format(config))

    yaml_config = yaml.load(open(yaml_config_fn))

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

    showImage(subscriber, camera_matrix, kcf_tracker_h)
