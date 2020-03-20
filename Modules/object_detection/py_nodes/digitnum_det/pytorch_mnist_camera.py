#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import torch
from pytorch_mnist import LeNet
import torchvision as tv
import torchvision.transforms as transforms
import cv2
import numpy as np
import os
import yaml


# pip install opencv-python=='3.4.2.16'
model_name = os.path.dirname(os.path.abspath(__file__)) + '/model/net_012.pth'
# define use GPU or not
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
camera_matrix = np.zeros((3, 3), np.float32)
distortion_coefficients = np.zeros((5,), np.float32)
digitnum_det_len = 1.0

rmat_s = dict()
tvec_s = dict()


# load LeNet model trained on mnist dataset
def load_mnist_model():
    # the def of LeNet
    net = LeNet().to(device)
    # load paramet.
    checkpoint = torch.load(model_name)
    net.load_state_dict(checkpoint)

    return net


net = load_mnist_model()


def draw_approx_curve(img, approx):
    for i in range(len(approx) - 1):
        cv2.line(img, (approx[i,0,0], approx[i,0,1]), (approx[i+1,0,0], approx[i+1,0,1]), (0, 0, 255), 2)
    cv2.line(img, (approx[0,0,0], approx[0,0,1]), (approx[-1,0,0], approx[-1,0,1]), (0, 0, 255), 2)


def dis_points(p1, p2):
    p1 = np.squeeze(p1)
    p2 = np.squeeze(p2)
    dist = np.sqrt((p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]))
    return dist


def sort4points(points):
    lmin = 1e6
    lmax = 0
    imin = 0
    imax = 0
    for i in range(4):
        if points[i, 0] + points[i, 1] < lmin:
            lmin = points[i, 0] + points[i, 1]
            imin = i
        if points[i, 0] + points[i, 1] > lmax:
            lmax = points[i, 0] + points[i, 1]
            imax = i
    lx = 1e6
    ix = 0
    for i in range(4):
        if i != imin and i != imax:
            if points[i, 0] < lx:
                lx = points[i, 0]
                ix = i
    for i in range(4):
        if i != imin and i != imax and i != ix:
            iy = i
    newpts = np.zeros_like(points)
    newpts[0] = points[imin]
    newpts[1] = points[iy]
    newpts[2] = points[imax]
    newpts[3] = points[ix]
    return newpts


def box_extractor(img, net):
    edges = cv2.Canny(img, 100, 200)
    _, cnts, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    num = -1
    det_nums = []
    for cnt in cnts:
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            edges = np.zeros(4, np.float32)
            edges[0] = dis_points(approx[0], approx[1])
            edges[1] = dis_points(approx[1], approx[2])
            edges[2] = dis_points(approx[2], approx[3])
            edges[3] = dis_points(approx[3], approx[0])
            e_min = np.min(edges)
            e_avg = np.mean(edges)
            e_std = np.std(edges)

            if e_min > 10 and e_std / e_avg < 0.2:
                draw_approx_curve(img, approx)
                pts_res = np.float32([[0, 0], [28, 0], [28, 28], [0, 28]])
                approx = np.squeeze(approx).astype(np.float32)
                approx = sort4points(approx)
                M = cv2.getPerspectiveTransform(approx, pts_res)
                N = cv2.warpPerspective(img, M, (28, 28), cv2.INTER_NEAREST)
                N_gray = cv2.cvtColor(N, cv2.COLOR_BGR2GRAY).astype(np.float32)
                N_gray /= 255
                N_gray = 1 - N_gray
                cv2.imshow("N", N_gray)
                N_gray = np.expand_dims(np.expand_dims(N_gray, axis=0), axis=0)
                with torch.no_grad():
                    N_in = torch.from_numpy(N_gray)
                    N_in = N_in.to(device)
                    outputs = net(N_in)
                num = np.argmax(outputs.cpu().numpy())
                if num not in det_nums:
                    det_nums.append(num)
                    rmat_s[num] = np.eye(3).astype(np.float32)
                    tvec_s[num] = np.zeros((3,), np.float32)
                
                obj_pts = np.array([[-digitnum_det_len/2, -digitnum_det_len/2, 0],
                                    [digitnum_det_len/2, -digitnum_det_len/2, 0],
                                    [digitnum_det_len/2, digitnum_det_len/2, 0],
                                    [-digitnum_det_len/2, digitnum_det_len/2, 0]], np.float32)
                rvec = cv2.Rodrigues(rmat_s[num])[0]
                ret, rvec, tvec = cv2.solvePnP(obj_pts, approx, camera_matrix, distortion_coefficients)
                print(tvec)

    cv2.putText(img, 'nums: {}'.format(det_nums), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 4)

    return img, num


def image_callback(imgmsg):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(imgmsg, "bgr8")
    # processing
    frame, num = box_extractor(frame, net)
    # end
    cv2.imshow("color", frame)
    cv2.waitKey(10)


def num_det():
    pub = rospy.Publisher('/vision/num_det', String, queue_size=10)
    rospy.init_node('num_det', anonymous=True)
    rospy.Subscriber("/prometheus/camera/rgb/image_raw", Image, image_callback)
    
    # cap = cv2.VideoCapture(0)
    rospy.spin()
    
    # rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        
        state, frame = cap.read()
        frame, num = box_extractor(frame, net)
        cv2.imshow("capture", frame)
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break
        
        num_str = "num: %d" % num
        rospy.loginfo(num_str)
        pub.publish(num_str)
        # rate.sleep()

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    inparam = 'camera_param.yaml'
    yaml_config_fn = os.path.dirname(os.path.abspath(__file__)) + '/../../config/' + inparam
    print('Input config file: {}'.format(inparam))

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

    digitnum_det_len = yaml_config['digitnum_det_len']
    print(digitnum_det_len)

    try:
        num_det()
    except rospy.ROSInterruptException:
        pass
