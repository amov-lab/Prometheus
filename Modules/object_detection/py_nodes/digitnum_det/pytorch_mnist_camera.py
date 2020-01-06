#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import torch
from pytorch_mnist import LeNet
import torchvision as tv
import torchvision.transforms as transforms
import cv2
import numpy as np
import os

model_name = os.path.dirname(os.path.abspath(__file__)) + '/model/net_012.pth'
# define use GPU or not
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


# load LeNet model trained on mnist dataset
def load_mnist_model():
    # the def of LeNet
    net = LeNet().to(device)
    # load paramet.
    checkpoint = torch.load(model_name)
    net.load_state_dict(checkpoint)

    return net


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
    image, cnts, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    num = -1
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

                cv2.putText(img, 'num: %d' % num, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 4)
                pass

    return img, num


def num_det():
    pub = rospy.Publisher('/vision/num_det', String, queue_size=10)
    rospy.init_node('num_det', anonymous=True)
    
    cap = cv2.VideoCapture(0)
    net = load_mnist_model()
    
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
    try:
        num_det()
    except rospy.ROSInterruptException:
        pass
