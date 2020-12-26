#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point, Quaternion
from cv_bridge import CvBridge
from std_msgs.msg import String
from std_msgs.msg import Bool
import numpy as np
import cv2
import os
import yaml
import math
import glob


camera_matrix = np.array([[274.4425695994789, 0.0, 422.808978794362], [0.0, 273.66842474114827, 407.3969079515446], [0.0, 0.0, 1.0]], np.float32)
distortion_coefficients = np.array([[0.01841136025813201], [-0.006751972660967855], [0.009935398363079766], [-0.008198696622455868]], np.float32)

rospy.init_node('fisheye_undistort', anonymous=True)
img_pub = rospy.Publisher('/camera/fisheye1/image_undistort', Image, queue_size = 10)


def image_callback(imgmsg):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(imgmsg, "bgr8")

    # processing
    DIM = (848, 800)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(camera_matrix, distortion_coefficients, np.eye(3), camera_matrix, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    # end

    """
    h, w = frame.shape[:2]
    img_resize = 360
    if h > w:
        h = int(float(h) / w * img_resize)
        w = img_resize
    else:
        w = int(float(w) / h * img_resize)
        h = img_resize
    frame = cv2.resize(frame, (w, h))
    """
    # cv2.imshow("undistorted_img", undistorted_img)
    # cv2.imshow("area", area)
    # cv2.waitKey(10)
    img_pub.publish(bridge.cv2_to_imgmsg(undistorted_img, "bgr8"))


def fisheye_undistort(topic_name):
    rospy.Subscriber(topic_name, Image, image_callback)
    rospy.spin()


def get_K_and_D(checkerboard, imgsPath):
    CHECKERBOARD = checkerboard
    subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
    calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW
    objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    _img_shape = None
    objpoints = []
    imgpoints = []
    images = glob.glob(imgsPath)
    for fname in images:
        img = cv2.imread(fname)
        if _img_shape == None:
            _img_shape = img.shape[:2]
        else:
            assert _img_shape == img.shape[:2], "All images must share the same size."

        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ret == True:
            objpoints.append(objp)
            cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria)
            imgpoints.append(corners)
    N_OK = len(objpoints)
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
    rms, _, _, _, _ = cv2.fisheye.calibrate(
        objpoints,
        imgpoints,
        gray.shape[::-1],
        K,
        D,
        rvecs,
        tvecs,
        calibration_flags,
        (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    )
    DIM = _img_shape[::-1]
    print("Found " + str(N_OK) + " valid images for calibration")
    print("DIM=" + str(_img_shape[::-1]))
    print("K=np.array(" + str(K.tolist()) + ")")
    print("D=np.array(" + str(D.tolist()) + ")")
    return DIM, K, D


if __name__ == '__main__':
    DIM, K, D = get_K_and_D((6, 9), os.path.dirname(os.path.abspath(__file__)) + '/checkerboard_imgs/*.png')

    subscriber = rospy.get_param('~camera_topic', '/camera/fisheye1/image_raw')
    camera_matrix = K
    distortion_coefficients = D

    try:
        fisheye_undistort(subscriber)
    except rospy.ROSInterruptException:
        pass
