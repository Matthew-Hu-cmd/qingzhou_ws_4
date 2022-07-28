#!/usr/bin/envs python3.6
# -*- coding: utf-8 -*-

from tkinter import RIGHT
import cv2
from matplotlib.pyplot import contour, hist
import matplotlib.pyplot as plt
import rospy
import numpy as np
import time
import sys
import signal
from ackermann_msgs.msg import AckermannDrive  # 引用阿克曼的消息类型
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Int32

def quit():
    sys.exit()

signal.signal(signal.SIGINT,quit)
signal.signal(signal.SIGTERM,quit)

def gstreamer_pipeline(
        capture_width=640,
        capture_height=480,
        display_width=640,
        display_height=480,
        framerate=40,
        flip_method=0,
):
    return (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
    )

# np.set_printoptions(suppress=True, precision=4)
color_dict = {
    'Blue':[[75, 50, 20], [135, 200, 120]],
    'Green':[[46, 50, 160], [92, 255, 255]],
    'RandY':[[0, 30, 50], [35, 255, 255]]
}

# 提取ROI部分，根据传入的x， y参数确定提取的范围
def ROI_Img(img, x1, x2, y1, y2):
    poly = np.array([
        [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
    ])

    if len(img.shape) > 2:
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mask = np.zeros_like(gray_img)
    else:
        mask = np.zeros_like(img)
    cv2.fillPoly(mask, poly, 255)
    masked_img = cv2.bitwise_and(img, img, mask = mask)
    return masked_img

# 处理一下图片，开闭运算
def handleImg(img, kernel_size, low):
    gray_Img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
    ret, img_thresh = cv2.threshold(gray_Img, low, 255, cv2.THRESH_BINARY)
    # cv2.imshow('ac',img_thresh)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
    img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_CLOSE, kernel)
    # img_thresh = cv2.dilate(img_thresh, kernel)
    return img_thresh

# 根据传入的color，从color_dict中获取对应上下限提取对应的颜色
def getColorArea(img, color):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_img, lowerb=np.array(color_dict[color][0]), upperb=np.array(color_dict[color][1])) 
    color_img = cv2.bitwise_and(img, img, mask = mask)

    return color_img

# 获取角度的函数
def getShift(img):
    h,w = img.shape[:2]
    offset, shift = 0, 0
    # 得到处理后的蓝色图片
    b_img = getColorArea(img, 'Blue')
    # cv2.imshow("b", b_img)
    img_thresh = handleImg(b_img, 4, 10)

    # 获取车道线中点坐标，通过cv2.moments方法获取图心坐标
    histogram = np.sum(img_thresh[:, :], axis=0)
    # plt.plot(histogram)
    # plt.show()
    midpoint = int(histogram.shape[0] / 2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint + 80:])

    # 如果左右同时有点的话，基点为靠左的点，+50是为了拿到完整提取的车道线，以便提取图心
    if leftx_base != 0 and rightx_base != 0:
        final_img = ROI_Img(img_thresh, 0, leftx_base + 50, h - 70, h)
        # cv2.imshow('final', final_img)
    else:
        final_img = ROI_Img(img_thresh, 0, w, h - 70, h)
        # judge_base = np.argmax(histogram[int(midpoint/3):])
        # if judge_base != 0:
        #     print("Error point.")
        #     final_img = ROI_Img(color, 0, judge_base + 20, h - 50, h)
        #     # cv2.imshow('final', final_img)
        #     # offset = 70 - con * 6
        # else:
        #     print("Only blue.")
        #     final_img = ROI_Img(color, 0, rightx_base + midpoint + 40, h - 50, h)

    # out_img = cv2.resize(img_thresh, (320, 240), interpolation=cv2.INTER_AREA)
    # final_img = cv2.bitwise_xor(final_img, dst)
    # final_ing = cv2.bitwise_or(xor_img, final_img)
    contours, hierarchy = cv2.findContours(final_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(img, contours, 0, (0, 0, 255), 3)
    # cv2.imshow('final', final_img)
    # cv2.imshow("img", img)
    # cv2.imshow("img", img)
    cv2.waitKey(25)
    # contours, hierarchy = cv2.findContours(final_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        (x, y, w, h) = cv2.boundingRect(contours[0])
        print(w,h,x)
        if w > 70 or h > 36:
            print("Blue")
        # print(contours)
    #     # 获取图心，即提取部分的图像的车道线中点坐标
        M = cv2.moments(contours[0])
        cx = int(M['m10']/(M['m00']+float("1e-5")))
        cy = int(M['m01']/(M['m00']+float("1e-5")))
        cv2.circle(img,(cx,cy),2,(0,0,255),2)
        cv2.imshow('img',img)
        # 防止中间有时候处理有问题导致找不到中点，同时在出弯时可以用来判断
        if cx == 0 and cy == 0:
            shift = 0
        else:
            shift = np.degrees(np.arctan(float(160 - cx + offset)/float(cy - 26)))
    return shift

if __name__ =='__main__':
    # ros节点设置
    rospy.init_node("vision_control")

    # cmd_vel_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=1)    # 单开脚本的时候用这个
    cmd_vel_pub = rospy.Publisher('/vision_control', AckermannDrive, queue_size=1)
    locate_pub = rospy.Publisher("/qingzhou_locate", Int32, queue_size=1)

    max_x, min_x, min_y, aruco_id = 0, 0, 0, -1
    out_check, angle, now_angle = 0, 0, 0
    Kp, Kd = 1.0, 0.2
    now_err, last_err = 0, 0
    D_error=[0, 0, 0]
    green_count, RandY_count, color_flag = 0, 0, 0

    Video = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

    ark_contrl = AckermannDrive()     # 阿克曼消息

    rate = rospy.Rate(100)

    try:
        rospy.loginfo("line detector is started...")
        while not rospy.is_shutdown():
            # 清除缓存
            for i in range(10):
                ret = Video.grab()

            if aruco_id == -1:
                print("------------------------------Line------------------------------")
                while Video.isOpened():
                    rate.sleep()
                    ret,img = Video.read()
                    if ret is False:
                        print('img is none')
                        break
                    else:
                        # 出弯判断
                        if out_check > 8:
                            ark_contrl.speed = 0
                            ark_contrl.steering_angle = 0
                            cmd_vel_pub.publish(ark_contrl)
                            print('out, stop')
                            out_check, angle, now_angle, now_err, last_err = 0, 0, 0, 0, 0
                            D_error = [0, 0, 0]

                        img = cv2.resize(img, (320, 240), interpolation=cv2.INTER_AREA)
                        # img = cv2.GaussianBlur(img, (5, 5), 0)
                        pre_Img = ROI_Img(img, 20, img.shape[1] - 20, img.shape[0] - 75, img.shape[0])
                        # cv2.imshow("pre", pre_Img)
                        shift = getShift(pre_Img)

                        if shift:     # PID控制角度
                            out_check = 0
                            last_err = now_err
                            
                            now_err = 0 - shift
                            D_error = D_error[1:]
                            D_error.append(now_err - last_err)
                            now_angle = Kp * now_err + Kd * (D_error[2] * 0.6 + D_error[1] * 0.3 + D_error[0] * 0.1)
                            # now_angle = Kp * now_err + Kd * (now_err - last_err)
                            if -now_angle > 30:
                                now_angle = -30
                        else:
                            out_check += 1
                    
                        angle = -now_angle
                        if cv2.waitKey(25) == ord('q'):
                            break
                        # ark_contrl.speed = 0
                        # ark_contrl.steering_angle = angle
                        print(f"angle:{angle}")
                        # cmd_vel_pub.publish(ark_contrl)

    finally:
        print('End')
        Video.release() 

