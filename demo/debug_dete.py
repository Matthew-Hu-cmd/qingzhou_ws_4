# -*- coding:utf-8 _*-
from pickle import REDUCE, TRUE
from PIL.Image import new
import cv2
import numpy as np
from ackermann_msgs.msg import AckermannDrive  # 引用阿克曼的消息类型
import rospy
from std_msgs.msg import Bool
import sys
import signal

h_min = 96
h_max = 127
s_min = 43
s_max = 157
v_min = 140
v_max = 213
old_angle = 0
angle = 0
check_count = 0
error_check = 0
max_error_check = 5
        
def gstreamer_pipeline(
        capture_width=1280,
        capture_height=720,
        display_width=320,
        display_height=240,
        framerate=120,
        flip_method=0,):
    return (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! "
            "appsink"
            % (
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
    )

cam = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

while True:
    while True:

        for i in range(5):
            success,img = cam.read()
        img = cv2.resize(img, (320, 240), interpolation=cv2.INTER_AREA)# 将图像压缩
        img = cv2.GaussianBlur(img, (5, 5), 0)
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # 将BGR图像转为HSV
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(imgHSV, lower, upper)  # 创建蒙版 指定颜色上下限 范围内颜色显示 否则过滤
        kernel_width = 4  # 调试得到的合适的膨胀腐蚀核大小
        kernel_height = 4  # 调试得到的合适的膨胀腐蚀核大小
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_width, kernel_height))
        mask = cv2.erode(mask, kernel)
        mask = cv2.dilate(mask, kernel)
        mask = cv2.dilate(mask, kernel)


        line_img = mask[120:220,0:220]
        # check_img = mask[120:,261:]
        light_img = mask[:100,:200 ]
        cv2.imshow("raw",img)
        cv2.imshow("line",line_img)
        # cv2.imshow("light",light_img)# 各个模块的视觉处理可视化
        # cv2.imshow('check',check_img)

        line_xy = np.column_stack(np.where(line_img == 255))
        # print(line_xy)
        # check_xy = np.column_stack(np.where(check_img == 255)) 
        line_x = np.mean(line_xy[:,0])
        line_y = np.mean(line_xy[:,1])
        # check_x = np.mean(check_xy[:,0])
        # check_y = np.mean(check_xy[:,1])
        # if np.isnan(check_x) and np.isnan(check_y):
        center_x = line_x + 120
        center_y = line_y #计算中点坐标


        if np.isnan(center_x) or np.isnan(center_y):
            angle = 0
        else:
            angle = np.degrees(np.arctan(float(160-center_y)/float(center_x - 26)))
            print(center_x, center_y, angle)
            if angle<0:
                angle = angle-10
            else:
                angle = angle+10
        # print(line_x,line_y)
        # print(angle)
        angle = 0.7 * angle + 0.3 * old_angle
        print(angle)
        old_angle = angle
        # ark_contrl.steering_angle = angle
        # ark_contrl.speed = 0.55
        # cmd_vel_pub.publish(ark_contrl)

        if (np.isnan(line_x) or np.isnan(line_y)):            
            error_check += 1
            if error_check > max_error_check:
                # ark_contrl.steering_angle = 0.0
                # ark_contrl.speed = 0.0
                # cmd_vel_pub.publish(ark_contrl)
                error_check = 0
                # done_pub.publish(True)
                print("done")
                break
                
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
        
    print("finish")

cam.release()