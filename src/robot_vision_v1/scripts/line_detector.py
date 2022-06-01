#!/usr/bin/env python
# -*- coding:utf-8 _*-
# from typing import Counter
# from scripts.trafficlight_detector2 import Video
import cv2
import numpy as np
# from imutils import paths
import argparse
from _02GStreamer import *
import time
import rospy
# from std_msgs.msg import String
from std_msgs.msg import String, Float32

# # 采集设备（摄像头）信息

flag = 0
pianyi_befor = 0
# WIDTH=3280
# HEIGHT=2464
# 视野宽(cm)
FOV_w=105

Video = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

# 提取ROI
def region_of_interest(r_image):
    h = r_image.shape[0]
    w = r_image.shape[1]
    gray = cv2.cvtColor(r_image, cv2.COLOR_BGR2GRAY)

    poly = np.array([
        [(0, h-100), (w, h-100), (w, h), (0, h)]
    ])
    mask = np.zeros_like(gray)
    cv2.fillPoly(mask, poly, 255)   #fillPoly（） ： 多个多边形填充函数原型——cv2.fillPoly( image , [ 多边形顶点array1, 多边形顶点array2, … ] , RGB color)
    masked_image = cv2.bitwise_and(r_image,r_image, mask=mask)  #cv2.bitwise_and()是对二进制数据进行“与”操作，即对图像（灰度图像或彩色图像均可）每个像素值进行二进制“与”操作，1&1=1，1&0=0，0&1=0，0&0=0
    return masked_image

# 色彩分割
def color_seperate(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   #对目标图像进行色彩空间转换
    lower_hsv = np.array([80, 60, 60])          #设定蓝色下限
    upper_hsv = np.array([150, 180, 180])        #设定蓝色上限
    mask = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)  #依据设定的上下限对目标图像进行二值化转换
    dst = cv2.bitwise_and(image, image, mask=mask)    #将二值化图像与原图进行“与”操作；实际是提取前两个frame 的“与”结果，然后输出mask 为1的部分
                                                 #注意：括号中要写mask=xxx
    #dst[dst==0]=255
    #cv2.imshow('result', mask)                     #输出
    return dst

# 色彩分割
def color_seperate_1(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   #对目标图像进行色彩空间转换
    lower_hsv = np.array([156, 43, 46])          #设定红色下限
    upper_hsv = np.array([180, 255, 255])        #设定红色上限
    mask = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)  #依据设定的上下限对目标图像进行二值化转换
    dst = cv2.bitwise_and(image, image, mask=mask)    #将二值化图像与原图进行“与”操作；实际是提取前两个frame 的“与”结果，然后输出mask 为1的部分
                                                 #注意：括号中要写mask=xxx
    #dst[dst==0]=255
    cv2.imshow('result', mask)                     #输出
    return dst

# 检测偏移的函数
def pianyi_detect(img):
    pianyi=0
    pianyi_text=''
    #读取图像
    (img_w, img_h) = img.shape[:2]
    lane_img=img.copy()
    cropped_img=region_of_interest(lane_img)
    # cv2.imshow("cpp",cropped_img)
    cropped_img_1=cropped_img.copy()

    cropped_img = color_seperate(cropped_img)

    gray_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
    #gray_img = cv2.GaussianBlur(gray_img, (5, 5), 0, 0, cv2.BORDER_DEFAULT)
    # cv2.imshow("grw",gray_img)
    ret, img_thresh = cv2.threshold(gray_img,10, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
    img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_CLOSE, kernel)
    contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.imshow("imm",img_thresh)
    if(255 not in img_thresh[420] and 255 not in img_thresh[400]):
        print("out")
        return 999
    # print(cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE))
    #1.如果检测到蓝色线或者蓝白线，进行判断
    if (len(contours) > 0):
        con_num = len(contours)
        contour1 = []
        for c1 in range(len(contours)):
            for c2 in range(len(contours[c1])):
                contour1.append(contours[c1][c2])
                # cv2.imshow('c', c1)
        contour1 = np.array(contour1)

        # res = cv2.drawContours(img, contour1, -1, (0, 0, 255), 1)
        (x, y, w, h) = cv2.boundingRect(contour1)

        # 1.1 同时检测到蓝白线和蓝线，删选出蓝白线，计算位置
        if w>img_h/4:
            mask=np.zeros_like(gray_img)
            cv2.rectangle(mask, (x, y), (x + 250, y + h), (255, 255, 255), cv2.FILLED)
            # cv2.imshow('ccccc',mask)
            temp_img=cv2.bitwise_and(img_thresh, img_thresh, mask=mask)
            # print(img_thresh)
            # a = 0
            # b = 0
            # for i in img_thresh:

            #     # print(i)
            #     for j in i:
            #         if(j>0):
            #             print(a,b)
            #         b= b+1
            #     a = a+1
            # time.sleep(5)
            # # if(img_thresh[])

            contours1, hierarchy = cv2.findContours(temp_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if (len(contours1) > 0):
                contour2 = []
                for c1 in range(len(contours1)):
                    for c2 in range(len(contours1[c1])):
                        contour2.append(contours1[c1][c2])
                contour2 = np.array(contour2)
                # cv2.imshow('c',contour2)
                # print(contour2)
                # res = cv2.drawContours(img, contour1, -1, (0, 0, 255), 1)

                (x1, y1, w1, h1) = cv2.boundingRect(contour2)
                cv2.rectangle(img, (x1, y1), (x1 + w1, img_h), (250, 250, 255), 3)

                pianyi=((x1+w1/2)-(img_h/2))*FOV_w/img_h
                if pianyi>0:
                    #print('右偏')
                    pianyi_text='right'
                elif pianyi<0:
                    #print('左偏')
                    pianyi_text='left'
                else:
                    # print('左偏')
                    pianyi_text = 'stright'
                #print(pianyi)

        # 1.2 判断是只检测到一条线，需要判断是蓝白线还是蓝线
        else:
            # 如果是蓝白线
            if con_num>1:
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 3)
                pianyi = ((x + w / 2) - (img_h / 2)) * FOV_w / img_h
                if pianyi > 0:
                    #print('右偏')
                    pianyi_text='right'
                elif pianyi<0:
                    #print('左偏')
                    pianyi_text='left'
                else:
                    # print('左偏')
                    pianyi_text = 'stright'
                #print(pianyi)
            # 蓝线
            else:
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3)
                pianyi = 40-((x + w / 2) - (img_h / 2)) * FOV_w / img_h
                pianyi_text='left'
                #print('左偏移')
                #print(pianyi)

    # 2.未检测到蓝线或者蓝白线，就检测红线
    else:
        cropped_img_1 = color_seperate_1(cropped_img_1)
        gray_img = cv2.cvtColor(cropped_img_1, cv2.COLOR_BGR2GRAY)
        # gray_img = cv2.GaussianBlur(gray_img, (5, 5), 0, 0, cv2.BORDER_DEFAULT)
        ret, img_thresh = cv2.threshold(gray_img, 10, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
        img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_CLOSE, kernel)
        contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if (len(contours) > 0):
            contour2 = []
            for c1 in range(len(contours)):
                for c2 in range(len(contours[c1])):
                    contour2.append(contours[c1][c2])
            contour2 = np.array(contour2)
            # res = cv2.drawContours(img, contour1, -1, (0, 0, 255), 1)
            (x2, y2, w2, h2) = cv2.boundingRect(contour2)
            cv2.rectangle(img, (x2, y2), (x2 + w2, img_h), (255, 0, 255), 3)

            pianyi = 40 - ((x2 + w2 / 2) - (img_h / 2)) * FOV_w / img_h
            pianyi_text='right'
            #print('右偏移')
            #print(pianyi)

    # cv2.imshow('res1', img)
    # key = cv2.waitKey(5)
    # if key != -1:
    # #     # rospy.loginfo(" try to exit")
    #     pass
    # # 返回偏移量（cm）和偏移方向（左偏或者右偏）
    global pianyi_befor
    pianyi_now = abs(pianyi)
    # print("first {}".format(pianyi_now))
    if(abs(pianyi_now - pianyi_befor) > 10):
        pianyi_now = pianyi_befor
    # print("second {}".format(pianyi_now))   
    if pianyi_text == 'right' :
        pianyi_now = 0 - pianyi_now
        # print("third {}".format(pianyi_now))   
    elif  pianyi_text == 'left' :
        pianyi_now =  pianyi_now
        # print("third {}".format(pianyi_now))   
    
        
    pianyi_befor = abs(pianyi_now)
    return pianyi_now


if __name__ == '__main__':
    # 输入图像设置
    # arg = argparse.ArgumentParser()
    # arg.add_argument("-i", "--images", required=True,
    #                  help="path to input directory of images")
    # args = vars(arg.parse_args())
    
    # cap.release() 
    # cv.destroyAllWindows() 
    # 循环读取文件夹下的图像进行处理
    # for imagePath in paths.list_images('/home/cquer/Documents/python/opencv/Image'): #args["images"]
    #     img = cv2.imread(imagePath)
    #     result = pianyi_detect(img)
    #     print(result)
    #     input('input any key to continue')
    try:
        rospy.loginfo("line_detector node is started...")
        while not rospy.is_shutdown():
        # 初始化ros节点
            rospy.init_node("line_detector")
            ret, img = Video.read()
            if(img is None):
                print("img is none")
            else:
                pianyi = pianyi_detect(img)
                pub_pianyi= rospy.Publisher('pianyi', Float32, queue_size=10)   
                pub_pianyi.publish(pianyi)
                print(pianyi)        
                # print(type(pianyi))
                # rospy.spinonce()
    except rospy.ROSInterruptException:
        print('End')
        Video.release()


        
    
