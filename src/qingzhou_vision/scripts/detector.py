#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
from pickle import LIST
import cv2.aruco as aruco
import numpy as np
import threading
import socket
import struct
import traceback
import time
import rospy
# import matplotlib.pyplot as plt
# from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from robot_vision.srv import app
from ackermann_msgs.msg import AckermannDrive

import ctypes

def gstreamer_pipeline(
		# capture_width=3264, #原来的
		# capture_height=2464,
		capture_width=640,
		capture_height=480,
		display_width=640,
		display_height=480,
		# display_width=1920, #原来的
		# display_height=1080,
		framerate=25,
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
np.set_printoptions(suppress=True, precision=4)
controlFlag = 0
openColorDetector = 0
Frame = 1       # 从第1帧开始读取视频
starttime = 0
global startT
emdtime = 0
greenflag = 0
Video = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
Video.set(1, Frame)
Img = []

flag = 0
pianyi_befor = 0
# WIDTH=3280
# HEIGHT=2464
# 视野宽(cm)
FOV_w=105

# #####################提取ROI#############################
def region_of_interest(r_image):
    h = r_image.shape[0] #图片的高
    w = r_image.shape[1] #图片的宽
    gray = cv2.cvtColor(r_image, cv2.COLOR_BGR2GRAY) #转化为灰度图

    poly = np.array([
        [(0, h-90), (w, h-90), (w, h), (0, h)]  #将数组转化为矩阵，四个为长方形的四个顶点，从左上角开始顺时针,最下面竖着100个像素的图 #原来是100
    ])
    mask = np.zeros_like(gray) #输入为矩阵gray，输出为形状和gray一致的矩阵，其元素全部为黑色0
    cv2.fillPoly(mask, poly, 255)   #将mask的poly部分填充为白色255
    masked_image = cv2.bitwise_and(r_image,r_image, mask=mask)  #将r_image的mask区域提取出来给masked_image
    return masked_image

# ###############################蓝色分割############################
def color_seperate(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   #对目标图像进行色彩空间转换
    lower_hsv = np.array([100, 43, 46])          #设定蓝色下限
    upper_hsv = np.array([124, 160, 160])        #设定蓝色上限
    mask = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)  #依据设定的上下限对目标图像进行二值化转换，低于lower,高于upper都变成0，在中间为255
    dst = cv2.bitwise_and(image, image, mask=mask)    #将image的mask区域提取出来给dst,即找到蓝色区域并赋值给dst
    # cv2.imshow('blue', dst)  #查看蓝色区域的图像
    # cv2.waitKey(3)
    return dst

# ############################红色分割##############################
def color_seperate_1(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   #对目标图像进行色彩空间转换
    # lower_hsv = np.array([156, 43, 46])          #设定红色下限
    # upper_hsv = np.array([180, 255, 255])        #设定红色上限
    lower_hsv = np.array([0, 43, 46])          #设定红色下限
    upper_hsv = np.array([10, 255, 255])        #设定红色上限
    mask = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)  #依据设定的上下限对目标图像进行二值化转换，低于lower,高于upper都变成0，在中间为255
    dst = cv2.bitwise_and(image, image, mask=mask)    #将image的mask区域提取出来给dst,即找到红色区域并赋值给dst
    # cv2.imshow('red', dst)  #查看红色区域的图像
    # # cv2.waitKey(3)
    return dst

# ################检测偏移的函数####################################
def pianyi_detect(img):
    pianyi=0
    pianyi_text=''
    ##############读取图像#########################
    (img_w, img_h) = img.shape[:2] #获取传入图片的长与宽   w是高 h是宽
    # print(img_h) #640-------这是宽
    lane_img=img.copy() #复制一份获取的图像给lane_img
    cropped_img=region_of_interest(lane_img) #对图像进行ROI的分割
    # cv2.imshow("ROI",cropped_img) #ROI区域的画面
    # cv2.waitKey(5)
    cropped_img_1=cropped_img.copy() #将ROI图像复制一份给cropped_img_1
    cropped_img = color_seperate(cropped_img) #将ROI图像中的蓝色部分提取出来
    gray_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY) #将提取的  ROI的蓝色部分转化为灰度图
    # cv2.imshow("gray",gray_img) #查看转化后的灰度图
    # cv2.waitKey(5)
    ret, img_thresh = cv2.threshold(gray_img,10, 255, cv2.THRESH_BINARY)  #大于10的地方就转化为白色255，返回两个值第一个是域值，第二个是二值图图像
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4, 4)) #返回一个4*4的椭圆形矩阵核，椭圆的地方是1，其他地方是0
    img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel) #开运算，运用核kernel先进行腐蚀，再进行膨胀
    img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_CLOSE, kernel) #闭运算，运用核kernel先进行膨胀，再进行腐蚀
    # cv2.imshow("img_thresh",img_thresh) #开闭运算后的图像
    # # cv2.waitKey(5)
    contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # print(len(contours))#findcontours返回两个值，一个是一组轮廓信息，还有一个是每条轮廓对应的属性
    # cv2.drawContours(img_thresh,contours,-1,(0,0,255),3) #将检测到的轮廓画上去 
    # cv2.imshow("img_thresh_2",img_thresh) #绘制轮廓后的图像
    # # cv2.waitKey(5)
    ####1.如果检测到蓝色线或者蓝白线，进行判断###########
    nothing_point = 0 #无用的点
    if (len(contours) > 0):   # 如果检测到的轮廓数量大于0
        con_num = len(contours) #将轮廓的个数赋值给con_num
        contour1 = [] #将contour1 赋值为空列表，[]表示列表，列表是可变的序列
        for c1 in range(len(contours)): #遍历每一个轮廓
                for c2 in range(len(contours[c1])): #遍历每一个轮廓的轮廓上的点
                    contour1.append(contours[c1][c2]) #将每一个轮廓的每一个点都排列起来组成一个新列表，.append() 方法用于在列表末尾添加新的对象
        contour1 = np.array(contour1) #将组成的新列表转化为矩阵，方便下一步处理
        (x, y, w, h) = cv2.boundingRect(contour1) #用一个最小的矩形，把找到的所有的轮廓包起来，返回轮值x，y是矩阵左上点的坐标，w，h是矩阵的宽和高
        # print(h)
        # ####################1.1 同时检测到蓝白线和蓝线，删选出蓝白线，计算位置########################
        if w>img_h/3  and con_num > 1 : #如果整体轮廓的宽度大于三分之图片的宽度，则说明同时检测到了蓝白线和蓝线 #原来是除以3
            # print(con_num)
            mask=np.zeros_like(gray_img) #
            cv2.rectangle(mask, (x, y), (x + 180, y + h-3), (255, 255, 255), cv2.FILLED) #将mask的部分进行白色填充，参数为填充区域的左上角顶将gray_img转化为全是0的矩阵并赋值给mask即全黑点和右下角顶点
            # cv2.imshow('mask',mask) #进行填充后的mask的图像
            # cv2.waitKey(5)   
            temp_img=cv2.bitwise_and(img_thresh, img_thresh, mask=mask) #将优化后的二值图img_thresh中的mask区域提取出来给temp_img
            # cv2.imshow('temp_img',temp_img) #只剩下蓝白线的二值图图像
            # cv2.waitKey(5)   
            contours1, hierarchy = cv2.findContours(temp_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #对只剩下蓝白线的二值图图像进行轮廓检测
            if (len(contours1) > 0):
                contour2 = []
                for c1 in range(len(contours1)):
                    for c2 in range(len(contours1[c1])):
                        contour2.append(contours1[c1][c2])
                contour2 = np.array(contour2) #将蓝白线的轮廓信息存于contour2矩阵中
                (x1, y1, w1, h1) = cv2.boundingRect(contour2) #蓝白线的轮廓信息
                if con_num > 2 : #右边的蓝线有时会看不清，断成两节
                    # cv2.rectangle(img, (x1, y1), (x1 + w1, img_h), (255, 255, 255), 3)#白框-----同时检测到蓝线和蓝白线给蓝白线画白框——永远贴着底画矩形框
                    pianyi=((x1+w1/2)-(img_h/2))*FOV_w/img_h #pianyi值为矩形方框的中线距离视野中央的实际距离
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
                else : #这个时候才是真正的蓝线
                    # cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3) #蓝框-----------------只检测到蓝线并用蓝框画出
                    pianyi = 80-((x + w / 2) - (img_h / 2)) * FOV_w / img_h #80凑数,为了让车不开出赛道去
                    pianyi_text='left'

            #########################1.2 只检测到一条线，需要判断是蓝白线还是蓝线##############
        elif w<img_h/3  :
            # 如果是蓝白线
            if con_num>1: #轮廓数量大于1，就是有好几段蓝色
                # cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 3) #黄框----------只检测到蓝白线并用黄框画出
                pianyi = ((x + w / 2) - (img_h / 2)) * FOV_w / img_h #pianyi值为矩形方框的中线距离视野中央的实际距离
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
                    #竖直蓝线######
                # else: #看见一块蓝色并且真的是蓝线
                if h < 50 :
                    # cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 3) #黄框----------------检测到了最后一小段的蓝白线的蓝色
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
                else :
                    # cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3) #蓝框-----------------只检测到蓝线并用蓝框画出
                    pianyi = 80-((x + w / 2) - (img_h / 2)) * FOV_w / img_h #80凑数,为了让车不开出赛道去
                    pianyi_text='left'
                        # if h <60 and w < 150 : #当只检测到中间最后一点蓝色时，判断为出去
                        #     print('99999')
                        #     return 999
        elif con_num == 1: #横向蓝线和最后一小段蓝白线的蓝线
                if h < 50 :
                    # cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 3) #黄框----------------检测到了最后一小段的蓝白线的蓝色
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
                else: #看见一块蓝色并且真的是蓝线
                    # cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3) #蓝框-----------------只检测到蓝线并用蓝框画出
                    pianyi = 83-((x + w / 2) - (img_h / 2)) * FOV_w / img_h #平滑过渡
                    pianyi_text='left'
                    # if h <60 and w < 100: #当只检测到中间最后一点蓝色时，判断为出去
                    #     print('9999')
                    #     return 999
        else : #检测到了左下角的点了
            nothing_point = 1


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
            # cv2.rectangle(img, (x2, y2), (x2 + w2, img_h), (255, 0, 255), 3) #红框
            if h2 < 50 :
                pianyi = pianyi_befor
            else :
                pianyi = 50 - ((x2 + w2 / 2) - (img_h / 2)) * FOV_w / img_h
                pianyi_text='right'
            #print('右偏移')
            #print(pianyi)

    # cv2.imshow('final_img', img) #车道线全图
    # cv2.waitKey(50)
    # # 返回偏移量（cm）和偏移方向（左偏或者右偏）
    global pianyi_befor
    pianyi_now = abs(pianyi)
    # print("first {}".format(pianyi_now))

    # print("second {}".format(pianyi_now))   
    if pianyi_text == 'right' :
        pianyi_now = 0 - pianyi_now-14#这个数要试 #12
        # print("third {}".format(pianyi_now))   
    elif  pianyi_text == 'left' :
        pianyi_now =  pianyi_now+8 #这个数要试 #6
        # print("third {}".format(pianyi_now))   
    # print(nothing_point) #打印出有没有左下角点的干扰
    # if(abs(pianyi - pianyi_befor) > 30) or pianyi_befor == -pianyi_now  or nothing_point ==1: #去除剧烈跳变和检测到左下角点
    if pianyi_befor == -pianyi_now  or nothing_point ==1: #这一句如果加上防止突变有点危险
        pianyi_now = pianyi_befor       
        # print("*****检测到干扰*******")    
    pianyi_befor = pianyi_now
    # print(pianyi_now) #暂时
    return pianyi_now
# def iscontrolsubcb(data,arg):
#     global controlFlag
#     # if(data.data == 1):
#     #     controlFlag = 1


if __name__ == '__main__':
    # 输入图像设置
    data = Vector3()
    pianyibefore = 0
    pianyicount = 0
    pianyisamelist = [0,0,0,0,0,0,0]#大概需要4/40=0.1s判断车车有没有出去，可能太短了
    controlFlag = 10 #原来是10
    openColorDetector = 0 #原来是0
    global flag_traffic
    flag_traffic = 0
    redtime = 0
    acc = -0.1
    ret, Img = Video.read()
    # print(Img.shape)
    # iscontrolsub = rospy.Subscriber("/is_version_cont",Float32,iscontrolsubcb,1)
    cmdData = AckermannDrive()
    cmdpub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size=2)
    rospy.init_node("detector") # 初始化ros节点
    vision = rospy.Publisher('/pianyi', Vector3, queue_size=1) 
    cmd_data_filter_client = rospy.ServiceProxy("/cmdvel_filter_client",app)

    rate = rospy.Rate(10)

    try:
        rospy.loginfo("detector node is started...")
        while not rospy.is_shutdown():
            rate.sleep()
            ret, Img = Video.read()
            if(Img is None):
                print("img is none")
                #当检测读取图片失败的时候，让红绿灯为绿灯1，车道线为超出车道线999
            data.x = 1
            data.y = 999
            Img_2 = cv2.resize(Img,(640,480))
            Img_2 = Img_2[3: 475,3:635]  #切割掉左右下角干扰点
            Img_2 = cv2.resize(Img_2,(640,480))
            cv2.imshow('c',Img_2) #车道线输入全图
            cv2.waitKey(5)
            # time1 = time.time()
            pianyi = pianyi_detect(Img_2)
            cmdData.speed = 0.1
            cmdData.steering_angle = (pianyi*1.6 + 3.489) #新增加了data.y的系数和最后的常数项 k =0.80837   1.6
            cmdpub.publish(cmdData)

            pianyibefore = pianyi
            for index,value in enumerate(pianyisamelist):
                if(not(index == len(pianyisamelist)-1)):#每个元素往前移动
                    pianyisamelist[index] = pianyisamelist[index+1]
                else:#列表最后一个进来
                    pianyisamelist[index] = pianyi
            if(len(set(pianyisamelist)) == 2):
                if((time.time()-starttime) > 2.0):
                ##4.5s前不考虑车会退出赛道 大概需要5s多  可以加一个判断如果超出赛道一定时间就判断退出
                    # print("****************out****************") #暂时
                    pianyi = 999
                # print(time.time()-starttime)
            print(pianyisamelist)
            

    except rospy.ROSInterruptException:
        print('End')
        Video.release()    
