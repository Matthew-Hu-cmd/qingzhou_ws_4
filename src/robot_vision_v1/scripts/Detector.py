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
# import argparse
from _02GStreamer import *
import time
import rospy
# import matplotlib.pyplot as plt
# from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from _02CalculatePositon import *
from _03TrafficLight import *
from robot_vision.srv import app


def gstreamer_pipeline(
		# capture_width=3264, #原来的
		# capture_height=2464,
		capture_width=640,
		capture_height=480,
		display_width=640,
		display_height=480,
		# display_width=1920, #原来的
		# display_height=1080,
		framerate=33,
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

# tcp server
class Server:
    def __init__(self):
        # 设置tcp服务端的socket
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 设置重复使用
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)
        # 绑定地址和端口
        # self.server.bind(('192.168.2.111', 11000))
        self.server.bind(('192.168.43.85', 11000))
        # 设置被动监听
        print("listening 11000")
        self.server.listen(128)
        # 等待客户端连接

    def run(self):
        print('等待客户端连接')
        client_socket, addr = self.server.accept()
        print("接收到来自{}的连接".format(addr))
        ProcessServer(client_socket).start()

class ProcessServer(threading.Thread):
    """img_to_send: 将要被发送的图片，我也不太清楚他是什么类型的"""

    def __init__(self,img_to_send):
        super(ProcessServer, self).__init__()

        print("初始化Socket server")
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 设置重复使用
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)
        # 绑定地址和端口
        self.server.bind(('192.168.43.85', 11000))
        # 设置被动监听
        print("listening 11000")
        self.server.listen(128)
        # 等待客户端连接

        self.client_socket = None
        self.addr = None
        # 创建一个窗口
        # self.screen = pygame.display.set_mode((640, 480), 0, 32)
        # 分辨率
        self.resolution = (320, 240)
        # 图像
        # self.img = img

        self.is_connected = False

        self.img_to_send = img_to_send

    def updata_img_to_send(self,img):
        self.img_to_send = img

    def run(self):

        """读摄像头数据 发送给服务器"""
        # camera = cv2.VideoCapture(0)  # 摄像头对象
        # print('isOpened:', camera.isOpened())

        global Img
        # while camera.isOpened():
        while True:
            print('等待客户端连接')
            self.client_socket, self.addr = self.server.accept()
            print("接收到来自{}的连接，开始传送图像".format(self.addr))
            self.is_connected = True
            # ProcessServer(client_socket).start()
            while self.is_connected:
                try:
                    # print(Img.size)
                    # 获取摄像头数据
                    # ret, frame = camera.read()
                    # 对每一帧图片做大小处理　和大小的压缩
                    if(not len(self.img_to_send)):
                        print("no image")
                        time.sleep(1)
                        continue
                    img = cv2.resize(self.img_to_send, self.resolution)
                    # 参1图片后缀名 参2 原图片的数据 参3图片质量 0-100 越大越清晰
                    ret, img_encoded = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 100])
                    # # 转为二进制数据
                    img_encoded = img_encoded.tobytes()
                    length = len(img_encoded)
                    # 发送的数据  大小 宽 高 图片数据
                    # 数据打包变为二进制
                    # pack方法参数1 指定打包数据的数据大小  i 4字节 h代表2字节
                    all_data = struct.pack('ihh', length, self.resolution[0], self.resolution[1])+ img_encoded

                    self.client_socket.send(all_data)
                    time.sleep(0.05)
                    #发送的速度取决于客户端
                    recv_data = self.client_socket.recv(1024)
                    while(not recv_data.decode("utf-8") == "ACK"):
                        recv_data = self.client_socket.recv(1024)
                        if(not len(recv_data)):
                            print("断开来自{}的连接".format(self.addr))
                            self.client_socket.close()
                            self.client_socket = None
                            self.is_connected = False
                            break
                except:
                    # camera.release()  # 释放摄像头
                    traceback.print_exc()   
                    self.client_socket = None
                    self.is_connected = False

class Detector():
    def __init__(self):
        self.road_line_control_flag = 0 # 控制模式的标志位
        self.has_opened_color_detection = False # 是否打开颜色识别
        self.Frame = 1       # 从第1帧开始读取视频
        self.pianyi_befor = 0 # 在detect road line 里面防止突变
        # WIDTH=3280
        # HEIGHT=2464
        # 视野宽(cm)
        self.FOV_w=105
        print("初始化Video Capture")
        self.Video = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
        self.Video.set(1, self.Frame)
        print("结束初始化")

        self.ret = False # video.read返回值 判断读取是否正确，文件是否结束
        self.Img = [] # 从视频流里读取到的帧
        self.img_to_send = []# 要发送的图片

        print("初始化ROS节点:detector")
        rospy.init_node("detector") # 初始化ros节点
        self.s = rospy.Service('/vision_control', app, self.handle_app_req)     
        self.cmdpub = rospy.Publisher("/cmd_vel_filted",Twist, queue_size=2)
        self.vision = rospy.Publisher('/pianyi', Vector3, queue_size=1)   
        print("结束初始化")
        self.starttime = 0

        self.server_thread = None

    def handle_app_req(self,req):
        if(req.statue == 1):
            self.road_line_control_flag = 1
            print("++++++open roadline*******")
            self.starttime = time.time()
        elif(req.statue == 0):
            self.road_line_control_flag = 0
            print("++++++reset roadline*******")
        elif(req.statue == 2):
            self.road_line_control_flag = 2
            print("++++++close roadline(stop car)*******")
        elif(req.statue == 3):
            print("++++++open color*******")
            self.has_opened_color_detection = 1
        elif(req.statue == 4):  
            self.has_opened_color_detection = 0
            print("++++++close color*******")
        return 0

    # #####################提取ROI#############################
    def region_of_interest(self,r_image):
        """提取ROI，返回masked_image"""

        h = r_image.shape[0] #图片的高
        w = r_image.shape[1] #图片的宽
        gray = cv2.cvtColor(r_image, cv2.COLOR_BGR2GRAY) #转化为灰度图

        poly = np.array([
            [(0, h-100), (w, h-100), (w, h), (0, h)]  #将数组转化为矩阵，四个为长方形的四个顶点，从左上角开始顺时针,最下面竖着100个像素的图
        ])
        mask = np.zeros_like(gray) #输入为矩阵gray，输出为形状和gray一致的矩阵，其元素全部为黑色0
        cv2.fillPoly(mask, poly, 255)   #将mask的poly部分填充为白色255
        masked_image = cv2.bitwise_and(r_image,r_image, mask=mask)  #将r_image的mask区域提取出来给masked_image

        return masked_image

    # ###############################蓝色分割############################
    def color_seperate(self,image):
        """返回蓝色区域 dst"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   #对目标图像进行色彩空间转换
        lower_hsv = np.array([100, 43, 46])          #设定蓝色下限
        upper_hsv = np.array([124, 255, 255])        #设定蓝色上限
        # lower_hsv = np.array([80, 60, 60])          #设定蓝色下限
        # upper_hsv = np.array([150, 180, 180])        #设定蓝色上限
        mask = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)  #依据设定的上下限对目标图像进行二值化转换，低于lower,高于upper都变成0，在中间为255
        dst = cv2.bitwise_and(image, image, mask=mask)    #将image的mask区域提取出来给dst,即找到蓝色区域并赋值给dst
        # cv2.imshow('blue', dst)  #查看蓝色区域的图像
        # cv2.waitKey(3)
        return dst

        # ############################红色分割##############################
    
    def color_seperate_1(self,image):
        """返回红色区域 dst"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   #对目标图像进行色彩空间转换
        # lower_hsv = np.array([156, 43, 46])          #设定红色下限
        # upper_hsv = np.array([180, 255, 255])        #设定红色上限
        lower_hsv = np.array([0, 43, 46])          #设定红色下限
        upper_hsv = np.array([10, 255, 255])        #设定红色上限
        mask = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)  #依据设定的上下限对目标图像进行二值化转换，低于lower,高于upper都变成0，在中间为255
        dst = cv2.bitwise_and(image, image, mask=mask)    #将image的mask区域提取出来给dst,即找到红色区域并赋值给dst
        # cv2.imshow('red', dst)  #查看红色区域的图像
        # cv2.waitKey(3)
        return dst

    def detect_color_and_marker(self,Img):
        if self.ret == True:
            # global Frame
            self.Frame = self.Frame + 1
            # print(Frame)
            # %% 由Marker计算小车的相对位置，同时得到marker图像区域
            # time1 = time.time()
                # print(Frame)
            CamPosition, MarkerROI = DealMarker(Img)  # CamPosition：(x,y,z)
            # time2 = time.time()
            # print("marker :{}".format(time2-time1))
            a =  np.array([1])

            # 干嘛用的？
            projection_matrix =  np.array([[3.04423340e+02, 0, 3.25529293e+02,0],
                                                                        [0, 4.99492126e+02, 2.95309865e+02,0],
                                                                        [0, 0, 1,0]])

            # CamPosition_1 =  np.append(CamPosition, a, axis=1)
            # CamPosition_T = CamPosition_1.reshape(CamPosition_1.shape[0], 1)
            # pos = np.matmul(projection_matrix, CamPosition_T)
            if CamPosition is not None:
                # CamPosition_1 =  np.hstack(CamPosition,a)
                # CamPosition_T = CamPosition_1.reshape(CamPosition_1.shape[0], 1)
                # pos = np.matmul(projection_matrix, CamPosition_T)
                tmpPosition = Vector3(CamPosition[0],CamPosition[1],0)
            else:
                # print ('position is none')
                tmpPosition = Vector3(0,0,0)


            # %% 实现交通灯颜色检测
            # time1 = time.time()
            LightColors = TrafficLight(MarkerROI, Img)  # LightColors：0-'Red', 1-'Yellow', 2-'Green'
            # time2 = time.time()
            # print("traffic light :{}".format(time2-time1))
            # print('Frame:', Frame)
            # print(CamPosition, LightColors)     # 输出小车位置和交通灯颜色
            # print(type(LightColors))
            if(len(LightColors) > 0):
                # print(LightColors)
                colortype = LightColors[0]
                return colortype
        else:
            print('no image')
            return 1    

    def detect_road_line(self,img):
        """检测车道线偏移量"""
        pianyi=0
        pianyi_text=''
        ##############读取图像#########################
        (img_w, img_h) = img.shape[:2] #获取传入图片的长与宽   w是高 h是宽
        # print(img_h) #640-------这是宽
        lane_img=img.copy() #复制一份获取的图像给lane_img
        cropped_img=self.region_of_interest(lane_img) #对图像进行ROI的分割
        # cv2.imshow("ROI",cropped_img) #ROI区域的画面
        # cv2.waitKey(5)
        cropped_img_1=cropped_img.copy() #将ROI图像复制一份给cropped_img_1
        cropped_img = self.color_seperate(cropped_img) #将ROI图像中的蓝色部分提取出来
        gray_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY) #将提取的  ROI的蓝色部分转化为灰度图
        # cv2.imshow("gray",gray_img) #查看转化后的灰度图
        # cv2.waitKey(5)
        ret, img_thresh = cv2.threshold(gray_img,10, 255, cv2.THRESH_BINARY)  #大于10的地方就转化为白色255，返回两个值第一个是域值，第二个是二值图图像
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)) #返回一个4*4的椭圆形矩阵核，椭圆的地方是1，其他地方是0
        img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel) #开运算，运用核kernel先进行腐蚀，再进行膨胀
        img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_CLOSE, kernel) #闭运算，运用核kernel先进行膨胀，再进行腐蚀
        # cv2.imshow("img_thresh",img_thresh) #开闭运算后的图像
        # cv2.waitKey(5)
        contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # print(len(contours))#findcontours返回两个值，一个是一组轮廓信息，还有一个是每条轮廓对应的属性
        # cv2.drawContours(img_thresh,contours,-1,(0,0,255),3) #将检测到的轮廓画上去 
        # cv2.imshow("img_thresh_2",img_thresh) #绘制轮廓后的图像
        # print(len(img_thresh))
        self.server_thread.updata_img_to_send(img_thresh)
        # cv2.waitKey(5)
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
            if w>img_h/3  and con_num >1 : #如果整体轮廓的宽度大于二分之图片的宽度，则说明同时检测到了蓝白线和蓝线
                mask=np.zeros_like(gray_img) #
                cv2.rectangle(mask, (x, y), (x + 250, y + h-3), (255, 255, 255), cv2.FILLED) #将mask的部分进行白色填充，参数为填充区域的左上角顶将gray_img转化为全是0的矩阵并赋值给mask即全黑点和右下角顶点
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
                    cv2.rectangle(img, (x1, y1), (x1 + w1, img_h), (255, 255, 255), 3)#白框-----同时检测到蓝线和蓝白线给蓝白线画白框——永远贴着底画矩形框
                    pianyi=((x1+w1/2)-(img_h/2))*self.FOV_w/img_h #pianyi值为矩形方框的中线距离视野中央的实际距离
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

                #########################1.2 只检测到一条线，需要判断是蓝白线还是蓝线##############
            elif w<img_h/3  :
                # 如果是蓝白线
                if con_num>1: #轮廓数量大于1，就是有好几段蓝色
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 3) #黄框----------只检测到蓝白线并用黄框画出
                    pianyi = ((x + w / 2) - (img_h / 2)) * self.FOV_w / img_h #pianyi值为矩形方框的中线距离视野中央的实际距离
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
                        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 3) #黄框----------------检测到了最后一小段的蓝白线的蓝色
                        pianyi = ((x + w / 2) - (img_h / 2)) * self.FOV_w / img_h
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
                        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3) #蓝框-----------------只检测到蓝线并用蓝框画出
                        pianyi = 80-((x + w / 2) - (img_h / 2)) * self.FOV_w / img_h #80凑数,为了让车不开出赛道去
                        pianyi_text='left'
                            # if h <60 and w < 150 : #当只检测到中间最后一点蓝色时，判断为出去
                            #     print('99999')
                            #     return 999
            elif con_num == 1: #横向蓝线和最后一小段蓝白线的蓝线
                    if h < 50 :
                        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 3) #黄框----------------检测到了最后一小段的蓝白线的蓝色
                        pianyi = ((x + w / 2) - (img_h / 2)) * self.FOV_w / img_h
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
                        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3) #蓝框-----------------只检测到蓝线并用蓝框画出
                        pianyi = 83-((x + w / 2) - (img_h / 2)) * self.FOV_w / img_h #平滑过渡
                        pianyi_text='left'
                        # if h <60 and w < 100: #当只检测到中间最后一点蓝色时，判断为出去
                        #     print('9999')
                        #     return 999
            else : #检测到了左下角的点了
                nothing_point = 1
        # 2.未检测到蓝线或者蓝白线，就检测红线
        else:
            cropped_img_1 = self.color_seperate_1(cropped_img_1)
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
                cv2.rectangle(img, (x2, y2), (x2 + w2, img_h), (255, 0, 255), 3) #红框
                if h2 < 50 :
                    pianyi = self.pianyi_befor
                else :
                    pianyi = 20 - ((x2 + w2 / 2) - (img_h / 2)) * self.FOV_w / img_h
                    pianyi_text='right'
                #print('右偏移')
                #print(pianyi)

        #cv2.imshow('final_img', img) #车道线全图
        #cv2.waitKey(1)
        # # 返回偏移量（cm）和偏移方向（左偏或者右偏）
        # global selfpianyi_befor
        pianyi_now = abs(pianyi)
        # print("first {}".format(pianyi_now))

        # print("second {}".format(pianyi_now))   
        if pianyi_text == 'right' :
            pianyi_now = 0 - pianyi_now-12#这个数要试 #10
            # print("third {}".format(pianyi_now))   
        elif  pianyi_text == 'left' :
            pianyi_now =  pianyi_now+12 #这个数要试 #6
            # print("third {}".format(pianyi_now))   
        # print(nothing_point) #打印出有没有左下角点的干扰
        # if(abs(self.pianyi - pianyi_befor) > 30) or pianyi_befor == -pianyi_now  or nothing_point ==1: #去除剧烈跳变和检测到左下角点
        if self.pianyi_befor == -pianyi_now  or nothing_point ==1: #这一句如果加上防止突变有点危险
            pianyi_now = self.pianyi_befor       
            # print("*****检测到干扰*******")    
        self.pianyi_befor = pianyi_now
        # print(pianyi_now) #暂时
        return pianyi_now


    def run(self):
        pianyisamelist = [0,0,0,0,0,0,0]#大概需要4/40=0.1s判断车车有没有出去，可能太短了
        data = Vector3() # 发布红绿灯、偏移量的数据
        cmd_data = Twist() # 发布速度消息
        self.road_line_control_flag = 1 # 默认 0: 关闭车道线控制和检测 1 检测车道线并控制车的前进 2 减速停车
        acc = -0.1 # 退出车道线之后的减速加速度
        self.ret, self.Img = self.Video.read()
        flag_traffic = 0 # 红绿灯分频标记
        try:
            rospy.loginfo("running detector node")

            # 启动图片发送线程
            self.server_thread = ProcessServer(self.img_to_send)
            self.server_thread.start()

            while not rospy.is_shutdown():
                # time1 = time.time()
                self.ret, self.Img = self.Video.read()
                # print(Img.shape)
                # time2 = time.time()
                # cv2.imshow('c',Img) #红绿灯输入全图
                # cv2.waitKey(5)
                # time2 = time.time()
                # print("get pic :{}".format(time2-time1))
                if(self.Img is None):
                    print("img is none")
                    #当检测读取图片失败的时候，让红绿灯为绿灯1，车道线为超出车道线999
                    data.x = 1
                    data.y = 999
                else:
                    # time1 = time.time()
                    if(self.has_opened_color_detection):
                        if (flag_traffic%7) == 0 :  #对帧率进行7分频，没到整数倍就不读进来
                        # time1 = time.time()
                            Img_1 = cv2.resize(Img,(1920,1080))
                            # time2 = time.time()
                            # print("resize :{}".format(time2-time1))
                            colortype = self.detect_color_and_marker(Img_1)
                            # print(colortype) #打印红绿灯的检测最终结果
                        flag_traffic = flag_traffic + 1
                    else:
                        colortype = -1
                    # time2 = time.time()
                    # print("traffic :{}".format(time2-time1))
                    
                    if colortype == 0:
                        data.x = 0 # 红灯
                    elif colortype == 1 or colortype == 2: # 绿灯
                        data.x = 1
                    elif colortype == 3:
                        data.x = 2 # 黄灯
                    else:
                        data.x = -999 #没有检测到灯
                    if(self.has_opened_color_detection):
                        self.vision.publish(data)


                    Img_2 = cv2.resize(self.Img,(640,480))
                    Img_2 = Img_2[3: 475,3:635]  #切割掉左右下角干扰点
                    Img_2 = cv2.resize(Img_2,(640,480))


                    # cv2.imshow('c',Img_2) #车道线输入全图
                    # cv2.waitKey(5)
                    # time1 = time.time()
                    # 检测车道线偏移量并控制
                    if(self.road_line_control_flag):
                        pianyi = self.detect_road_line(Img_2)
                    else:
                        pianyi = 0

                    if(self.road_line_control_flag == 1 or False): #原来是false
                        cmd_data.linear.x = 1.0  
                        cmd_data.angular.z = (pianyi*1.8 + 3.489) /180.0*3.1415926 #新增加了data.y的系数和最后的常数项 k =0.80837   1.6
                        self.cmdpub.publish(cmd_data)

                    elif(self.road_line_control_flag == 2):
                        # print("****************stop****************")
                        cmd_data.linear.x = 1.2+acc*(time.time()-startT)  #1.2
                        print("now speed:{}".format(cmd_data.linear.x))
                        cmd_data.angular.z = -1.3 #-1.3
                        self.cmdpub.publish(cmd_data)
                    # 检测车是否开出去车道线
                    for index,value in enumerate(pianyisamelist):
                        if(not(index == len(pianyisamelist)-1)):#每个元素往前移动
                            pianyisamelist[index] = pianyisamelist[index+1]
                        else:#列表最后一个进来
                            pianyisamelist[index] = pianyi
                    if(len(set(pianyisamelist)) == 1 and (time.time()-self.starttime) > 3.0):
                        ##4.5s前不考虑车会退出赛道 大概需要5s多  可以加一个判断如果超出赛道一定时间就判断退出
                        # print("****************out****************") #暂时
                        pianyi = 999
                        # print(time.time()-starttime)
                    # print(pianyisamelist)

                    if(self.road_line_control_flag == 1):
                        data.y = pianyi
                        self.vision.publish(data)
            threading.Thread.__Thread__stop(self.server_thread)
            print("kill server thread")
                                 
        except rospy.ROSInterruptException as e:
            print('capture ROSInterruptException:{}'.format(e))
            print("killing node and release camera")
            Video.release()       
     

if __name__ == "__main__":
    Detector().run()