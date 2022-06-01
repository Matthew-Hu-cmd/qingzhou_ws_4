#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Using to adjust parameters of controlling car to go S way
import cv2
import numpy as np
import threading
import socket
import struct
import traceback
from _02GStreamer import *
import time
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from robot_vision.srv import app
import netifaces as ni

# Whether to show image
Debug = True

# Set parameters of camera
def gstreamer_pipeline(
        capture_width=640,
        capture_height=480,
        display_width=640,
        display_height=480,
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

Video = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

Img = []	# Save data of image from camera


# TCP Server
class ProcessServer(threading.Thread):

    def __init__(self):
        super(ProcessServer, self).__init__()
        #Get wlan0 IP address, set listen port
        host = ni.ifaddresses("wlan0")[2][0]['addr']
        port = 11000
        print("初始化Socket Server")
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 设置重复使用
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)
        # 绑定地址和端口
        self.server.bind((host, port))
        # 设置被动监听
        print("listening %s:%d" % (host, port))
        self.server.listen(128)
        # 等待客户端连接

        self.client_socket = None
        self.addr = None
        self.is_connected = False
        # Set resolution of image
        self.resolution = (320, 240)

    #A thread of transport image to client
    def run(self):

        # 读摄像头数据 发送给服务器
        global Img
        if Video.isOpened():
            print('Camera is opened')
        else:
            print('Camera is closed')
        while True:
            print('等待客户端连接')
            self.client_socket, self.addr = self.server.accept()
            print("接收到来自{}的连接，开始传送图像".format(self.addr))
            self.is_connected = True
            # ProcessServer(client_socket).start()
            while self.is_connected:
                try:
                    # 对每一帧图片做大小处理　和大小的压缩
                    img = cv2.resize(Img, self.resolution)
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
                except Exception as e:
                    print("Error occured: "+e)
                    traceback.print_exc()
                    self.client_socket = None
                    self.is_connected = False


######################提取ROI#############################
def region_of_interest(r_image):
    h = r_image.shape[0] #图片的高
    w = r_image.shape[1] #图片的宽
    gray = cv2.cvtColor(r_image, cv2.COLOR_BGR2GRAY) #转化为灰度图
    poly = np.array([
        [(0, h-90), (w, h-90), (w, h), (0, h)]  #将数组转化为矩阵，四个为长方形的四个顶点，从左上角开始顺时针,最下面竖着100个像素的图 #原来是100
    ])
    mask = np.zeros_like(gray) #输入为矩阵gray，输出为形状和gray一致的矩阵，其元素全部为黑色0
    cv2.fillPoly(mask, poly, 255)   #将mask的poly部分填充为白色255
    masked_image = cv2.bitwise_and(r_image, r_image, mask=mask)  #将r_image的mask区域提取出来给masked_image
    # if Debug == True:
        # cv2.imshow("masked_image", masked_image)
    return masked_image


################################蓝色分割############################
def color_seperate_b(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   #对目标图像进行色彩空间转换
    lower_hsv = np.array([100, 60, 60])          #设定蓝色下限
    upper_hsv = np.array([124, 255, 160])        #设定蓝色上限
    # lower_hsv = np.array([80, 60, 60])          #设定蓝色下限
    # upper_hsv = np.array([150, 180, 180])        #设定蓝色上限
    mask = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)  #依据设定的上下限对目标图像进行二值化转换，低于lower,高于upper都变成0，在中间为255
    dst = cv2.bitwise_and(image, image, mask=mask)    #将image的mask区域提取出来给dst,即找到蓝色区域并赋值给dst
    # if Debug == True:
    # 	cv2.imshow('blue', dst)  #查看蓝色区域的图像
    return dst


#############################红色分割##############################
def color_seperate_r(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   #对目标图像进行色彩空间转换
    # lower_hsv = np.array([156, 43, 46])          #设定红色下限
    # upper_hsv = np.array([180, 255, 255])        #设定红色上限
    lower_hsv = np.array([0, 43, 46])          #设定红色下限
    upper_hsv = np.array([10, 255, 255])        #设定红色上限
    mask = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)  #依据设定的上下限对目标图像进行二值化转换，低于lower,高于upper都变成0，在中间为255
    dst = cv2.bitwise_and(image, image, mask=mask)    #将image的mask区域提取出来给dst,即找到红色区域并赋值给dst
    if Debug == True:
        cv2.imshow('red', dst)  #查看红色区域的图像
    return dst


##########3# Process image after color seperate##################
def binary_convert(img):
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)		#将提取的ROI的蓝色部分转化为灰度图
    ret, img_thresh = cv2.threshold(gray_img, 10, 255, cv2.THRESH_BINARY)	#大于10的地方就转化为白色255，返回两个值第一个是域值，第二个是二值图图像
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))			#返回一个4*4的椭圆形矩阵核，椭圆的地方是1，其他地方是0
    img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)		#开运算，运用核kernel先进行腐蚀，再进行膨胀
    img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_CLOSE, kernel)		#闭运算，运用核kernel先进行膨胀，再进行腐蚀
    if Debug == True:
        cv2.imshow("img_thresh", img_thresh)		#开闭运算后的图像
    return img_thresh


########################### 检测偏移的函数###########################
def pianyi_detect(img):
    pianyi = 0

    ########################读取图像#########################
    (img_w, img_h) = img.shape[:2]				#获取传入图片的长与宽   w是高 h是宽
    img_roi = region_of_interest(img)			#对图像进行ROI的分割
    img_b = color_seperate_b(img_roi)			#将ROI图像中的蓝色部分提取出来
    # img_r = color_seperate_r(img_roi)			#将ROI图像中的hong色部分提取出来
    img_thresh = binary_convert(img_b)
    contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)		#findcontours返回两个值，一个是一组轮廓信息，还有一个是每条轮廓对应的属性
    if Debug == True:
        ret = cv2.drawContours(img_b, contours, -1, (255,0,0), 3)	#将检测到的轮廓画上去 
        cv2.imshow("img_thresh_2", ret)		#绘制轮廓后的图像

    ##################1.如果检测到蓝色线或者蓝白线，进行判断###########
    if (len(contours) > 1):   # 如果检测到的轮廓数量大于1
        con_num = len(contours) #将轮廓的个数赋值给con_num
        contour1 = [] #将contour1 赋值为空列表，[]表示列表，列表是可变的序列
        for c1 in range(len(contours)): #遍历每一个轮廓
            for c2 in range(len(contours[c1])): #遍历每一个轮廓的轮廓上的点
                contour1.append(contours[c1][c2]) #将每一个轮廓的每一个点都排列起来组成一个新列表，.append() 方法用于在列表末尾添加新的对象
        contour1 = np.array(contour1) #将组成的新列表转化为矩阵，方便下一步处理
        (x, y, w, h) = cv2.boundingRect(contour1) #用一个最小的矩形，把找到的所有的轮廓包起来，返回轮值x，y是矩阵左上点的坐标，w，h是矩阵的宽和高

        #####################1.1 同时检测到蓝白线和蓝线，删选出蓝白线，计算位置########################
        if w>img_h/3  and con_num > 1 : #如果整体轮廓的宽度大于三分之图片的宽度，则说明同时检测到了蓝白线和蓝线 #原来是除以3
            mask=np.zeros_like(gray_img) #
            cv2.rectangle(mask, (x, y), (x + 180, y + h-3), (255, 255, 255), cv2.FILLED) #将mask的部分进行白色填充，参数为填充区域的左上角顶将gray_img转化为全是0的矩阵并赋值给mask即全黑点和右下角顶点
            temp_img=cv2.bitwise_and(img_thresh, img_thresh, mask=mask)         #将优化后的二值图img_thresh中的mask区域提取出来给temp_img
            contours1, hierarchy = cv2.findContours(temp_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #对只剩下蓝白线的二值图图像进行轮廓检测

            if (len(contours1) > 0):
                contour2 = []
                for c1 in range(len(contours1)):
                    for c2 in range(len(contours1[c1])):
                        contour2.append(contours1[c1][c2])
                contour2 = np.array(contour2) #将蓝白线的轮廓信息存于contour2矩阵中
                (x1, y1, w1, h1) = cv2.boundingRect(contour2) #蓝白线的轮廓信息
                if con_num > 2 : #右边的蓝线有时会看不清，断成两节
                    cv2.rectangle(img, (x1, y1), (x1 + w1, img_h), (255, 255, 255), 3)#白框-----同时检测到蓝线和蓝白线给蓝白线画白框——永远贴着底画矩形框
                    pianyi=((x1+w1/2)-(img_h/2))*FOV_w/img_h #pianyi值为矩形方框的中线距离视野中央的实际距离
                    if pianyi>0:
                        pianyi_text='right'
                    elif pianyi<0:
                        pianyi_text='left'
                    else:
                        pianyi_text = 'stright'
                else : #这个时候才是真正的蓝线
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3) #蓝框-----------------只检测到蓝线并用蓝框画出
                    pianyi = 80-((x + w / 2) - (img_h / 2)) * FOV_w / img_h #80凑数,为了让车不开出赛道去
                    pianyi_text='left'

        #########################1.2 只检测到一条线，需要判断是蓝白线还是蓝线##############
        elif w<img_h/3  :
            # 如果是蓝白线
            if con_num>1: #轮廓数量大于1，就是有好几段蓝色
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 3) #黄框----------只检测到蓝白线并用黄框画出
                pianyi = ((x + w / 2) - (img_h / 2)) * FOV_w / img_h #pianyi值为矩形方框的中线距离视野中央的实际距离
                if pianyi > 0:
                    pianyi_text='right'
                elif pianyi<0:
                    pianyi_text='left'
                else:
                    pianyi_text = 'stright'
            # 蓝线
            else: 
                    #竖直蓝线######
                # else: #看见一块蓝色并且真的是蓝线
                if h < 50 :
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 3) #黄框----------------检测到了最后一小段的蓝白线的蓝色
                    pianyi = ((x + w / 2) - (img_h / 2)) * FOV_w / img_h
                    if pianyi > 0:
                        pianyi_text='right'
                    elif pianyi<0:
                        pianyi_text='left'
                    else:
                        pianyi_text = 'stright'
                else :
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3) #蓝框-----------------只检测到蓝线并用蓝框画出
                    pianyi = 80-((x + w / 2) - (img_h / 2)) * FOV_w / img_h #80凑数,为了让车不开出赛道去
                    pianyi_text='left'
        elif con_num == 1: #横向蓝线和最后一小段蓝白线的蓝线
                if h < 50 :
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 3) #黄框----------------检测到了最后一小段的蓝白线的蓝色
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
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3) #蓝框-----------------只检测到蓝线并用蓝框画出
                    pianyi = 83-((x + w / 2) - (img_h / 2)) * FOV_w / img_h #平滑过渡
                    pianyi_text='left'
                    # if h <60 and w < 100: #当只检测到中间最后一点蓝色时，判断为出去
                    #     print('9999')
                    #     return 999
        else : #检测到了左下角的点了
            nothing_point = 1

    ################# 2.未检测到蓝线或者蓝白线，就检测红线####################
    else:
        img_r = color_seperate_r(img_roi)
        gray_img = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)
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
                pianyi = pianyi_befor
            else :
                pianyi = 50 - ((x2 + w2 / 2) - (img_h / 2)) * FOV_w / img_h
                pianyi_text='right'
            #print('右偏移')
            #print(pianyi)

    # cv2.imshow('final_img', img) #车道线全图
    # cv2.waitKey(100)
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



if __name__=="__main__":

    tcp_server_thread = ProcessServer()
    tcp_server_thread.start()
    while Video.isOpened():
        ret, Img = Video.read()
        img_copyed = Img.copy()
        img_roi = region_of_interest(img_copyed)
        pianyi_detect(img_roi)
        if Debug == True:
            cv2.imshow("img", Img)
            if cv2.waitKey(1) == ord('q'):
                print('Quit')
                cv2.destroyAllWindows()
                break
    Video.release()
    print('Camera Closed')