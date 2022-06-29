#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
import time
import sys
import signal
from ackermann_msgs.msg import AckermannDrive  # 引用阿克曼的消息类型
from std_msgs.msg import Bool
from pickle import REDUCE, TRUE
from PIL.Image import new
# from Marker import *
from Trafficlight import *
from Gstreamer import *
from handleImg import *

reached = False
done = False
use = 'line'

def quit():
    sys.exit()
def reach_cb(msg):
    global reached 
    reached = msg.data
def socket_cb(msg):
    global use
    print(msg.data)
    if (msg.data == True):
        print("----------------------LIGHT-------------------------")
        use = "trafficlight"
    elif (msg.data == False):
        print("---------------------LINE------------------------")
        use = "line"

# ################################红绿灯识别##############################
def detector(Img):
    CamPosition, MarkerROI = DealMarker(Img)  # CamPosition：(x,y,z)
    projection_matrix =  np.array([[3.04423340e+2, 0, 3.25529293e+02,0],
                                                                [0, 4.99492126e+02, 2.95309865e+02,0],
                                                                [0, 0, 1,0]])
    if CamPosition is not None:
        tmpPosition = Vector3(CamPosition[0],CamPosition[1],0)
    else:
        # print ('position is none')
        tmpPosition = Vector3(0,0,0)
    # %% 实现交通灯颜色检测
    LightColors = TrafficLight(MarkerROI, Img)  # LightColors：0-'Red', 1-'Yellow', 2-'Green'
    # print(CamPosition, LightColors)     # 输出小车位置和交通灯颜色
    if(len(LightColors) > 0):
        #print(LightColors)
        colortype = LightColors[0]
        return colortype
# #########################################################################

# #################################偏移检测#################################
def pianyi_detect(img):
    pianyi_befor = 0
    FOV_w=105
    pianyi = 0
    pianyi_text = ''

    ##############处理图像##############
    img_w = img.shape[1] #图片的宽
    cropped_img = region_of_interest(img)  #对图像进行ROI的分割
    # cv2.imshow("ROI",cropped_img) 
    # cv2.waitKey(5)

    blue_img = color_blue_seperate(cropped_img) #将ROI图像中的蓝色部分提取出来
    gray_img = cv2.cvtColor(blue_img, cv2.COLOR_BGR2GRAY) #将提取的ROI的蓝色部分转化为灰度图
    # cv2.imshow("gray",gray_img) #查看转化后的灰度图
    # cv2.waitKey(5)

    ret, img_thresh = cv2.threshold(gray_img, 10, 255, cv2.THRESH_BINARY)  #大于10的地方就转化为白色255，返回两个值第一个是域值，第二个是二值图图像
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4, 4)) #返回一个4*4的椭圆形矩阵核，椭圆的地方是1，其他地方是0
    img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel) #开运算，运用核kernel先进行腐蚀，再进行膨胀
    img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_CLOSE, kernel) #闭运算，运用核kernel先进行膨胀，再进行腐蚀
    # cv2.imshow("img_thresh",img_thresh) #开闭运算后的图像
    # cv2.waitKey(5)

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

        # ####################1.1 同时检测到蓝白线和蓝线，删选出蓝白线，计算位置########################
        if w>img_w/3  and con_num > 1 : #如果整体轮廓的宽度大于三分之图片的宽度，则说明同时检测到了蓝白线和蓝线 #原来是除以3
            mask=np.zeros_like(gray_img) 
            # cv2.rectangle(mask, (x, y), (x + 180, y + h-3), (255, 255, 255), cv2.FILLED) #将mask的部分进行白色填充，参数为填充区域的左上角顶将gray_img转化为全是0的矩阵并赋值给mask即全黑点和右下角顶点
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
                    # cv2.rectangle(img, (x1, y1), (x1 + w1, img_w), (255, 255, 255), 3)#白框-----同时检测到蓝线和蓝白线给蓝白线画白框——永远贴着底画矩形框
                    pianyi=((x1+w1/2)-(img_w/2))*FOV_w/img_w #pianyi值为矩形方框的中线距离视野中央的实际距离
                    if pianyi>0:
                        pianyi_text='right'
                    elif pianyi<0:
                        pianyi_text='left'
                    else:
                        pianyi_text = 'stright'
                else : 
                    #这个时候才是真正的蓝线
                    # cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3) #蓝框-----------------只检测到蓝线并用蓝框画出
                    pianyi = 80-((x + w / 2) - (img_w / 2)) * FOV_w / img_w #80凑数,为了让车不开出赛道去
                    pianyi_text='left'

        # #########################1.2 只检测到一条线，需要判断是蓝白线还是蓝线##############
        elif w<img_w/3  :
            # 如果是蓝白线
            if con_num>1: #轮廓数量大于1，就是有好几段蓝色
                # cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 3) #黄框----------只检测到蓝白线并用黄框画出
                pianyi = ((x + w / 2) - (img_w / 2)) * FOV_w / img_w #pianyi值为矩形方框的中线距离视野中央的实际距离
                if pianyi > 0:
                    pianyi_text='right'
                elif pianyi<0:
                    pianyi_text='left'
                else:
                    pianyi_text = 'stright'
            # 蓝线
            else: 
                ########竖直蓝线######
                if h < 50 :
                    # cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 3) #黄框----------------检测到了最后一小段的蓝白线的蓝色
                    pianyi = ((x + w / 2) - (img_w / 2)) * FOV_w / img_w
                    if pianyi > 0:
                        pianyi_text='right'
                    elif pianyi<0:
                        pianyi_text='left'
                    else:
                        pianyi_text = 'stright'
                else :
                    # cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3) #蓝框-----------------只检测到蓝线并用蓝框画出
                    pianyi = 80-((x + w / 2) - (img_w / 2)) * FOV_w / img_w #80凑数,为了让车不开出赛道去
                    pianyi_text='left'
        elif con_num == 1: 
                #横向蓝线和最后一小段蓝白线的蓝线
                if h < 50 :
                    # cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 3) #黄框----------------检测到了最后一小段的蓝白线的蓝色
                    pianyi = ((x + w / 2) - (img_w / 2)) * FOV_w / img_w
                    if pianyi > 0:
                        pianyi_text='right'
                    elif pianyi<0:
                        pianyi_text='left'
                    else:
                        pianyi_text = 'stright'
                else: 
                    #看见一块蓝色并且真的是蓝线
                    # cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3) #蓝框-----------------只检测到蓝线并用蓝框画出
                    pianyi = 83-((x + w / 2) - (img_w / 2)) * FOV_w / img_w #平滑过渡
                    pianyi_text='left'
        else : 
            #检测到了左下角的点了
            nothing_point = 1


    # 2.未检测到蓝线或者蓝白线，就检测红线
    else:
        cropped_img = color_red_seperate(cropped_img)
        gray_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
        # gray_img = cv2.GaussianBlur(gray_img, (5, 5), 0, 0, cv2.BORDER_DEFAULT)
        ret, img_thresh = cv2.threshold(gray_img, 10, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4, 4))
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
            # cv2.rectangle(img, (x2, y2), (x2 + w2, img_w), (255, 0, 255), 3) #红框
            if h2 < 50 :
                pianyi = pianyi_befor
            else :
                pianyi = 50 - ((x2 + w2 / 2) - (img_w / 2)) * FOV_w / img_w
                pianyi_text='right'

    # cv2.imshow('final_img', img) #车道线全图
    # cv2.waitKey(50)
    # print("pianyi_text")
    
    # 返回偏移量（cm）和偏移方向（左偏或者右偏）
    pianyi_now = abs(pianyi)

    if pianyi_text == 'right' :
        pianyi_now = 0 - pianyi_now-14    #这个数要试 #12  
    elif  pianyi_text == 'left' :
        pianyi_now =  pianyi_now+8        #这个数要试 #6  

    # print(nothing_point) #打印出有没有左下角点的干扰
    # if(abs(pianyi - pianyi_befor) > 30) or pianyi_befor == -pianyi_now  or nothing_point ==1: #去除剧烈跳变和检测到左下角点

    if pianyi_befor == -pianyi_now  or nothing_point ==1: #这一句如果加上防止突变有点危险
        pianyi_now = pianyi_befor       
        # print("*****检测到干扰*******")    
    pianyi_befor = pianyi_now
    # print(pianyi_now) #暂时
    return pianyi_now
# #############################################################################

if __name__ == '__main__':
    # 初始化设置
    cam = cv2.VideoCapture(0)    #gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER
    freq = 5
    openColorDetector = 0
    controlFlag = 0
    pianyisamelist = [0,0,0,0,0,0,0]
    pianyi_before = 0

    # 节点设置
    rospy.init_node('linetrack', anonymous=False)

    green_light = rospy.Publisher('/traffic_light',Bool,queue_size=10)#ture检测到绿灯
    sub_color = rospy.Subscriber('/detector_trafficlight', Bool, socket_cb)#true检测灯，false检测线
    ark_contrl = AckermannDrive()  # 实例化阿克曼消息
    sub_reached = rospy.Subscriber('/reached',Bool,reach_cb)
    done_pub = rospy.Publisher('/done',Bool,queue_size=10)

    signal.signal(signal.SIGINT,quit)
    signal.signal(signal.SIGTERM,quit)

    while True:
        idx = 0
        while True:
            idx += 1
            ret = cam.grab()
            if not ret:
                print("grab error")
                break

            if idx % freq == 1:
                ret, img = cam.retrieve()

                if img is None:
                    print("img is none")
                    break
                else:
                    openColorDetector = 1
                    controlFlag = 1

                if (use == "line" and reached): # 到达车道线前面的一个点，并且识别到车道线
                    cmd_vel_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=10)
                    done_pub.publish(False)
                    img = cv2.resize(img,(640,480))
                    starttime = time.time()
                    img = img[3: 475,3:635]  #切割掉左右下角干扰点
                    img_line = cv2.resize(img,(640,480))

                    if(controlFlag):
                        pianyi = pianyi_detect(img_line)
                    else:
                        pianyi = 0

                    if(controlFlag == 1 or False): #原来是false
                        angle = (pianyi*1.6 + 3.489) /180.0*3.1415926 #新增加了data.y的系数和最后的常数项 k =0.80837   1.6
                        ark_contrl.steering_angle = angle
                        ark_contrl.speed = 0.55  #0.55
                        cmd_vel_pub.publish(ark_contrl)

                    pianyi_before = pianyi

                    for index,value in enumerate(pianyisamelist):
                        if(not(index == len(pianyisamelist)-1)):
                            #每个元素往前移动
                            pianyisamelist[index] = pianyisamelist[index+1]
                        else:
                            #列表最后一个进来
                            pianyisamelist[index] = pianyi

                    if(len(set(pianyisamelist)) == 1):
                        #4.5s前不考虑车会退出赛道 大概需要5s多  可以加一个判断如果超出赛道一定时间就判断退出
                        if((time.time()-starttime) > 2.0):
                            ark_contrl.steering_angle = 0.0
                            ark_contrl.speed = 0.0
                            cmd_vel_pub.publish(ark_contrl)
                            done_pub.publish(True)
                            print("done")
                            break

                    print(pianyisamelist) 

                elif (use == "trafficlight"):
                    if openColorDetector:
                        img_light = cv2.resize(img,(1920,1080))
                        colortype = detector(img_light)
                    else:
                        colortype = -1
                    
                    if colortype == 0 or colortype == 3: #红灯或者黄灯
                        green_light.publish(False)
                        print("red or yellow, stop")

                    elif colortype == 1 or colortype ==2: #绿灯
                        green_light.publish(True)
                        print("green, pass")         
                        
                    else:
                        green_light.publish(True)
                        print("no light, pass")  

                if cv2.waitKey(25) & 0xFF == ord('q'):
                    break
    
        print ("nav on & line off")


    cam.release()
