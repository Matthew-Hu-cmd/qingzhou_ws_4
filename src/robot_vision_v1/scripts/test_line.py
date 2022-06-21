#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import time
from ackermann_msgs.msg import AckermannDrive  # 引用阿克曼的消息类型
from Gstreamer import *
from handleImg import *
from combine_dete import *

if __name__ == '__main__':
    # 初始化设置
    cam = cv2.VideoCapture(1)    #gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER
    freq = 3
    openColorDetector = 0
    controlFlag = 0
    pianyisamelist = [0,0,0,0,0,0,0]
    pianyi_before = 0

    # 节点设置
    ark_contrl = AckermannDrive()  # 实例化阿克曼消息

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

                cmd_vel_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=10)
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
                    ark_contrl.speed = 1.0  #0.55
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
                        print("done")
                        break

                print(pianyisamelist) 

                if cv2.waitKey(25) & 0xFF == ord('q'):
                    break
    
        print ("nav on & line off")


    cam.release()
