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

# h_min = 97
# h_max = 108
# s_min = 43
# s_max = 255
# v_min = 16
# v_max = 177
# color = "b"
# old_angle = 0
# kpl = 560
# kpm = 760
# kpr = 720
# kdm = 1600
# kdl = 1400
# kdr = 1300
# middle = 1500
# green_exist = 0
# error_check = 0
# max_error_check = 5
# check_count = 0
# angle = 0
# e_before = 0
# reached = False
# done = False
# def quit():
#     sys.exit()
# def reach_cb(msg):
#     global reached 
#     reached = msg.data
# def socket_cb(msg):
#     global h_max,h_min,s_max,s_min,v_max,v_min
#     global color
#     print(msg.data)
#     if (msg.data == True):
#         print("----------------------GREEN-------------------------")
#         color = "g"
#         h_min = 46
#         h_max = 92
#         s_min = 64
#         s_max = 207
#         v_min = 0
#         v_max = 83
#     elif (msg.data == False):
#         print("---------------------BLUE------------------------")
#         color = "b"
#         h_min = 96
#         h_max = 127
#         s_min = 43
#         s_max = 157
#         v_min = 140
#         v_max = 213
        
        
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
# rospy.init_node('linetrack', anonymous=False)

# green_light = rospy.Publisher('/traffic_light',Bool,queue_size=10)#ture检测到绿灯
# sub_color = rospy.Subscriber('/detector_trafficlight', Bool, socket_cb)#true检测红色，false检测蓝色
# ark_contrl = AckermannDrive()  # 实例化阿克曼消息
# sub_reached = rospy.Subscriber('/reached',Bool,reach_cb)
# done_pub = rospy.Publisher('/done',Bool,queue_size=10)

# signal.signal(signal.SIGINT,quit)
# signal.signal(signal.SIGTERM,quit)

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
        check_img = mask[120:,261:]
        light_img = mask[:100,:200 ]
        # cv2.imshow("raw",img)
        # cv2.imshow("line",line_img)
        # cv2.imshow("light",light_img)# 各个模块的视觉处理可视化

        # done_pub.publish(True)

        if (color == "b" and reached): # 到达车道线前面的一个点，并且识别到车道线
            cmd_vel_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=10)
            done_pub.publish(False)
            line_xy = np.column_stack(np.where(line_img == 255))
            check_xy = np.column_stack(np.where(check_img == 255)) 
            line_x = np.mean(line_xy[:,0])
            line_y = np.mean(line_xy[:,1])
            check_x = np.mean(check_xy[:,0])
            check_y = np.mean(check_xy[:,1])
            # if np.isnan(check_x) and np.isnan(check_y):
            center_x = line_x + 120
            center_y = line_y #计算中点坐标


            if np.isnan(center_x) or np.isnan(center_y):
                angle = 0
            else:
                angle = np.degrees(np.arctan(int(160-center_y)/int(center_x - 26)))
                if angle<0:
                    angle = angle-10
                else:
                    angle = angle+10
            # print(line_x,line_y)
            print(angle)
            angle = 0.7 * angle + 0.3 * old_angle
            old_angle = angle
            ark_contrl.steering_angle = angle
            ark_contrl.speed = 0.55
            cmd_vel_pub.publish(ark_contrl)

            if (np.isnan(line_x) or np.isnan(line_y))and reached:            
                error_check += 1
                if error_check > max_error_check:
                    ark_contrl.steering_angle = 0.0
                    ark_contrl.speed = 0.0
                    cmd_vel_pub.publish(ark_contrl)
                    error_check = 0
                    done_pub.publish(True)
                    print("done")
                    break



        elif (color == "g"):
            green_exist = 0
            light_xy = np.column_stack(np.where(light_img == 255))
            light_x = light_xy[:,0]
            exist = np.mean(light_x)
            if np.isnan(exist):
                green_exist = 0
            else: 
                green_exist = len(light_x)
            if (green_exist == 0):
                green_light.publish(False)
                print("stop")
            else:
                green_light.publish(True)
                print("pass")
                
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
    
    print ("nav on & line off")


cam.release()