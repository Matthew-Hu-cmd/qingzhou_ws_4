#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

'''#####################################
文件名: final_dete.py

功  能：视觉 S弯

作  者:  0E
#####################################'''

import cv2
import cv2.aruco as aruco
import numpy as np
import queue
import rospy
import sys
import signal
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Int32

def quit():
    sys.exit()

signal.signal(signal.SIGINT,quit)
signal.signal(signal.SIGTERM,quit)

class Start():
    def __init__(self):
        self.ark_pub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 3)
    def __call__(self):
        ark_data = AckermannDrive() 
        rospy.timer.sleep(rospy.Duration(2))

        rospy.loginfo("Robot will move backward for 3s.")
        ark_data.speed = -0.3
        ark_data.steering_angle = 0
        self.ark_pub.publish(ark_data)
        rospy.timer.sleep(rospy.Duration(3))

        rospy.loginfo("Robot will move forward for 3s.")
        ark_data.speed = 0.3
        ark_data.steering_angle = 0
        self.ark_pub.publish(ark_data)
        rospy.timer.sleep(rospy.Duration(3))

        rospy.loginfo("Robot stop.")
        ark_data.speed = 0
        ark_data.steering_angle = 0
        self.ark_pub.publish(ark_data)
        rospy.timer.sleep(rospy.Duration(2))

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

# 全局先进先出队列
funcQueue = queue.Queue()
color_dict = {
	'Blue':[[100, 55, 20], [135, 200, 115]],
	'Green':[[46, 50, 160], [92, 255, 255]],
	'RandY':[[0, 30, 50], [35, 255, 255]]
}

def getFunc(sub):
	global bre
	bre = 0
	if sub.data == 4:
		funcQueue.put("line")
	elif sub.data == 1:
		funcQueue.put("trafficlight")
	elif sub.data == 3 or sub.data == 0:
		bre = 1

# 提取ROI部分，根据传入的x， y参数确定提取的范围
def ROI_Img(img, x, y):
	h, w = img.shape[:2]
	
	poly = np.array([
		[(0, h - y), (w - x, h - y), (w - x, h), (0, h)]
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
	kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
	img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
	img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_CLOSE, kernel)
	return img_thresh

# 根据传入的color，从color_dict中获取对应上下限提取对应的颜色
def getColorArea(img, color):
	hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv_img, lowerb=np.array(color_dict[color][0]), upperb=np.array(color_dict[color][1])) 
	color_img = cv2.bitwise_and(img, img, mask = mask)
	return color_img

# 获取角度的函数
def getShift(img):
	# 得到处理后的蓝色图片
	b_img = getColorArea(img, 'Blue')
	img_thresh = handleImg(b_img, 4, 10)

	# 获取车道线中点坐标，通过cv2.moments方法获取图心坐标
	histogram = np.sum(img_thresh[:, :], axis=0)
	midpoint = int(histogram.shape[0] / 2)
	leftx_base = np.argmax(histogram[:midpoint])
	rightx_base = np.argmax(histogram[midpoint + 30:])
	# 如果左右同时有点的话，基点为靠左的点，+50是为了拿到完整提取的车道线，以便提取图心
	if leftx_base != 0 and rightx_base != 0:
		base = leftx_base + 30
	else:
		base = rightx_base + midpoint + 30

	final_img = ROI_Img(img_thresh, img_thresh.shape[1] - base, 60)
	# out_img = cv2.resize(img_thresh, (320, 240), interpolation=cv2.INTER_AREA)
	# cv2.imshow('final', out_img)
	# cv2.waitKey(25)
	contours, hierarchy = cv2.findContours(final_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	if len(contours) > 0:
		# 获取图心，即提取部分的图像的车道线中点坐标
		M = cv2.moments(contours[0])
		cx = int(M['m10']/(M['m00']+float("1e-5")))   # 加上float防止除数可能为零的情况
		cy = int(M['m01']/(M['m00']+float("1e-5")))
		# 防止中间有时候处理有问题导致找不到中点，同时在出弯时可以用来判断
		if cx == 0 and cy == 0:
			shift = 0
		else:
			shift = np.degrees(np.arctan(float(160 - cx)/float(cy - 26)))
		return shift

if __name__ =='__main__':
	# ros节点设置
	rospy.init_node("vision_control")

	# cmd_vel_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=1)    # 单开脚本的时候用这个
	cmd_vel_pub = rospy.Publisher('/vision_control', AckermannDrive, queue_size=1)
	locate_pub = rospy.Publisher("/qingzhou_locate", Int32, queue_size=1)

	func_sub = rospy.Subscriber('/qingzhou_locate', Int32, getFunc)

	first_start = Start()
	first_start()

	max_x, min_x, min_y, aruco_id = 0, 0, 0, -1
	out_check, angle, now_angle = 0, 5, 0
	Kp, Kd = 1.0, 0.22
	now_err, last_err = 0, 0
	green_count, RandY_count, color_flag = 0, 0, 0
	dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
	parameters =  aruco.DetectorParameters_create()

	Video = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

	ark_contrl = AckermannDrive()     # 阿克曼消息

	rate = rospy.Rate(100)

	try:
		rospy.loginfo("line detector is started...")
		while not rospy.is_shutdown():
			
			# .get()方法获取到数据才会执行下面的程序，否则阻塞
			func = funcQueue.get()

			# 清除缓存
			for i in range(10):
				ret = Video.grab()

			if func == "line":
				print("------------------------------Line------------------------------")
				while Video.isOpened():
					rate.sleep()
					if bre == 1:
						break
					ret,img = Video.read()
					if ret is None:
						print('img is none')
						break
					else:
						# 出弯判断
						if out_check > 10:
							ark_contrl.speed = 0
							ark_contrl.steering_angle = 0
							cmd_vel_pub.publish(ark_contrl)
							print('out, stop')
							out_check, angle, now_angle, now_err, last_err = 0, 5, 0, 0, 0
							break

						img = cv2.resize(img, (320, 240), interpolation=cv2.INTER_AREA)
						pre_Img = ROI_Img(img, 0, 60)
						shift = getShift(pre_Img)

						if shift:     # PID控制角度
							out_check = 0
							last_err = now_err
							now_err = 0 - shift
							now_angle = Kp * now_err + Kd * (now_err - last_err)
						else:
							out_check += 1
						
						# if abs(angle + now_angle) > 17:
						# 	print("Sudden change.")
						# elif abs(angle + now_angle) > 4 and angle * (-now_angle) < 0:
						# 	print("Error point.")
						# else:
						angle = -now_angle
						ark_contrl.speed = 1.2
						ark_contrl.steering_angle = angle
						print(f"angle:{angle}")
						cmd_vel_pub.publish(ark_contrl)
						
				locate_pub.publish(5)
				print("------------------------------Navigation start------------------------------")

			elif func == "trafficlight":
				print("------------------------------Light open------------------------------")
				# color_flag = 0时在判断中，color_flag = 1时检测到绿灯，color_flag = 3时检测到红灯或黄灯
				while Video.isOpened():
					rate.sleep()
					if bre == 1:
						break
					if color_flag == 1:
						print("Green, you can go.")
						color_flag, green_count, RandY_count = 0, 0, 0
						break
					elif color_flag == 2:
						print("Red or yellow, stop.")
						color_flag, green_count, RandY_count = 0, 0, 0

					ret, img = Video.read()
					if img is not None: 
						img = cv2.resize(img, None, fx = 0.5, fy = 0.5, interpolation = cv2.INTER_CUBIC)
						gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
						corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_img, dict, parameters=parameters)   # 没有标定，参数是默认的
						if ids is not None:
							aruco_id = ids[0][0]

						if aruco_id == 1:   # 只有看到的id为1才执行判断
							if len(corners) > 0:
								x = corners[0][0][:,0]
								y = corners[0][0][:,1]
								max_x, min_x, min_y = int(max(x)), int(min(x)), int(min(y))   # 提取aruco码上面的部分，即红绿灯部分

							poly = np.array([
								[(min_x, 0), (max_x, 0), (max_x, min_y), (min_x, min_y)]
							])

							mask = np.zeros_like(gray_img)
							cv2.fillPoly(mask, poly, 255)
							light_img = cv2.bitwise_and(img, img, mask = mask)

							img_Lthresh = handleImg(light_img, 2, 180)
							cen_img = cv2.bitwise_and(light_img, light_img, mask = img_Lthresh)
							green_Limg = getColorArea(cen_img, 'Green')
							RandY_Limg = getColorArea(cen_img, 'RandY')

							if green_Limg.any():
								green_count += 1
							if RandY_Limg.any():
								RandY_count += 1

							if green_count == 8:
								color_flag = 1
							elif RandY_count == 8:
								color_flag = 2
				locate_pub.publish(8)
				print("------------------------------Light close------------------------------")
	finally:
		print('End')
		Video.release() 
