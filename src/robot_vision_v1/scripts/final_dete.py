#! /usr/bin/env python3.6
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
import time
import sys
import signal
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Int32

filename = 1 # debug

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
		framerate=60,
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
	# 'Blue':[[75, 50, 20], [135, 200, 130]],
	'Blue':[[75, 55, 20], [135, 200, 120]], # 识别蓝白线更稳
	'Green':[[46, 30, 160], [92, 255, 255]],
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

	# cv2.imshow('masked_img', masked_img) #debug

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
def getShift(img, last_x):
	errorp = False
	cx, cy = 0, 0
	h, w = img.shape[:2]
	offset, shift, x = 0, 0, 160
	# 得到处理后的蓝色图片
	b_img = getColorArea(img, 'Blue')
	img_thresh = handleImg(b_img, 4, 10)

	# 获取车道线中点坐标，通过cv2.moments方法获取图心坐标
	histogram = np.sum(img_thresh[:, :], axis=0)
	midpoint = int(histogram.shape[0] / 2)
	leftx_base = np.argmax(histogram[:midpoint])
	rightx_base = np.argmax(histogram[midpoint + 80:])

	# 如果左右同时有点的话，基点为靠左的点，+50是为了拿到完整提取的车道线，以便提取图心
	if leftx_base != 0:
		final_img = ROI_Img(img_thresh, 0, leftx_base + 55, h - 70, h)
	else:
		final_img = ROI_Img(img_thresh, 0, w, h - 70, h)

	# cv2.imshow('final', final_img)
	# cv2.waitKey(25)
	contours, hierarchy = cv2.findContours(final_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	if len(contours) > 0:
		(x, y, w, h) = cv2.boundingRect(contours[0])

		# cv2.circle(img, (x, y), 3, (0, 0, 255), -1) #debug
		
		# print(x, last_x) #debug
		if w > 70 or h > 40:
			print("Blue.") 
			offset = 150	
		elif (x - last_x) > 70:
			print("Error point.") #debug
			errorp = True
		# 获取图心，即提取部分的图像的车道线中点坐标
		M = cv2.moments(contours[0])
		if M['m00']:
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			# cv2.circle(img, (cx, cy), 3, (255, 0, 0), -1) #debug
		# 防止中间有时候处理有问题导致找不到中点，同时在出弯时可以用来判断
		if cx == 0 and cy == 0:
			shift = 0
		else:
			shift = np.degrees(np.arctan(float(160 - cx + offset)/float(cy - 26)))
	if errorp:	#debug
		shift = 0
	# else: #debug
	# 	cv2.imwrite('/home/cquer/qingzhou_ws_4/src/robot_vision_v1/img/out_img'+str(filename)+'.png', img)
	# 	print("write out img "+str(filename), shift, cx, cy)
	# 	filename = filename + 1
	return shift, x

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
	out_check, angle, now_angle, last_x = 0, 0, 0, 160
	Kp, Kd = 1.0, 0.22
	now_err, last_err = 0, 0
	D_error=[0, 0, 0]
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
			for i in range(20):
				ret = Video.grab()

			if func == "line":
				print("------------------------------Line------------------------------")
				while Video.isOpened():
					rate.sleep()
					ret,img = Video.read()
					if ret is None:
						print('img is none')
						break
					else:
						# 出弯判断
						if out_check > 8:
							ark_contrl.speed = 0
							ark_contrl.steering_angle = 0
							cmd_vel_pub.publish(ark_contrl)
							print('out, stop')
							out_check, angle, now_angle, now_err, last_err, last_x = 0, 0, 0, 0, 0, 160
							D_error = [0, 0, 0]
							break

						img = cv2.resize(img, (320, 240), interpolation=cv2.INTER_AREA)
						img = cv2.GaussianBlur(img, (5, 5), 0)
						# pre_Img = ROI_Img(img, 0, img.shape[1], img.shape[0] - 75, img.shape[0])
						#  尝试将图像边缘去除，边缘亮度比中心亮度大，影响到HSV分割
						pre_Img = ROI_Img(img, 20, img.shape[1]-20, img.shape[0] - 75, img.shape[0])
						shift, last_x = getShift(pre_Img, last_x)
						# cv2.waitKey(1)

						if shift:     # PID控制角度
							out_check = 0
							now_err = 0 - shift
							D_error = D_error[1:]
							D_error.append(now_err - last_err)
							now_angle = Kp * now_err + Kd * (D_error[2] * 0.6 + D_error[1] * 0.3 + D_error[0] * 0.1)
							last_err = now_err
						else:
							out_check += 1
							print("out_check: ", out_check)
						
						if abs(angle + now_angle) > 30:
							print("Sudden change.")
						elif out_check > 3:
							angle = angle * 0.6
						else:
							angle = -now_angle

						ark_contrl.speed = 1.2
						ark_contrl.steering_angle = angle

						print(f"angle:{angle}") #debug
						cmd_vel_pub.publish(ark_contrl)
						
				locate_pub.publish(5)
				print("------------------------------Navigation start------------------------------")

			elif func == "trafficlight":
				# time1 = time.time()
				print("------------------------------Light open------------------------------")
				# color_flag = 0时在判断中，color_flag = 1时检测到绿灯，color_flag = 2时检测到红灯或黄灯
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
						# img = cv2.resize(img, None, fx = 0.5, fy = 0.5, interpolation = cv2.INTER_CUBIC)
						gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
						corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_img, dict, parameters=parameters)   # 没有标定，参数是默认的
						if ids is not None:
							aruco_id = ids[0][0]

						if aruco_id == 1:   # 只有看到的id为1才执行判断
							if len(corners) > 0:
								x = corners[0][0][:,0]
								y = corners[0][0][:,1]
								max_x, min_x, min_y, max_y = int(max(x)), int(min(x)), int(min(y)), int(max(y))   # 提取aruco码上面的部分，即红绿灯部分

							sy =  150000/((max_y - min_y) ** 2)
							if sy > 200:
								sy = 200
							masked_img = img[int(sy):min_y + 10,min_x - 5:max_x + 5]

							gray_img_2 = cv2.cvtColor(masked_img, cv2.COLOR_BGR2GRAY)
							ret, img_thresh = cv2.threshold(gray_img_2, 190, 255, cv2.THRESH_BINARY)
							kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
							img_Lthresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
							cen_img = cv2.bitwise_and(masked_img, masked_img, mask = img_Lthresh)
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
				# print(time.time() - time1)
	finally:
		print('End')
		Video.release() 
