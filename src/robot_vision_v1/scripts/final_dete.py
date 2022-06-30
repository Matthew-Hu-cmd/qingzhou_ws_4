'''#####################################
文件名: final_dete.py

功  能：视觉 S弯

作  者:  0E
#####################################'''

#!/usr/lib/python3.6
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import queue
import rospy
import sys
import signal
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Int32

funcQueue = queue.Queue()

def quit():
    sys.exit()

signal.signal(signal.SIGINT,quit)
signal.signal(signal.SIGTERM,quit)

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

def getFunc(sub):
	if sub.data == 4:
		funcQueue.put("line")
	if sub.data == 7:
		funcQueue.put("trafficlight")

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

# 获取角度的函数
def getShift(img):
	text = ''
	# 提取蓝色部分
	hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	lower_blue = np.array([90, 53, 60])
	upper_blue = np.array([140, 157, 145]) 
	mask = cv2.inRange(hsv_img, lowerb=lower_blue, upperb=upper_blue) 
	bImg = cv2.bitwise_and(img, img, mask = mask)

	# 对提取后的图像进行处理
	gray_BImg = cv2.cvtColor(bImg, cv2.COLOR_BGR2GRAY)
	ret, img_thresh = cv2.threshold(gray_BImg,10, 255, cv2.THRESH_BINARY)
	kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4, 4))
	img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
	img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_CLOSE, kernel)

	# 获取车道线中点坐标，通过cv2.moments方法获取图心坐标
	histogram = np.sum(img_thresh[:, :], axis=0)
	midpoint = int(histogram.shape[0] / 2)
	leftx_base = np.argmax(histogram[:midpoint])
	# 如果左边有点在图的左半平面搜索，没有就在右边搜索
	if leftx_base != 0:
		base = leftx_base + 50
		text = 'left'
	else:
		rightx_base = np.argmax(histogram[midpoint:]) + midpoint
		base = rightx_base + 50
		text = 'right'

	final_img = ROI_Img(img_thresh, img_thresh.shape[1] - base, 90)
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
			if text == 'left':
				shift = -np.degrees(np.arctan(float(320-cy)/float(cx + 200)))
			elif text == 'right':
				shift = np.degrees(np.arctan(float(320-cy)/float(cx + 200))) - 8.5    
				if shift < -19:
					shift = -19     # 限制右转角度，右转角度太大会擦到车道线
		# print(shift)
		return shift

if __name__ =='__main__':
	out_check = 0

	Video = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

	# ros节点设置
	ark_contrl = AckermannDrive()

	rospy.init_node("vision_control")
	# cmd_vel_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=10)
	cmd_vel_pub = rospy.Publisher('/vision_control', AckermannDrive, queue_size=10)
	locate_pub = rospy.Publisher("/qingzhou_locate", Int32, queue_size=1)

	func_sub = rospy.Subscriber('/qingzhou_locate', Int32, getFunc)

	rate = rospy.Rate(100)

	try:
		rospy.loginfo("line detector is started...")
		while not rospy.is_shutdown():

			func = funcQueue.get()

			# 清除视频缓存
			for i in range(10):
				ret = Video.grab()

			if func == "line":
				while Video.isOpened():
					rate.sleep()
					ret,img = Video.read()
					if not ret:
						print('img is none')
						break
					else:
						pre_Img = ROI_Img(img, 0, 90)
						shift = getShift(pre_Img)

						if shift:
							ark_contrl.speed = 1.2
							ark_contrl.steering_angle = shift
							ark_contrl.steering_angle_velocity = 5
							print(ark_contrl.steering_angle)
							cmd_vel_pub.publish(ark_contrl)
							out_check = 0
						else:
							out_check += 1
						
						# 出弯判断
						if out_check > 10:
							ark_contrl.speed = 0
							ark_contrl.steering_angle = 0
							cmd_vel_pub.publish(ark_contrl)
							print('out, stop')
							out_check = 0
							break
				locate_pub.publish(5)

				print("navigation start")
			elif func == "trafficlight":
				locate_pub.publish(8)
				print("open light")

	finally:
		print('End')
		Video.release() 