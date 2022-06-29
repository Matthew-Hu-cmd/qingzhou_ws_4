#!/usr/lib/python3.6
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
import sys
import signal
from ackermann_msgs.msg import AckermannDrive

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

def getShift(img):
	text = ''
	# 提取蓝色部分
	hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	lower_blue = np.array([85, 43, 60])
	upper_blue = np.array([140, 157, 145]) 
	mask = cv2.inRange(hsv_img, lowerb=lower_blue, upperb=upper_blue) 
	bImg = cv2.bitwise_and(img, img, mask = mask)

	# 对提取后的图像进行处理
	gray_BImg = cv2.cvtColor(bImg, cv2.COLOR_BGR2GRAY)
	ret, img_thresh = cv2.threshold(gray_BImg,10, 255, cv2.THRESH_BINARY)
	kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4, 4))
	img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
	img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_CLOSE, kernel)

	# 获取车道线中点坐标
	histogram = np.sum(img_thresh[:, :], axis=0)
	midpoint = int(histogram.shape[0] / 2)
	leftx_base = np.argmax(histogram[:midpoint])
	rightx_base = np.argmax(histogram[midpoint:]) + midpoint
	if leftx_base != 0:
		base = leftx_base + 50
		text = 'left'
	else:
		base = rightx_base + 50
		text = 'right'
	final_img = ROI_Img(img_thresh, img_thresh.shape[1] - base, 90)
	# out_img = cv2.resize(final_img, (320, 240), interpolation=cv2.INTER_AREA)
	# cv2.imshow('final', out_img)
	# cv2.waitKey(25)
	contours, hierarchy = cv2.findContours(final_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	# contours, hierarchy = cv2.findContours(final_img, 1, 2)
	if len(contours) > 0:
		M = cv2.moments(contours[0])
		cx = int(M['m10']/(M['m00']+float("1e-5")))
		cy = int(M['m01']/(M['m00']+float("1e-5")))
		if cx == 0 and cy == 0:
			return 0
		else:
			# print(cx, cy)
			# cv2.circle(img, (cx, cy), 10, (0, 0, 255))
			if text == 'left':
				shift = -np.degrees(np.arctan(float(320-cy)/float(cx + 200)))
			elif text == 'right':
				shift = np.degrees(np.arctan(float(320-cy)/float(cx + 200))) - 9
			return shift

if __name__ =='__main__':
	Video = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

	# ros节点设置
	ark_contrl = AckermannDrive()
	rospy.init_node("ackermann_cmd")
	cmd_vel_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=10)
	rate = rospy.Rate(100)

	try:
		rospy.loginfo("line detector is started...")
		while not rospy.is_shutdown():
			rate.sleep()
			for i in range(5):
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
					print(ark_contrl.steering_angle)
					cmd_vel_pub.publish(ark_contrl)
	finally:
		print('End')
		Video.release() 