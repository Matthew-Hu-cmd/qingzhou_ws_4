#!/usr/lib/envs python3.6
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import cv2.aruco as aruco
import time
import sys
import signal

color_dict = {
	# 'Blue':[[75, 50, 20], [135, 200, 130]],
	'Blue':[[75, 55, 20], [135, 200, 120]], # 识别蓝白线更稳
	'Green':[[46, 30, 160], [92, 255, 255]],
	'RandY':[[0, 30, 50], [35, 255, 255]]
}

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

def getColorArea(img, color):
	hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv_img, lowerb=np.array(color_dict[color][0]), upperb=np.array(color_dict[color][1])) 
	color_img = cv2.bitwise_and(img, img, mask = mask)
	return color_img

if __name__ == "__main__":
	color_flag = 0
	max_x, min_x, min_y, max_y, aruco_id = 0, 0, 0, 0, -1
	RandY_count, green_count, R_count = 0, 0, 0
	dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
	Video = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

	# while Video.isOpened():
	time1 = time.time()
	while Video.isOpened():
		# for i in range(5):
		# 	img = Video.grab()
		if color_flag == 1:
			print("Green, you can go.")
			color_flag, green_count, RandY_count = 0, 0, 0
			print(f'time:{time.time() - time1}')
			break
		elif color_flag == 2:
			print("Red or yellow, stop.")
			print(f'time:{time.time() - time1}')
			color_flag, green_count, RandY_count = 0, 0, 0
			time1 = time.time()
		time.sleep(0.01)
		ret, img = Video.read()
		if img is not None: 
			# cv2.imshow('aimg', img)
			# img = cv2.resize(img, None, fx=0.6, fy=0.6, interpolation=cv2.INTER_LINEAR)
			gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
			parameters =  aruco.DetectorParameters_create()
			corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_img, dict, parameters=parameters)
			# a_img = aruco.drawDetectedMarkers(img, corners, ids)
			# cv2.imshow('img', img)
			if ids is not None:
				aruco_id = ids[0][0]

			if aruco_id == 1:
				if corners:
					x = corners[0][0][:,0]
					y = corners[0][0][:,1]
					max_x, min_x, min_y, max_y = int(max(x)), int(min(x)), int(min(y)), int(max(y))

				sy =  150000/((max_y - min_y) ** 2)
				if sy > 200:
					sy = 200
				masked_img = img[int(sy):min_y + 1,min_x - 10:max_x + 10]
				# cv2.imshow("img", masked_img)

				gray_img_2 = cv2.cvtColor(masked_img, cv2.COLOR_BGR2GRAY)
				ret, img_thresh = cv2.threshold(gray_img_2, 190, 255, cv2.THRESH_BINARY)
				kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
				img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
				cen_img = cv2.bitwise_and(masked_img, masked_img, mask = img_thresh)
				# cv2.imshow("cen", cen_img)

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
				# locate_pub.publish(8)
				# print("------------------------------Light close------------------------------")

			if cv2.waitKey(25) == ord("q"):
				break
		else:
			print("NOne")
	print("------------------------------Light close------------------------------")
	Video.release()
