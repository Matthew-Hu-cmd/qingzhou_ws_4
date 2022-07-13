#!/usr/bin/python3.6
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import cv2.aruco as aruco
import sys
import signal

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

if __name__ == "__main__":
	max_x, min_x, min_y, aruco_id = 0, 0, 0, 0
	dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
	Video = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

	while True:
		ret, img = Video.read()
		if img is not None: 
			img = cv2.resize(img, None, fx=0.8, fy=0.8, interpolation=cv2.INTER_CUBIC)
			gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
			parameters =  aruco.DetectorParameters_create()
			corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_img, dict, parameters=parameters)
			if ids is not None:
				aruco_id = ids[0][0]

			if aruco_id == 1:
				if corners is not None:
					x = corners[0][0][:,0]
					y = corners[0][0][:,1]
					max_x, min_x, min_y = int(max(x)), int(min(x)), int(min(y))

				poly = np.array([
					[(min_x, 0), (max_x, 0), (max_x, min_y), (min_x, min_y)]
				])

				mask = np.zeros_like(gray_img)
				cv2.fillPoly(mask, poly, 255)
				masked_img = cv2.bitwise_and(img, img, mask = mask)

				gray_img_2 = cv2.cvtColor(masked_img, cv2.COLOR_BGR2GRAY)
				ret, img_thresh = cv2.threshold(gray_img_2, 160, 255, cv2.THRESH_BINARY)
				kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
				img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
				img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_CLOSE, kernel)
				cen_img = cv2.bitwise_and(masked_img, masked_img, mask = img_thresh)

				hsv_img = cv2.cvtColor(cen_img, cv2.COLOR_BGR2HSV)
				lower_green = np.array([46, 50, 160])
				upper_green = np.array([92, 255, 255]) 
				lower_RandY = np.array([0, 30, 50])
				upper_RandY = np.array([35, 255, 255]) 
				mask_G = cv2.inRange(hsv_img, lowerb=lower_green, upperb=upper_green) 
				G_Img = cv2.bitwise_and(masked_img, masked_img, mask = mask_G)
				mask_RandY = cv2.inRange(hsv_img, lowerb=lower_RandY, upperb=upper_RandY) 
				RandY_Img = cv2.bitwise_and(masked_img, masked_img, mask = mask_RandY)
				
				cv2.imshow("masked_g", G_Img)
				cv2.imshow("masked_y", RandY_Img)


				if cv2.waitKey(25) == ord("q"):
					break
	Video.release()
