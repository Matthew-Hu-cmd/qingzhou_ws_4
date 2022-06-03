# -*- coding:utf-8 _*-

import numpy as np
import cv2
import cv2.aruco as aruco
import time
from numpy.core.fromnumeric import shape

# ******************************************************************************************************************#
#提取编码点#

dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)  # 编码点的类型,与生成的时候对应
K = np.array([[401.27, 0, 324.47],
              [0, 541.61,  286.99],
              [0, 0, 1]])
Dis = np.array([0.305256, 0.077563, 0.003689, 0.000838])
EMap = np.loadtxt('/home/cquer/ROSApplications/qingzhou_ws_2/src/robot_vision_v1/scripts/EMap.txt')

def CalculatePositon(Point3D, Point2D, K, Dis):
	Center = np.mean(Point3D, axis=0)
	Point3D = Point3D - Center  # 去中心化
	ret, Rvec, t = cv2.solvePnP(Point3D, Point2D, K, Dis)  # 解算位姿
	CamPosition = t[0:2, 0].flatten() + Center[0:2]  # 相机在世界坐标系下的坐标
	return CamPosition

def DealMarker(Img):
	CamPosition = None
	MarkerROI = None
	Corners, IDs, rejectedImgPoints = aruco.detectMarkers(Img, dict)
	try:
		if (Corners[0].size is 8):
			# print("into if")
			Point3D = np.empty((0, 3))
			Point2D = np.empty((0, 2))
			for i, Corner in enumerate(Corners):
				Point2D = np.vstack((Point2D, Corner.reshape(-1, 2)))
				ID = IDs.flatten()[i]
				Point3D = np.vstack((Point3D, np.hstack((EMap[ID, 3:].reshape(-1, 2), np.zeros((4, 1))))))
			CamPosition = CalculatePositon(Point3D, Point2D, K, Dis)
			aruco.drawDetectedMarkers(Img,Corners,IDs)
			cv2.putText(Img, str(CamPosition), (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 3)
			MarkerROI = np.hstack((np.min(Point2D,axis=0),np.max(Point2D,axis=0))).astype(np.int)  # xmin,ymin,xmax,ymax
	except Exception as e:
		pass
		# print(e)
	return CamPosition, MarkerROI

Red = np.array([115, 86, 232.])
#Yellow = np.array([10, 100, 140.])
Green1 = np.array([135, 200, 17.]) #之前是这个，电很足的时候的绿色
# yellow = np.array([236, 250, 28.]) #之前是这个
# Green = np.array([64, 145, 61.]) #试验，找到个中间绿色
# Red = np.array([38, 28, 230.])
Yellow = np.array([11, 81, 178.])
Green2 = np.array([35, 128, 10.]) #没什么电时候的绿色
# Colors = (Red, Yellow, Green)
Colors = (Red, Green1,Green2,Yellow)
# ColorsName = ('Red', 'Yellow', 'Green')
ColorsName = ('Red', 'Green1','Green2','Yellow')
DistThreshold =  6000   # 颜色距离阈值 #原来是5000，分辨能力较弱
#DistThreshold =  2000   # 颜色距离阈值

def JudgeLightColor(Light):
	Dist = np.empty((0,))
	for Color in Colors:
		Dist = np.append(Dist, np.sum(abs(Color - Light) ** 2)) #**代表乘方
	return np.argmin(Dist), np.min(Dist) #np.argmin 返回列表Dist中最小值的索引，np.min返回列表Dist中的最小值

# ********************************************************************************************************************#

#红绿灯相关函数

def TrafficLight(MarkerROI, Img):
	LightColors = []
	if MarkerROI is not None:  # 如果检测到Marker，CamPosition和MarkerROI就不是None
		W = MarkerROI[2] - MarkerROI[0]
		H = MarkerROI[3] - MarkerROI[1]
		MinY = max(MarkerROI[1] - int(2.2 * H), 0)
		MaxY = min(MarkerROI[3] - H, Img.shape[0])
		if MaxY <= MinY + 10:
			return LightColors
		LightImg = Img[MinY:MaxY, MarkerROI[0]:MarkerROI[2], :]  # 提取交通灯的小块区域图像

		# 提取亮点中心轮廓
		LightImgGray = cv2.cvtColor(LightImg, cv2.COLOR_BGR2GRAY)
		# cv2.imshow('1111', LightImgGray)
		th, MaskImg = cv2.threshold(LightImgGray, 200, 255, cv2.THRESH_TOZERO)
		# cv2.imshow('1111', MaskImg)
		MaskImg = cv2.morphologyEx(MaskImg, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
		# cv2.imshow('1111', MaskImg)
		contours, hierarchy = cv2.findContours(MaskImg, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		
		# a = cv2.findContours(MaskImg, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		# print(a)
		# exit()
		sel_contours = []

		# 根据面积筛选轮廓
		for index, contour in enumerate(contours):
			Area = cv2.contourArea(contour)
			Hull = cv2.convexHull(contour, False)
			HullArea = cv2.contourArea(Hull)
			# print(Area)
			# print(Area / HullArea)
			if Area > 15 and Area < 1000 and Area / HullArea > 0.9: # Area原来是20 可能太大了
			# if Area > 2 and Area < 1000 and Area / HullArea > 0.9:
				sel_contours.append(contour)
				# 形态学提取外轮廓区域
				MaskImg = np.zeros_like(LightImgGray)
				cv2.drawContours(MaskImg, [contour], -1, 255, cv2.FILLED)
				kernel = np.ones((int(H / 8), int(H / 8)), np.uint8)
				dilation = cv2.dilate(MaskImg, kernel, iterations=1)  # 膨胀
				res = np.hstack((MaskImg, dilation))
				# cv2.imshow('1111', res)
				MaskImg = dilation - MaskImg
				MaskImg = cv2.cvtColor(MaskImg, cv2.COLOR_GRAY2BGR)
				OutSide = LightImg & MaskImg
				# cv2.imshow('1111', OutSide)
				# print (OutSide.shape)
				Index = np.argwhere(np.sum(OutSide, axis=2) > 0)
				# print(Index)
				GrayLevel = OutSide[Index[:, 0], Index[:, 1], :]
				# cv2.imshow('2222', GrayLevel)
				Light = np.mean(GrayLevel, axis=0)
				# print(Light)
				Color, Dist = JudgeLightColor(Light)
				if Dist < DistThreshold:    # 颜色空间L2距离足够小，完成颜色判断
					LightColors.append(Color)

		# %% 显示交通灯小块区域
		cv2.drawContours(LightImg, sel_contours, -1, (255, 0, 0), 3)
		cv2.putText(Img, str([ColorsName[LightColor] for LightColor in LightColors]), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 3)
		# cv2.imshow('LightImg', LightImg) # 显示交通灯小块区域图像
		# cv2.waitKey(1)
	return LightColors
