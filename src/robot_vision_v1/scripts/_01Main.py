# -*- coding:utf-8 _*-
import numpy as np
import cv2
import matplotlib.pyplot as plt
from _02CalculatePositon import *
from _03TrafficLight import *

np.set_printoptions(suppress=True, precision=4)

# 载入参数，载入地图
Frame = 1       # 从第1000帧开始读取视频
#Video = cv2.VideoCapture('/home/ltz/test003.avi')
# Video = cv2.VideoCapture('/home/ltz/test004.mp4')
Video = cv2.VideoCapture(0)
#Video = cv2.VideoCapture('/home/ltz/test002.avi')
#Video = cv2.VideoCapture('/home/ltz/test001.avi')
#ImgTest = cv2.imread('/home/ltz/jtd/test 005.png')
Video.set(1, Frame)

# plt.ion()
# ax = plt.subplot()  # 创建一个三维的绘图工程
# for Marker in EMap:
# 	Corners = Marker[1:].reshape(5, 2)
# 	ax.scatter(Corners[:, 0], Corners[:, 1])
# 	ax.text(Corners[0, 0], Corners[0, 1], str(int(Marker[0])))
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_xlim(-400, 400)
# ax.set_ylim(400, -400)
# ax.set_aspect('equal')

while 1:
	# 逐帧读取
	ret, Img = Video.read()
#	Img  = cv2.resize(Img,(1920,1080),interpolation=cv2.INTER_AREA)
	if ret == True:
		Frame = Frame + 1
		# %% 由Marker计算小车的相对位置，同时得到marker图像区域
		CamPosition, MarkerROI = DealMarker(Img)  # CamPosition：(x,y,z)
		# %% 实现交通灯颜色检测
		LightColors = TrafficLight(MarkerROI, Img)  # LightColors：0-'Red', 1-'Yellow', 2-'Green'
		print('Frame:', Frame)
		print(CamPosition, LightColors)     # 输出小车位置和交通灯颜色
		
		Img = cv2.resize(Img, (int(Img.shape[1] / 2), int(Img.shape[0] / 2)))
		cv2.imshow('Video', Img)
		key = cv2.waitKey(5)
		if key != -1:
			exit()
		# if CamPosition is not None:
		# 	Point = ax.scatter(CamPosition[0], CamPosition[1], s=5)
		# 	plt.pause(0.01)
		# 	Point.remove()
	else:
		break
print('End')
Video.release()
