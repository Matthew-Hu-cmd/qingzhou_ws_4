#!/usr/bin/env python
# -*- coding:utf-8 _*-
from pickle import LIST
import numpy as np
import cv2
# from jetcam.csi_camera import CSICamera

import matplotlib.pyplot as plt
import rospy
# from std_msgs.msg import String
from std_msgs.msg import String, Float32

from geometry_msgs.msg import Vector3

from _02CalculatePositon import *
from _03TrafficLight import *

def gstreamer_pipeline(
		# capture_width=3264,
		# capture_height=2464,
		capture_width=1280,
		capture_height=720,
		# display_width=640,
		# display_height=480,
		display_width=1920,
		display_height=1080,
		framerate=10,
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


# camera = CSICamera(width=224, height=224, capture_width=1080, capture_height=720, capture_fps=30)
np.set_printoptions(suppress=True, precision=4)

# 载入参数，载入地图
Frame = 1       # 从第1000帧开始读取视频
#Video = cv2.VideoCapture('/home/ltz/test003.avi')
# Video = cv2.VideoCapture('/home/ltz/test004.mp4')
# Video = cv2.VideoCapture(0)
Video = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
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

def detector():
	# while 1:
	while not rospy.is_shutdown():
	# while not rospy.is_shutdown():
		# 逐帧读取
		ret, Img = Video.read()
	#	Img  = cv2.resize(Img,(1920,1080),interpolation=cv2.INTER_AREA)
		if ret == True:
			global Frame
			Frame = Frame + 1
			# %% 由Marker计算小车的相对位置，同时得到marker图像区域
			CamPosition, MarkerROI = DealMarker(Img)  # CamPosition：(x,y,z)
			a =  np.array([1])
			projection_matrix =  np.array([[3.04423340e+02, 0, 3.25529293e+02,0],
                                                                        [0, 4.99492126e+02, 2.95309865e+02,0],
                                                                        [0, 0, 1,0]])
			# CamPosition_1 =  np.append(CamPosition, a, axis=1)
			# CamPosition_T = CamPosition_1.reshape(CamPosition_1.shape[0], 1)
			# pos = np.matmul(projection_matrix, CamPosition_T)
			if CamPosition is not None:
				# CamPosition_1 =  np.hstack(CamPosition,a)
				# CamPosition_T = CamPosition_1.reshape(CamPosition_1.shape[0], 1)
				# pos = np.matmul(projection_matrix, CamPosition_T)
				tmpPosition = Vector3(CamPosition[0],CamPosition[1],0)
			else:
				print "position is none"
				tmpPosition = Vector3(0,0,0)
			# %% 实现交通灯颜色检测
			LightColors = TrafficLight(MarkerROI, Img)  # LightColors：0-'Red', 1-'Yellow', 2-'Green'
			print('Frame:', Frame)
			print(CamPosition, LightColors)     # 输出小车位置和交通灯颜色
			# print(type(LightColors))

			# for i in LightColors:
			# 	print(type(i))
			pub_color= rospy.Publisher('color', Float32, queue_size=10)
			pub_position= rospy.Publisher('position',Vector3, queue_size=10)
			# rate = rospy.Rate(10)
			# while not rospy.is_shutdown():
			if(len(LightColors) > 0):
				print(LightColors)
				colortype = LightColors[0]
				pub_color.publish(colortype)
			else:
				print("havent detected traffic light and do not publish colortype")
				pub_color.publish(-1)
			#colortype = ",".join('%s' %id for id in LightColors)
			
			pub_position.publish(tmpPosition)
					#rate.sleep()
			Img = cv2.resize(Img, (int(Img.shape[1] / 2), int(Img.shape[0] / 2)))
			# cv2.imshow('Video', Img)
			# key = cv2.waitKey(5)
			# if key != -1:
			# 	rospy.loginfo(" try to exit")
				# exit()
			# if CamPosition is not None:
			# 	Point = ax.scatter(CamPosition[0], CamPosition[1], s=5)
			# 	plt.pause(0.01)
			# 	Point.remove()
		else:
			break
if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("traffic_detector")
        rospy.loginfo("traffic_detector node is started...")
        detector()
        rospy.spin()
    except rospy.ROSInterruptException:
		print('End')
		Video.release()

