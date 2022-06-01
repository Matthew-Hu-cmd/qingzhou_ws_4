#-*- coding:utf-8 _*-

import numpy as np
import cv2
import cv2.aruco as aruco
import time
dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)  # 编码点的类型,与生成的时候对应
K = np.array([[401.27, 0, 324.47],
              [0, 541.61,  286.99],
              [0, 0, 1]])
Dis = np.array([0.305256, 0.077563, 0.003689, 0.000838])
EMap = np.loadtxt('/home/cquer/ROSApplications/qingzhou_ws_2/src/robot_vision_v1/scripts/EMap.txt')
# MinusY = np.array([[0], [-1], [0.]])  # 相机坐标系Y轴负方向为小车方向
# ZeroDirection = np.array([1,0.])            # 零角度的方向
# 解算位姿的函数
# def CalculatePositon(Point3D, Point2D, K, Dis):
# 	''''''
# 	Center = np.mean(Point3D, axis=0)
# 	Point3D = Point3D - Center  # 去中心化
# 	ret, RvecW2C, tW2C = cv2.solvePnP(Point3D, Point2D, K, Dis)  # 解算位姿
# 	RW2C = cv2.Rodrigues(RvecW2C)[0]
# 	RC2W = np.linalg.inv(RW2C)
# 	tC2W = -np.linalg.inv(RW2C).dot(tW2C)
# 	CamPosition = tC2W.flatten() + Center  # 相机在世界坐标系下的坐标
# 	CamDirection = (cv2.Rodrigues(RvecW2C)[0].dot(MinusY) + t).flatten()[0:2] + Center[0:2] - CamPosition  # 相机在世界坐标系下的方向坐标
# 	CamDirection = CamDirection/np.linalg.norm(CamDirection)
# 	Alpha = np.arccos(CamDirection.dot(ZeroDirection))/np.pi*180
# 	if CamDirection[1]<0:
# 		Alpha = 360-Alpha
# 	return CamPosition, Alpha, CamDirection
def CalculatePositon(Point3D, Point2D, K, Dis):
	''''''
	# Center = np.mean(Point3D, axis=0)
	# Point3D = Point3D - Center  # 去中心化
	# ret, RvecW2C, tW2C = cv2.solvePnP(Point3D, Point2D, K, Dis)  # 解算位姿
	# RW2C = cv2.Rodrigues(RvecW2C)[0]#输入旋转矩阵，输出旋转向量（矩阵）
	# RC2W = np.linalg.inv(RW2C)#矩阵求逆
	# tC2W = -np.linalg.inv(RW2C).dot(tW2C)
	# CamPosition = tC2W.flatten() + Center  # 相机在世界坐标系下的坐标
	Center = np.mean(Point3D, axis=0)
	Point3D = Point3D - Center  # 去中心化
	ret, Rvec, t = cv2.solvePnP(Point3D, Point2D, K, Dis)  # 解算位姿
	CamPosition = t[0:2, 0].flatten() + Center[0:2]  # 相机在世界坐标系下的坐标
	return CamPosition

def DealMarker(Img):
	CamPosition = None
	MarkerROI = None
	# time1 = time.time()
	Corners, IDs, rejectedImgPoints = aruco.detectMarkers(Img, dict)
	# time2 = time.time()
	# print("detectMarkers :{}".format(time2-time1))
	# print(IDs)
	# print(Corners)
#	print(type(Corners[0]))
#	if len(Corners[0]) >= 4:  # 如果检测点
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
		# print "error in dealmarker"
		# print(e)
	return CamPosition, MarkerROI
