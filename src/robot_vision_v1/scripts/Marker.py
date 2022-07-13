# -*- coding: utf-8 -*-
import numpy as np
import cv2
import cv2.aruco as aruco

dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)  # 编码点的类型,与生成的时候对应
K = np.array([[401.27, 0, 324.47],
              [0, 541.61,  286.99],
              [0, 0, 1]])
Dis = np.array([0.305256, 0.077563, 0.003689, 0.000838])
EMap = np.loadtxt('/home/cquer/qingzhou_ws_4/src/robot_vision_v1/scripts/EMap.txt')

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
		# print('error')
	return CamPosition, MarkerROI