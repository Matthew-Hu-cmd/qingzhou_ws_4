#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv_bridge
import rospy
import cv2
import numpy as np
import cv2.aruco as aruco
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
#from _02CalculatePositon import *
#from _03TrafficLight import *

dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)  # 编码点的类型,与生成的时候对应
K = np.array([[228.82, 0, 601.32],
              [0, 228.59, 363.39],
              [0, 0, 1]])
Dis = np.array([0.00490, 0.00105, 0.00012, 0.00047])
EMap = np.loadtxt('/home/ltz/jtd/EMap.txt')
Red = np.array([0, 0, 180.])
Yellow = np.array([10, 100, 140.])
Green = np.array([0, 180, 0.])
Colors = (Red, Yellow, Green)
ColorsName = ('Red', 'Yellow', 'Green')
DistThreshold =  30000   # 颜色距离阈值



class trafficDetector:
    tract = []
    def __init__(self):
        rospy.on_shutdown(self.cleanup);

        # 创建cv_bridge
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)

        # # 获取haar特征的级联表的XML文件，文件路径在launch文件中传入
        # cascade_1 = rospy.get_param("~cascade_1", "")
        # cascade_2 = rospy.get_param("~cascade_2", "")

        # # 使用级联表初始化haar特征检测器
        # self.cascade_1 = cv2.CascadeClassifier(cascade_1)
        # self.cascade_2 = cv2.CascadeClassifier(cascade_2)

        # # 设置级联表的参数，优化人脸识别，可以在launch文件中重新配置
        # self.haar_scaleFactor  = rospy.get_param("~haar_scaleFactor", 1.2)
        # self.haar_minNeighbors = rospy.get_param("~haar_minNeighbors", 2)
        # self.haar_minSize      = rospy.get_param("~haar_minSize", 40)
        # self.haar_maxSize      = rospy.get_param("~haar_maxSize", 60)
        # self.color = (50, 255, 50)






        # 初始化订阅rgb格式图像数据的订阅者，此处图像topic的话题名可以在launch文件中重映射
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)

    def image_callback(self, data):
        global Img
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            Img = self.bridge.imgmsg_to_cv2(data, "bgr8")     
            frame = np.array(Img, dtype=np.uint8)
        except CvBridgeError:
            rospy.loginfo("traffic detector is error")
    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

# ---------02文件------------


# 解算位姿的函数
def CalculatePositon(Point3D, Point2D, K, Dis):
	''''''
	Center = np.mean(Point3D, axis=0)
	Point3D = Point3D - Center  # 去中心化
	ret, RvecW2C, tW2C = cv2.solvePnP(Point3D, Point2D, K, Dis)  # 解算位姿
	RW2C = cv2.Rodrigues(RvecW2C)[0]
	RC2W = np.linalg.inv(RW2C)
	tC2W = -np.linalg.inv(RW2C).dot(tW2C)
	CamPosition = tC2W.flatten() + Center  # 相机在世界坐标系下的坐标

	return CamPosition

def DealMarker(Img):
	CamPosition = None
	MarkerROI = None
	Corners, IDs, rejectedImgPoints = aruco.detectMarkers(Img, dict)
	print(IDs)
	print(Corners)
#	print(type(Corners[0]))
#	if len(Corners[0]) >= 4:  # 如果检测点
	try:
		if (Corners[0].size is 8):
			print("into if")
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
		print(e)

# ---------03文件------------



def JudgeLightColor(Light):
	Dist = np.empty((0,))
	for Color in Colors:
		Dist = np.append(Dist, np.sum(abs(Color - Light) ** 2))
	return np.argmin(Dist), np.min(Dist)


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
		th, MaskImg = cv2.threshold(LightImgGray, 200, 255, cv2.THRESH_TOZERO)
		MaskImg = cv2.morphologyEx(MaskImg, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
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
			if Area > 20 and Area < 1000 and Area / HullArea > 0.9:
				sel_contours.append(contour)
				# 形态学提取外轮廓区域
				MaskImg = np.zeros_like(LightImgGray)
				cv2.drawContours(MaskImg, [contour], -1, 255, cv2.FILLED)
				kernel = np.ones((int(H / 8), int(H / 8)), np.uint8)
				dilation = cv2.dilate(MaskImg, kernel, iterations=1)  # 膨胀
				MaskImg = dilation - MaskImg
				MaskImg = cv2.cvtColor(MaskImg, cv2.COLOR_GRAY2BGR)
				OutSide = LightImg & MaskImg
				Index = np.argwhere(np.sum(OutSide, axis=2) > 0)
				GrayLevel = OutSide[Index[:, 0], Index[:, 1], :]
				Light = np.mean(GrayLevel, axis=0)
				Color, Dist = JudgeLightColor(Light)
				if Dist < DistThreshold:    # 颜色空间L2距离足够小，完成颜色判断
					LightColors.append(Color)



# while 1:
# CamPosition, MarkerROI = DealMarker(Img)  # CamPosition：(x,y,z)
# # %% 实现交通灯颜色检测
# LightColors = TrafficLight(MarkerROI, Img)  # LightColors：0-'Red', 1-'Yellow', 2-'Green'
# print('Frame:', Frame)
# print(CamPosition, LightColors)     # 输出小车位置和交通灯颜色
    
    # if CamPosition is not None:
    # 	Point = ax.scatter(CamPosition[0], CamPosition[1], s=5)
    # 	plt.pause(0.01)
    # 	Point.remove()

        # # 创建灰度图像
        # grey_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # # 创建平衡直方图，减少光线影响
        # grey_image = cv2.equalizeHist(grey_image)

        # # 尝试检测人脸
        # faces_result = self.detect_face(grey_image)

        # # 在opencv的窗口中框出所有人脸区域
        # if len(faces_result)>0:
        #     for face in faces_result: 
        #         x, y, w, h = face
        #         distance = (190.0/float(h))*50.0
        #         cv2.rectangle(cv_image, (x, y), (x+w, y+h), self.color, 2)
        #         cv2.putText(cv_image, "face:"+str(distance)+"cm", (x,y), cv2.FONT_HERSHEY_COMPLEX,1, (0,255,0),2)
        #         rospy.loginfo("distance = %.2fcm",distance)
        #         cv2.circle(cv_image,(x+w/2,y+h/2),10,(0,0,255),-1)

        # 将识别后的图像转换成ROS消息并发布
    #     self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

    # def detect_face(self, input_image):
    #     # 首先匹配正面人脸的模型
    #     if self.cascade_1:
    #         faces = self.cascade_1.detectMultiScale(input_image, 
    #                 self.haar_scaleFactor, 
    #                 self.haar_minNeighbors, 
    #                 cv2.CASCADE_SCALE_IMAGE, 
    #                 (self.haar_minSize, self.haar_maxSize))
                                         
    #     # 如果正面人脸匹配失败，那么就尝试匹配侧面人脸的模型
    #     if len(faces) == 0 and self.cascade_2:
    #         faces = self.cascade_2.detectMultiScale(input_image, 
    #                 self.haar_scaleFactor, 
    #                 self.haar_minNeighbors, 
    #                 cv2.CASCADE_SCALE_IMAGE, 
    #                 (self.haar_minSize, self.haar_maxSize))
    #     return faces

    # def cleanup(self):
    #     print "Shutting down vision node."
    #     cv2.destroyAllWindows()

if __name__ == '__main__':
    #global Img
    # a =  trafficDetector()
    # a.image_sub()



    try:
        tract = []
        # 初始化ros节点
        rospy.init_node("trafficlight_detector")
        rospy.loginfo("traffic detector is started..")
        rospy.loginfo("Please subscribe the ROS image.")
        trafficDetector()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down detector node.")
        cv2.destroyAllWindows()
    CamPosition, MarkerROI = DealMarker(Img)  # CamPosition：(x,y,z)
    # %% 实现交通灯颜色检测
    LightColors = TrafficLight(MarkerROI, Img)  # LightColors：0-'Red', 1-'Yellow', 2-'Green'
    print('Frame:', Frame)
    print(CamPosition, LightColors)     # 输出小车位置和交通灯颜色
    rospy.loginfo("11")