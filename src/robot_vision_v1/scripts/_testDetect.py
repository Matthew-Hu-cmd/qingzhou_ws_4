#-*- coding:utf-8 _*-
import numpy as np
import cv2
import cv2.aruco as aruco

dict = aruco.getPredefinedDictionary(aruco.DICT_7X7_1000)  # 编码点的类型,与生成的时候对应

markerImage = np.zeros((200, 200), dtype=np.uint8)
for i in range(10):
    markerImage = aruco.drawMarker(dict, i, 200)      

    firename=str(i)+'.png'
    cv2.imwrite(firename, markerImage)
def DealMarker(Img):
	CamPosition = None
	MarkerROI = None
	print("img:")
	print(Img)
	print("dict")
	print(dict)

	Corners, IDs, _ = aruco.detectMarkers(Img, dict)
	print(IDs)
	print(Corners)
	return CamPosition, MarkerROI


for  i  in range(10):
    frame=cv2.imread('test7.png')
    print(DealMarker(frame))