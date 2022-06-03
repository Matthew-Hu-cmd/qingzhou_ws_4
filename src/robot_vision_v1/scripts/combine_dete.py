import cv2
from all_function import *

# #####################提取ROI#############################
def region_of_interest(r_image):
    h = r_image.shape[0] #图片的高
    w = r_image.shape[1] #图片的宽
    gray = cv2.cvtColor(r_image, cv2.COLOR_BGR2GRAY) #转化为灰度图

    poly = np.array([
        [(0, h-90), (w, h-90), (w, h), (0, h)]  #将数组转化为矩阵，四个为长方形的四个顶点，从左上角开始顺时针,最下面竖着100个像素的图 #原来是100
    ])
    
    mask = np.zeros_like(gray) #输入为矩阵gray，输出为形状和gray一致的矩阵，其元素全部为黑色0
    cv2.fillPoly(mask, poly, 255)   #将mask的poly部分填充为白色255
    masked_image = cv2.bitwise_and(r_image,r_image, mask=mask)  #将r_image的mask区域提取出来给masked_image
    return masked_image

# ###############################蓝色分割############################
def color_seperate(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   #对目标图像进行色彩空间转换
    lower_hsv = np.array([100, 43, 46])          #设定蓝色下限
    upper_hsv = np.array([124, 160, 160])        #设定蓝色上限
    mask = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)  #依据设定的上下限对目标图像进行二值化转换，低于lower,高于upper都变成0，在中间为255
    dst = cv2.bitwise_and(image, image, mask=mask)    #将image的mask区域提取出来给dst,即找到蓝色区域并赋值给dst
    # cv2.imshow('blue', dst)  #查看蓝色区域的图像
    # cv2.waitKey(3)
    return dst

# ############################红色分割##############################
def color_seperate_1(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   #对目标图像进行色彩空间转换
    # lower_hsv = np.array([156, 43, 46])          #设定红色下限
    # upper_hsv = np.array([180, 255, 255])        #设定红色上限
    lower_hsv = np.array([0, 43, 46])          #设定红色下限
    upper_hsv = np.array([10, 255, 255])        #设定红色上限
    mask = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)  #依据设定的上下限对目标图像进行二值化转换，低于lower,高于upper都变成0，在中间为255
    dst = cv2.bitwise_and(image, image, mask=mask)    #将image的mask区域提取出来给dst,即找到红色区域并赋值给dst
    # cv2.imshow('red', dst)  #查看红色区域的图像
    # # cv2.waitKey(3)
    return dst