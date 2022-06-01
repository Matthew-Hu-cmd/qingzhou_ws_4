# -*- coding:utf-8 _*-
import cv2
import numpy as np
import time


def gstreamer_pipeline(
        capture_width=3264,
        capture_height=2464,
        display_width=640,
        display_height=480,
        framerate=21,
        flip_method=0,
):
    return (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! "
            "appsink"
            % (
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
           )
def empty(a):
    pass


path = cv2.VideoCapture(0)#gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER
cv2.namedWindow("TrackBars")  # 创建跟踪栏，帮助查找合适的取值范围
cv2.resizeWindow("TrackBars", 640, 240)  # 调整跟踪栏窗口大小
# 创建轨迹栏，定义名称 导入跟踪栏 初始值 最大值 调用emptu函数反复执行
cv2.createTrackbar("Hue Min", "TrackBars", 0, 255, empty)  # 范围
cv2.createTrackbar("Hue Max", "TrackBars", 33, 255, empty)
cv2.createTrackbar("Sat Min", "TrackBars", 60, 255, empty)  # 饱和度最小值
cv2.createTrackbar("Sat Max", "TrackBars", 189, 255, empty)  # 饱和度最大值
cv2.createTrackbar("Val Min", "TrackBars", 105, 255, empty)  # 数值
cv2.createTrackbar("Val Max", "TrackBars", 255, 255, empty)  #
# t = time.time.now()
w = 320
h = 90



while True:
    for i in range(1):
        success, img = path.read()
    img = cv2.resize(img, (640, 480), interpolation=cv2.INTER_AREA)
    img = cv2.GaussianBlur(img, (5, 5), 0)
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # 将BGR图像转为HSV

    h_min = cv2.getTrackbarPos("Hue Min", "TrackBars")  #
    h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
    s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
    s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
    v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
    v_max = cv2.getTrackbarPos("Val Max", "TrackBars")
    print(h_min, h_max, s_min, s_max, v_min, v_max)  # 打印六个值

    img = cv2.resize(img, (320, 240), interpolation=cv2.INTER_AREA)# 将图像压缩
    img = cv2.GaussianBlur(img, (5, 5), 0)
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # 将BGR图像转为HSV
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(imgHSV, lower, upper)  # 创建蒙版 指定颜色上下限 范围内颜色显示 否则过滤
    kernel_width = 4  # 调试得到的合适的膨胀腐蚀核大小
    kernel_height = 4  # 调试得到的合适的膨胀腐蚀核大小
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_width, kernel_height))
    mask = cv2.erode(mask, kernel)
    mask = cv2.dilate(mask, kernel)
    mask = cv2.dilate(mask, kernel)
    # line_img = mask[120:220,0:220]
    # light_img = mask[:100,:200 ]
    cv2.imshow("raw",img)
    # cv2.imshow("line",line_img)
    # cv2.imshow("light",light_img)
    cv2.imshow("mask",mask)


    # # imgResult = cv2.bitwise_and(img, img, mask=mask)  # 将原始图像和蒙版图像按位与
    # # cv2.line(img, (160, 0), (160, 90), (255, 0, 0), 2)
    # cv2.imshow("raw Images",img)  # 显示最终的堆栈img图像
    # cv2.imshow("mask Images", mask)  # 显示最终的堆栈图像
    # #cv2.imshow("check_img", check_img)  # 显示最终的堆栈图像


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
