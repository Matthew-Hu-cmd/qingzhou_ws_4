from pickle import REDUCE, TRUE
from PIL.Image import new
import cv2
import numpy as np
# from numpy.ma.core import flatten_structured_array
from ackermann_msgs.msg import AckermannDrive  # 引用阿克曼的消息类型
import rospy
from std_msgs.msg import Bool
import sys
import signal
import multiprocessing as mp


def socket_cb(msg):
    global h_max,h_min,s_max,s_min,v_max,v_min
    global color
    print(msg.data)
    if (msg.data == True):
        print("----------------------GREEN-------------------------")
        color = "g"
        h_min = 35
        h_max = 77
        s_min = 60
        s_max = 255
        v_min = 110
        v_max = 255

def gstreamer_pipeline(
        capture_width=320,
        capture_height=240,
        display_width=320,
        display_height=240,
        framerate=20,
        flip_method=0,):
    return (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1 ! "
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



    
                


def imgRead(imgQueue):
	# %% 从摄像头读取数据
	# cam = cv2.VideoCapture(0)
    cam = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    if not cam.isOpened():
        print("Unable to open camera")
    else:
        print('Open camera success!')
    while True:
        success, img = cam.read()
        # if not ret:
        #     break
        while not imgQueue.empty():
            imgQueue.get()
        # t1 = time.time()


        success, img = cam.read()

        # 红绿灯使用opencv腐蚀层对颜色进行判断，判断绿色
        print("----------------------GREEN-------------------------")
        h_min = 35
        h_max = 77
        s_min = 60
        s_max = 255
        v_min = 110
        v_max = 255


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
        light_img = mask[:100,:200 ]
        cv2.imshow("light",light_img) # 输出红绿灯检测结果

        if success == True:
            # 初始化信息
            # reached = False
            # done = False
            rospy.init_node('light', anonymous=False)
            green_light = rospy.Publisher('/traffic_light',Bool,queue_size=10) # ture检测到绿灯
            # sub_color = rospy.Subscriber('/detector_trafficlight', Bool, socket_cb) # true检测红色，false检测蓝色
            # ark_contrl = AckermannDrive()  # 实例化阿克曼消息
            # sub_reached = rospy.Subscriber('/reached',Bool,reach_cb)
            # done_pub = rospy.Publisher('/done',Bool,queue_size=10)
            signal.signal(signal.SIGINT,quit)
            signal.signal(signal.SIGTERM,quit)


            green_exist = 0
            light_xy = np.column_stack(np.where(light_img == 255))
            light_x = light_xy[:,0]
            exist = np.mean(light_x)
            if np.isnan(exist):
                green_exist = 0
            else: 
                green_exist = len(light_x)
            if (green_exist == 0):
                green_light.publish(False)
                print("stop")
            else:
                green_light.publish(True)
                print("pass")
                
        key = cv2.waitKey(5)
        if key == 27:
            break
    cam.release()




def vision():

	Frame = 0
	ImgQueue = mp.Queue()  # 先进先出队列，实现不同进程数据交互
	a = []
	a.append(mp.Process(target=imgRead, args=(ImgQueue,)))
	[Mp.start() for Mp in a]
	# Mps[0].join()
	while ImgQueue.empty():
		pass
	while True:
                Key = input('Press Q or q to quit:')
                if Key == 'Q' or Key == 'q':
                    break
	[Mp.terminate() for Mp in a]

if __name__ == '__main__':
	vision()
