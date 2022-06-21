# /usr/lib/python3.6
#-*- coding:utf-8 _*-
# from turtle import color
import torch, os, cv2
from torchvision.transforms import transforms
import matplotlib.pyplot as plt
from PIL import Image
import numpy as np
# import random
import rospy
from Unet import *
import multiprocessing as mp
from cv_bridge import CvBridge,CvBridgeError #ROS和cv通道
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Bool
import signal

def gstreamer_pipeline(
		capture_width=640,
		capture_height=480,
		display_width=640,
		display_height=480,
		framerate=30,
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


def Normalization(Array):  # 数组归一化到0~1
    min = np.min(Array)
    max = np.max(Array)
    if max - min == 0:
        return Array
    else:
        return (Array - min) / (max - min)


Device = torch.device("cuda:0")  # GPU加速
Unet = UNet(in_channels=3, out_channels=1, init_features=4, WithActivateLast=True, ActivateFunLast=torch.sigmoid).to(
Device)  
Unet.load_state_dict(torch.load(os.path.join('270nice.pt'), map_location=Device))  # 将权重作为素材，提升预测的效果
Unet.eval()  #验证模式
torch.set_grad_enabled(False)  # 将梯度除外
InputImgSize = (128, 128)
rospy.init_node('dete',anonymous=True)
cvBridge=CvBridge()
ark_contrl= AckermannDrive() #实例化阿克曼消息
ValImgTransform = transforms.Compose([
transforms.Resize(InputImgSize),
transforms.ToTensor(),
transforms.Normalize(mean=[0.46], std=[0.10]),])  # 把数据作为素材送去变形，全部变为tensor


reached = False
done = False
color = True

def reach_cb(msg):
    global reached 
    reached = msg.data

def socket_cb(msg):
    global color 
    color = msg.data


def imgRead(imgQueue):
	# %% 从摄像头读取数据
	# cam = cv2.VideoCapture(0)
    cam = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    if not cam.isOpened():
        print("Unable to open camera")
    else:
        print('Open camera success!')
    while True:
        ret, Img = cam.read()
        # if not ret:
        #     break
        while not imgQueue.empty():
            imgQueue.get()
        # t1 = time.time()
        
        h_min = 35
        h_max = 77
        s_min = 60
        s_max = 255
        v_min = 110
        v_max = 255
        

        success, Img = cam.read()

        if success == True:
            Img = cv2.resize(Img,(240,180),interpolation=cv2.INTER_AREA) #将opencv读取的图片resize来提高帧率


            img = cv2.GaussianBlur(Img, (5, 5), 0)
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

            sub_reached = rospy.Subscriber('/reached',Bool,reach_cb)
            done_pub = rospy.Publisher('/done',Bool,queue_size=10)
            sub_color = rospy.Subscriber('/detector_trafficlight', Bool, socket_cb) # true检测红色，false检测蓝色
            

            Img1 = Img
            Img = cv2.cvtColor(Img, cv2.COLOR_BGR2RGB)
            Img2 = Img
            cropped2 = Img2[70:128, 0:100]
            h,w,d = cropped2.shape #提取图像的信息

            Img = Image.fromarray(Img)  
            Img = ValImgTransform(Img)  # 连锁其它变形，变为tesor
            Img = torch.unsqueeze(Img, dim=0)  # 对tesor进行升维
            inputImg = Img.float().to(Device)  # 让数据能够使用
            OutputImg = Unet(inputImg)
            Output = OutputImg.cpu().numpy()[0]
            # print('inpormation',Output) # 分割的信息
            OutputImg = OutputImg.cpu().numpy()[0, 0]
            OutputImg = (OutputImg * 255).astype(np.uint8)
            Input = Img.numpy()[0][0]
            Input = (Normalization(Input) * 255).astype(np.uint8)
            OutputImg = cv2.resize(OutputImg,(128,128),interpolation=cv2.INTER_AREA) # 将opencv读取的图片resize来提高帧率
            ResultImg = cv2.cvtColor(Input, cv2.COLOR_GRAY2RGB)
            ResultImg[..., 1] = OutputImg
            cropped = ResultImg[70:128, 0:100]
            cropped1 = OutputImg[70:128, 0:100]
            cv2.imshow("out", cropped1)
            cv2.imshow("Img2", Img2)
            cv2.imshow("Img0", cropped)

        
        print(reached)

        if reached == True:

            
            done_pub.publish(False)
            line_xy = np.column_stack(np.where(cropped1 >= 245))
            line_x = np.mean(line_xy[:,0])
            line_y = np.mean(line_xy[:,1])
            # if np.isnan(check_x) and np.isnan(check_y):
            center_x = line_x + 70
            center_y = line_y #计算中点坐标
            old_angle = 0
            error_check = 0
            max_error_check = 5 

            if np.isnan(center_x) or np.isnan(center_y):
                angle = 0
            else:
                angle = np.degrees(np.arctan(int(60-center_y)/int(center_x - 43)))
                if angle < 0:
                    angle = angle - 10
                else:
                    angle = angle + 15
            print(line_x,line_y)

            print(angle)
            angle = 0.7 * angle + 0.3 * old_angle
            old_angle = angle
            ark_contrl.steering_angle = angle
            ark_contrl.speed = 0.55
            cmd_vel_pub=rospy.Publisher('/ackermann_cmd',AckermannDrive,queue_size=10)
            cmd_vel_pub.publish(ark_contrl)

            if (np.isnan(line_x) or np.isnan(line_y)) and reached: 
                print ('111')          
                # error_check += 1
                
                # for error_check in range (max_error_check):
                #     error_check += 1
                #     print (error_check)

                while True:
                    error_check += 1
                    print(error_check)
                    if error_check == max_error_check:
                        print ('222')
                        ark_contrl.steering_angle = 0.0
                        ark_contrl.speed = 0.0
                        cmd_vel_pub = rospy.Publisher('/ackermann_cmd',AckermannDrive,queue_size=10)
                        cmd_vel_pub.publish(ark_contrl)
                        # r = rospy.Rate(1/2.0)
                        # r.sleep()
                        done_pub.publish(True)
                        error_check = 0
                        print("done")
                        break
        

        if reached == False and color == True:
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

                    

            
        # cmd_vel_pub=rospy.Publisher('/ackermann_cmd',AckermannDrive,queue_size=10) #queue_size其实是一个存放图像的buffer
        # cmd_vel_pub.publish(ark_contrl)

        

        key = cv2.waitKey(5)
        if key == 27:
            break
    cam.release()


def vision():
	imgQueue = mp.Queue()  # 先进先出队列，实现不同进程数据交互
	Mps = []
	Mps.append(mp.Process(target=imgRead, args=(imgQueue,)))
	[Mp.start() for Mp in Mps]
	# Mps[0].join()
	while imgQueue.empty():
		pass
	while True:		
		Key = input('Press Q or q to quit:')
		if Key == 'Q' or Key == 'q':
			break
	[Mp.terminate() for Mp in Mps]
#torch.multiprocessing.set_start_method('spawn')	



if __name__ == '__main__':
        torch.multiprocessing.set_start_method('spawn')	
        vision()
