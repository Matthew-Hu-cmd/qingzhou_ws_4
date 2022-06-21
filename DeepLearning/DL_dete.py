# /usr/lib/python3.6
#-*- coding:utf-8 _*-
import torch, os, cv2
# from torch.utils.data import Dataset, DataLoader
from torchvision.transforms import transforms
import matplotlib.pyplot as plt
from PIL import Image
import numpy as np
# import random
import rospy
# import math
# from Timer import *
# from _02PipeDatasetLoader import *
from _03Unet1 import *
import multiprocessing as mp
from cv_bridge import CvBridge,CvBridgeError #ROS和cv通道
#from sensor_msgs.msg import CompressedImage  #压缩图像信息
from ackermann_msgs.msg import AckermannDrive
#from ackermann_msgs.msg import AckermannDriveStamped #引用阿克曼的消息类型
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
Unet.load_state_dict(torch.load(os.path.join('270nice.pt'), map_location=Device))  # 我将权重作为素材，提升预测的效果
Unet.eval()  #验证模式
torch.set_grad_enabled(False)  # 将梯度除外
InputImgSize = (128, 128)
rospy.init_node('linetrack',anonymous=True)
cvBridge=CvBridge()
ark_contrl= AckermannDrive() #实例化阿克曼消息
ValImgTransform = transforms.Compose([
transforms.Resize(InputImgSize),
transforms.ToTensor(),
transforms.Normalize(mean=[0.46], std=[0.10]),])  # 把数据作为素材送去变形，全部变为tensor



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


        ret, Img = cam.read()
        Img = cv2.resize(Img,(240,180),interpolation=cv2.INTER_AREA) #将opencv读取的图片resize来提高帧率
        Img1 = Img
        Img = cv2.cvtColor(Img, cv2.COLOR_BGR2RGB)
        Img2 = Img
        cropped2 = Img2[70:128, 0:100]
        h,w,d = cropped2.shape #提取图像的信息
        if ret == True:
            Img = Image.fromarray(Img)  
            Img = ValImgTransform(Img)  # 连锁其它变形，变为tesor
            Img = torch.unsqueeze(Img, dim=0)  # 对tesor进行升维
            inputImg = Img.float().to(Device)  # 让数据能够使用
            OutputImg = Unet(inputImg)
            Output = OutputImg.cpu().numpy()[0]
            print('inpormation',Output) # 分割的信息
            OutputImg = OutputImg.cpu().numpy()[0, 0]
            OutputImg = (OutputImg * 255).astype(np.uint8)
            Input = Img.numpy()[0][0]
            Input = (Normalization(Input) * 255).astype(np.uint8)
            OutputImg = cv2.resize(OutputImg,(128,128),interpolation=cv2.INTER_AREA) # 将opencv读取的图片resize来提高帧率
            ResultImg = cv2.cvtColor(Input, cv2.COLOR_GRAY2RGB)
            ResultImg[..., 1] = OutputImg
            cropped = ResultImg[70:128, 0:100]
            cropped1 = OutputImg[70:128, 0:100]


            # M = cv2.moments(cropped1,False) #找出输出图像的中心矩，也就是车道线的中心
            # cx , cy = int( M['m10'] / M['m00'] ) , int( M['m01'] / M['m00'])
            # print (cx , cy)



            # if (cx == w/2):
            #         angle = 0
            # else:  
            #         angle = np.arctan(int(cx - w/2)/int(h - cy))
            #         print('angle:',angle)
            #         if angle<0:
            #             angle = angle + 30
            #         else:
            #             angle = angle - 20
            # angle = 0.7 * angle + 0.3 * old_angle
            # old_angle = angle
            # ark_contrl.steering_angle = angle
            # ark_contrl.speed = 0.4
            # cmd_vel_pub=rospy.Publisher('/ackermann_cmd',AckermannDrive,queue_size=10)
            # cmd_vel_pub.publish(ark_contrl)

        # if ret == False:
        #     print ('111')
        #     error_check += 1
        #     if error_check > max_error_check:
        #         ark_contrl.steering_angle = 0.0
        #         ark_contrl.speed = 0.0
        #         cmd_vel_pub=rospy.Publisher('/ackermann_cmd',AckermannDrive,queue_size=10)
        #         cmd_vel_pub.publish(ark_contrl)
        #         error_check = 0
        #         # done_pub.publish(True)
        #         print("done")


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

        if (np.isnan(line_x) or np.isnan(line_y)): 
            print ('111')          
            # error_check += 1
            
            for error_check in range (max_error_check):
                error_check += 1
                print (error_check)

            if error_check == max_error_check:
                print ('222')
                ark_contrl.steering_angle = 0.0
                ark_contrl.speed = 0.0
                cmd_vel_pub=rospy.Publisher('/ackermann_cmd',AckermannDrive,queue_size=10)
                cmd_vel_pub.publish(ark_contrl)
                error_check = 0
                print("done")
                    

            
        # cmd_vel_pub=rospy.Publisher('/ackermann_cmd',AckermannDrive,queue_size=10) #queue_size其实是一个存放图像的buffer
        # cmd_vel_pub.publish(ark_contrl)

        cv2.imshow("out", cropped1)
        cv2.imshow("Img2", Img2)
        cv2.imshow("Img0", cropped)

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
