# -*- coding:utf-8 _*-

from glob import iglob
import numpy as np
np.set_printoptions(suppress=True, precision=4)
import cv2, time, socket, json, string
import multiprocessing as mp


import cv2


def gstreamer_pipeline(
		capture_width=1920,
		capture_height=1080,
		display_width=480,
		display_height=270,#can't be so big
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






def ImgRead(ImgQueue):
	# %% 从摄像头读取数据
	# cam = cv2.VideoCapture(0)
	cam = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
	if not cam.isOpened():
		print("Unable to open camera")
	else:
		print('Open camera success!')
		while True:
			ret, Img = cam.read()
			if not ret:
				break
			while not ImgQueue.empty():
				ImgQueue.get()
			ImgQueue.put(Img)
			cv2.imshow('ImgRead', Img)
			key = cv2.waitKey(5)
			if key == 27:
				break
		cam.release()




def vision():

	Frame = 0
	ImgQueue = mp.Queue()  # 先进先出队列，实现不同进程数据交互
	Mps = []
	Mps.append(mp.Process(target=ImgRead, args=(ImgQueue,)))
	[Mp.start() for Mp in Mps]
	# Mps[0].join()
	while ImgQueue.empty():
		pass
	while True:
		Key = input('Press s or S to save image:')
		if Key == 's' or Key == 'S':
			Img = ImgQueue.get()
			cv2.imwrite('%04d.jpg' % Frame, Img)
			print('Save image %04d.jpg success!' % Frame)
			Frame = Frame + 1
		elif Key == 'Q' or Key == 'q':
			break
	[Mp.terminate() for Mp in Mps]

if __name__ == '__main__':
	vision()
