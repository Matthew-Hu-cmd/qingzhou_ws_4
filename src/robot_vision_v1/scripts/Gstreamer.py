# -*- coding:utf-8 _*-
import cv2
# 3264 2464 21  CPU:27%, MEM:56.4
# 3264 1848 28  CPU:33%, MEM:55.4
# 1920 1080 30  CPU:30%, MEM:53.3
# 1280 720 60   CPU:45%, MEM:52.8
# 1280 720 120  CPU:75%, MEM:53.2

def gstreamer_pipeline(
		capture_width=640,
		capture_height=480,
		display_width=640,
		display_height=480,
		framerate=25,
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