#! /usr/bin/env python
#-*-coding:utf-8-*-
import cv2
import numpy as np

def gstreamer_pipeline(
		# capture_width=3264, #原来的
		# capture_height=2464,
		capture_width=640,
		capture_height=480,
		display_width=640,
		display_height=480,
		# display_width=1920, #原来的
		# display_height=1080,
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
			"video/x-raw, format=(string)BGR ! appsink drop=1"
			% (
				capture_width,
				capture_height,
				framerate,
				flip_method,
				display_width,
				display_height,
			)
	)

if __name__ == "__main__":
	Video = cv2.VideoCapture(0)
	while Video.isOpened():
			ret,img = Video.read()
			if ret:
				cv2.imshow('a',img)
				if cv2.waitKey(25) == ord('q'):
					break
			else:
				break

Video.release()
cv2.destroyAllWindows()