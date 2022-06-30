#!/usr/lib/python3.6
# -*- coding: utf-8 -*-

from robot_vision_v1.scripts.final_dete import ROI_Img, ROI_Img, gstreamer_pipeline, getShift
import cv2
import numpy as np

if __name__ == '__main__':
    out_check = 0
    Video = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

    try:
        while Video.isOpened():
            ret, img = Video.read()
            if not ret:
                print('img is none')
                break
            else:
                pre_Img = ROI_Img(img, 0, 90)
                shift = getShift(pre_Img)
                # out_img = cv2.resize(img, (320, 240), interpolation=cv2.INTER_AREA)
                if shift:
                    angle = shift
                else:
                    out_check += 1 
                    if out_check > 5:
                        print('stop')
                        out_check = 0

                # cv2.imshow('pre', img)
                # if cv2.waitKey(25) == ord('q'):
                #     break

    finally:
        print('End')
        Video.release() 