
from new_dete import ROI_Img, ROI_Img, gstreamer_pipeline, MyThread, checkColor, getShift
import cv2
import numpy as np
import matplotlib.pyplot as plt

np.set_printoptions(suppress=True, precision=4)

if __name__ == '__main__':
    Video = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

    try:
        while Video.isOpened():
            ret, img = Video.read()
            if not ret:
                print('img is none')
                break
            else:
                thread1 = MyThread(checkColor, (img,))
                thread1.start()
                pre_Img = ROI_Img(img, 0, 90)
                color = thread1.get_result()
                shift = getShift(color, pre_Img)
                # histogram = np.sum(b_img[:, :], axis=0)
                # midpoint = int(histogram.shape[0] / 2)
                # leftx_base = np.argmax(histogram[:midpoint]) + 50
                # final_img = ROI_Img(b_img, b_img.shape[1] - leftx_base, 90)
                # contours, hierarchy = cv2.findContours(final_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                # # contours, hierarchy = cv2.findContours(final_img, 1, 2)
                # if len(contours) > 0:
                #     M = cv2.moments(contours[0])
                #     cx = int(M['m10']/(M['m00']+float("1e-8")))
                #     cy = int(M['m01']/(M['m00']+float("1e-8")))
                #     if cx == 0 and cy == 0:
                #         pass
                #     else:
                #         # print(cx, cy)
                #         cv2.circle(img, (cx, cy), 10, (0, 0, 255))
                #         shift = (cx-(img.shape[1]/2))*105/img.shape[1]
                # angle = (shift*1.4 + 3.489) /180.0*3.1415926
                #         print(angle)
                # out_img = cv2.resize(img, (320, 240), interpolation=cv2.INTER_AREA)
                print(shift.__format__)
                # angle = (shift*1.4 + 3.489) /180.0*3.1415926
                # print(angle)
                cv2.imshow('pre', img)
                if cv2.waitKey(25) == ord('q'):
                    break

    finally:
        print('End')
        Video.release() 