### 定位

1. 如果发现标定完轮式编码器后，rviz中小车行驶中tf的坐标基本没有变化，可能是由于qingzhou_bringup.cpp文件中编码器读取数据的正负处理有问题，可以尝试修改   
'''cpp   
encoderLeft = -encoderLeft;   
encoderRight = -encoderRight;   
'''   
来解决。


2. 标定imu后无法正常使用，rviz中小车转弯时tf的方向也没变化，可能是qingzhou_bringup.launch中的robot_pose_ekf节点中的参数imu_used为false，改为true可能可以使用，也有可能是stm32程序的问题。


3. 




修改：
6.1
1.将demo.py脚本中的angle计算中数据类型int改为float，由于使用int计算时，角度小于45度结果都为0，并不能很好的跑车道线，修改后可以跑完车道线。


6.2
1.将qingzhou_bringup.cpp第108行printf注释，看着很多。

