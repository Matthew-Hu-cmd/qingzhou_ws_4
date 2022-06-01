### 定位

1. 如果发现标定完轮式编码器后，rviz中小车行驶中tf的坐标基本没有变化，可能是由于qingzhou_bringup.cpp文件中编码器读取数据的正负处理有问题，可以尝试修改   
`encoderLeft = -encoderLeft;   

encoderRight = -encoderRight;`  
来解决。


2. 标定imu后无法正常使用，rviz中小车转弯时tf的方向也没
3. 