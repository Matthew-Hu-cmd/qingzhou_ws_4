### Jetson nano频繁重启的问题
经过无数次的更改方案都无法有效解决  
但是我们发现了一个100%确定的规律：  
	不跑视觉的时候就不会有重启的问题  
	那么我们就以此为分支，制定一个全新的解决方案  
	--> 取消视觉也能完成整个流程：  
		1. 红绿灯：在客户端上面写两个不一样按钮，人的肉眼是知道现在是红灯还是绿灯的，红灯对应的按钮分两段发点、绿灯对应的直接发点  
		2. S弯的话，在地图层动动手脚，已经实现了盲人导航了  

### 控制

1. 和下位机通行部分在qingzhou_bringup.cpp文件里面
2. 

### 定位

1. 如果发现标定完轮式编码器后，rviz中小车行驶中tf的坐标基本没有变化，可能是由于qingzhou_bringup.cpp文件中编码器读取数据的正负处理有问题，可以尝试修改   
`encoderLeft = -encoderLeft;`  
`encoderRight = -encoderRight;`   
来解决。

2. 标定imu后无法正常使用，rviz中小车转弯时tf的方向也没变化，可能是qingzhou_bringup.launch中的robot_pose_ekf节点中的参数imu_used为false，改为true可能可以使用，也有可能是stm32程序的问题。


3. imu_filter_madgwick  
http://wiki.ros.org/imu_filter_madgwick  
- 3.1 parameters  

    >world_frame (string, default: "nwu" for Indigo - Kinetic, "enu" for Lunar+)  
    >>The world frame with respect to which the orientation is indicated. For historic reasons, the old default is "nwu" (North-West-Up). New deployments should use "enu". Valid values: "nwu", "enu", "ned". 

    >use_mag (bool, default: true)  
    >>Whether to use the magnetic field data in the data fusion.  

    >use_magnetic_field_msg (bool, default: false for Hydro and Indigo, true for Jade+)  
    >>If set to true, subscribe to the /imu/mag topic as a sensor_msgs/MagneticField; if set to false (deprecated), use geometry_msgs/Vector3Stamped.  

    >fixed_frame (string, default: odom)
    >>The parent frame to be used in publish_tf.

    >publish_tf (bool, default: true)
    >>Whether to publish a TF transform that represents the orientation of the IMU, using the frame specified in fixed_frame as the parent frame and the frame given in the input imu message as the child frame.

    >reverse_tf (bool, default: false)
    >>If set to true, publish transforms from imu_frame to fixed frame instead of the other way around.

	>constant_dt (double, default: 0.0)
    >>The dt to use; if 0.0 (default), compute dt dynamically from message headers.

    >publish_debug_topics (bool, default: false)
    >>If set to true, publish a couple of debug topics.

	>stateless (bool, default: false)
    >>If set to true, don't publish a filtered orientation. Instead, publish the stateless estimate of the orientation based on the latest accelerometer (and optionally magnetometer) readings alone. Useful for debugging.

	>remove_gravity_vector (bool, default: false)
    >>If set to true, subtract the gravity vector from the acceleration field in the published IMU message.

4. robot_pose_ekf  
http://wiki.ros.org/robot_pose_ekf  
- 4.1 Introduction  
robot_pose_ekf只适用于平面上的轮式移动机器人，因此odom信息中的z，pitch和roll分量可以被忽略。IMU可以提供车体坐标系相对于世界坐标系的姿态(RPY角)，其中Roll和Pitch是绝对角度，因为有重力方向作为参考，而偏航角Yaw则是一个相对角度(如果IMU中没有集成电子罗盘测量地球磁场角作为参考)。IMU姿态的协方差矩阵代表了姿态测量的不确定度。  
- 4.2 parameters  

    >odom_used (bool, default: true)
	>>是否启用轮式里程计

	>imu_used (bool, default: true)
	>>是否启用惯性里程计

	>vo_used (bool, default: true)
	>>是否启用视觉里程计

	>base_footprint_frame (string, default: odom)
	>>机器人基座标系

	>freq (double, default: 不知道)
	>>EKF数据融合的频率（受限于传感器的频率）

	>sensor_timeout (double, default: 不知道)
	>>传感器消息超时时间，传感器停止向滤波器发送信息之后等待多久接收下一个传感器信息  

- 4.3 limitations  
没有robot_localization好用，robot_localization更完善  

5. robot_localization


修改：  
6.1
1.将demo.py脚本中的angle计算中数据类型int改为float，由于使用int计算时，角度小于45度结果都为0，并不能很好的跑车道线，修改后可以跑完车道线。


6.2
1.将qingzhou_bringup.cpp第108行printf注释，看着很多。

6.5 
我在地图层的S弯道处画了很多无伤大雅的点，把S弯道的位置给标记出来，由于l1_controller只关心全局路径，而定位问题由于官方的amcl已经十分强大了，这些点对粒子滤波的定位结果没有任何影响，从而实现了不要视觉就能跑S弯的功能

