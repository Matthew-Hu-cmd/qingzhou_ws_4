## 一、TCP通信(已弃用)

	1.将进入accept函数后无法CTRL+C的问题解决  
	2.将TCP的阻塞换成非阻塞 
	3.结合Qt(先实现可以从机器人上读取电压等信息的功能)
	|----看底盘串口读取的数据顺序
	|----看bringup.cpp文件里面读取的串口数据
	4.把世奇学长的代码改一下
	5.发现了一种能够在客户端直接和master通信的机制，并且找到了很好的开源代码与课程，应该可以很快实现一个简易的直接发目标点的客户端。结合盲人导航的
	（qingzhou_nav l1_nav_blind.launch）更新应该可以实现一个百分之百不会重启的完整流程

## 二、路径控制&DynamicConfig
	1. add dynamicreconfigure feature inplace of the param server
   	(DONE 5.31 2022)

	2. solve the problem that can't reach the final goal
	|---maybe no extra function is needed to achieve that

	3. speed changed too abrptly while stopping & startting
	|---maybe a good thing 
	|---but can still be changed by 1/(（1/car2goal_dist）* speed)
	***git is shit***
	GIT IS FUCKING TRIFFIC

	4. 它tmd把bringup改了好多啊，加了视觉结果东西进来，目前还不知道是好事还是坏事：
   		（DONE 2022.6.8）
    |----暂时没发现对这个有什么用
	|----没什么事，义哲在修改客户端和上位机的代码了
	5.  有的点跑到了角落就无法规划了（问题不大，最终调参阶段统一解决）
    |\
	| \
	|  \
	|   \
	|————\ 把不能去到的区域用三角形划出来，在Dynamic里面强迫他后退
	倒不如直接用骚方法，绘制地图尽量来避免这种问题

	<!-- 5. **************搞懂它goal生成的路径到底跟最终控制有什么关系****************
   			看看能不能最后直接只发布固定的path，把movebase给直接省略
			   我可以写一个自己的路径规划器，用一种讨巧的方式
			   接收目标点之后，我的规划器分段发布对应的路径 -->
	
	6.ackermann_cmd_filter

## 我直接救命啊，L1去装货区有的时候会冲出去
## 出卸货区的时候有的时候会蹭到（小问题，画地图解决）
## l1里面订阅的是/odom的pose，应该改成/amcl_pose,不过我感觉现在用着也没什么大问题啊

## 三、视觉控制
	1. 把图像传输功能去掉，过于占用资源
	2. 多线程编程
   	|-----参考三院tradtional_ws 提供的方法
	3. 视觉控制最终发布需要能够符合bringup节点的接收
	4. 尝试神经网络方案做车道线识别
	5. 借鉴李世奇的实现在特定的位置才打开识别

## 四、定位
    1. robot_localization
    2. amcl
    3. imu
    4. odom
    5. 调参数

## 