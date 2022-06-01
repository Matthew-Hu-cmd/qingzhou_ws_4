## 一、TCP通信

	1.将进入accept函数后无法CTRL+C的问题解决  
	2.将TCP的阻塞换成非阻塞 
	3.结合Qt(先实现可以从机器人上读取电压等信息的功能)
	|----看底盘串口读取的数据顺序
	|----看bringup.cpp文件里面读取的串口数据
	4.把世奇学长的代码改一下

## 二、控制
	1. add dynamicreconfigure feature inplace of the param server(DONE 5.31 2022)

	2. solve the problem that can't reach the final goal
	|---maybe no extra function is needed to achieve that

	3. speed changed too abrptly while stopping & startting
	|---maybe a good thing 
	|---but can still be changed by 1/(（1/car2goal_dist）* speed)
