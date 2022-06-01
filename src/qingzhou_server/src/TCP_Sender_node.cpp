/******************************************
文件名：TCP_Sender_node.cpp
功能：运行ROS节点，负责ROS服务端的建立和控制

作者：0E
******************************************/
#include  "TCP_Sender.h"



int main(int argc, char** argv)
{
	//1.初始化ros节点
	ros::init(argc, argv, "TCP_Sender_node");

	//2.创建节点句柄
	ros::NodeHandle nh("~");

	//3.TCP_Sender类实例化
	TCP_Sender* tcpsender;
	tcpsender = new TCP_Sender();
	ROS_INFO("TCP Create");

	//4.初始化TCP_Sender
	tcpsender->initTCP();
	ROS_INFO("TCP init success!");
	while (ros::ok())
	{
		tcpsender->createLink();
		ROS_INFO("Create Link");
		tcpsender->transMessage();
		break;
	}

	tcpsender->closeSocket();
	//3.返回0
	return 0;
}