#ifndef __TCP_SENDER_H__
#define __TCP_SENDER_H__

#include <ros/ros.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#define SERVER_PORT 6666

class TCP_Sender
{
private:
	//********************TCP通信部分********************//
	//调用socket函数返回的文件描述符
	int serverSocket;
	int client;
	int addr_len;
	int data_len;
	char recvBuff[512];
	char sendBuff[512];
	//声明两个套接字sockaddr_in结构变量，分别表示客户端和服务器
	struct sockaddr_in serverAddr;
	struct sockaddr_in clientAddr;

	//********************ROS通信部分********************//
	ros::NodeHandle nh;
	ros::Subscriber bettarySub;
    ros::Subscriber speedSub;

public:
	//********************TCP通信部分********************//
	TCP_Sender(const ros::NodeHandle &nh);
	~TCP_Sender();
	void initTCP();
	void createLink();
	void transMessage();
	void closeSocket();

	//********************ROS通信部分********************//
	void SubMsgTopic();
	void SubMsgServer();
	
	//**********ROS话题服务的回调函数部分**********//
	void SubBettaryInfoCB();
    void SubSpeedCB(const geometry_msgs::Twist::ConstPtr &msg);

};


#endif