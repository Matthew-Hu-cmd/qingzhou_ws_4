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
#include <std_msgs/String.h>

#define SERVER_PORT 6666

class TCP_Sender
{
private:
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

	ros::NodeHandle nh;

public:
	TCP_Sender();
	~TCP_Sender();
	void initTCP();
	void createLink();
	void transMessage();
	void closeSocket();
};


#endif