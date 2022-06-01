/******************************************
文件名：TCP_Sender.cpp
功能：TCP_Sender类的实现

作者：0E
******************************************/

#include "TCP_Sender.h"


/********************************************
TCP_Sender类的构造函数
功能：初始化Socket相关的配置
********************************************/
TCP_Sender::TCP_Sender()
{
	addr_len = sizeof(clientAddr);
	
}

TCP_Sender::~TCP_Sender()
{

}

void TCP_Sender::initTCP()
{
	//创建套接字，参数：地址类型IPV4，套接字类型TCP，传输协议（一般为0）
	if ((serverSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		ROS_ERROR("Socket Error!");
		ROS_ERROR_STREAM(strerror(errno));
		exit(1);
	}
	else
	{
		ROS_INFO("Create Socket!");
	}
	//初始化服务端的套接字，并用htons和htonl将端口和地址转换成网络字节
	bzero(&serverAddr, sizeof(serverAddr));
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(SERVER_PORT);
	//IP可以是本服务器的IP，也可以用宏INADDR_ANY代替，代表0.0.0.0，表明所有地址
	serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	//bind参数：服务器端的套接字文件描述符
	if (bind(serverSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) != 0)
	{
		ROS_ERROR("Bind Error!");
		ROS_ERROR_STREAM(strerror(errno));
		exit(1);
	}
	else
	{
		ROS_INFO("Bind Success!");
	}
	//设置服务器上的socket为监听状态，第2个参数为排队最大连接个数
	if (listen(serverSocket, 2) != 0)
	{
		ROS_ERROR("Listen Error!");
		ROS_ERROR_STREAM(strerror(errno));
		exit(1);
	}
	else
	{
		ROS_INFO("Port:%d. Listening...", SERVER_PORT);
	}
}


/********************************************
函数名：createLink
功能：调用accept函数来和客户端建立连接
********************************************/
void TCP_Sender::createLink()
{
	//调用accept函数后，会进入阻塞状态
	//accpet返回一个套接字的文件描述符
	//serverSocket负责继续监听，client负责接收和发送数据
	//clientAddr是一个传出参数，accept返回时，传出客户端的地址和端口号
	//addr_len是一个传入-传出参数，传入的是调用者提供的缓冲区clientAddr的长度，以免缓冲区溢出，传出的是客户端地址结构体的实际长度
	if ((client = accept(serverSocket, (struct sockaddr *)&clientAddr, (socklen_t*)&addr_len)) != 0)
	{
		ROS_ERROR("accept failed!");
		ROS_ERROR_STREAM(strerror(errno));
		return;
	}
	else
	{
		ROS_INFO("Waiting for messages...");
	}
	ROS_INFO("IP is %s", inet_ntoa(clientAddr.sin_addr));
	ROS_INFO("Port is %d", htons(clientAddr.sin_port));
}


/********************************************
函数名：transMessage
功能：接收客户端信息，向客户端发送信息
********************************************/
void TCP_Sender::transMessage()
{
	while (1)
	{
		ROS_INFO("Read message");
		recvBuff[0] = '\0';
		//recv函数返回接收到信息的长度
		//参数：客户端套接字，数据缓冲区，数据最大长度，操作（一般0）
		//收到后将数据显示出来，再向客户端发送数据
		if ((data_len = recv(client, recvBuff, 1024, 0)) < 0)
		{
			ROS_ERROR("Recieve None");
			ROS_ERROR_STREAM(strerror(errno));
			continue;
		}
		else
		{
			recvBuff[data_len] = '\0';
			ROS_INFO_STREAM("RRecieve: " << recvBuff);
		}
		
		if (strcmp(recvBuff, "quit") == 0)
		{
			break;
		}

		ROS_INFO("Send message");
		scanf("%s", sendBuff);
		//send函数向客户端发送消息，参数同recv函数
		if ((send(client, sendBuff, strlen(sendBuff), 0)) != 0)
		{
			ROS_ERROR("Send failed");
			ROS_ERROR_STREAM(strerror(errno));
			continue;
		}
	}
}

/********************************************
函数名：closeSocket
功能：关闭TCP
********************************************/
void TCP_Sender::closeSocket()
{
	close(serverSocket);
	ROS_INFO("Close Server");
}

