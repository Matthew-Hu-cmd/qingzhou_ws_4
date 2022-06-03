//******************************
//文件名：robot_states_msgs.h
//功  能：定义发送给客户端的数据
//作  者：胡杨
//******************************
#ifndef __ROBOT_STATES_MSGS__
#define __ROBOT_STATES_MSGS__

#include <ros/ros.h>

//存放机器人状态信息的结构体
typedef struct robotStatesMsgs
{
	float battary;			//电池电压
	float linearSpeed;		//线速度
    float angularSpeed;		//角速度
    float locationX;		//X坐标
    float locationY;		//Y坐标
};

//订阅发送以上信息的话题
void SubMsgTopic(const ros::NodeHandle &nodeHandler);
#endif