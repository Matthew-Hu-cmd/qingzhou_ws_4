//******************************//
//文件名：robot_states_msgs.cpp
//功  能：实现机器人状态信息获取

#include "robot_states_msgs.h"

//订阅发送机器人状态信息的话题
void TCP_Sender::SubMsgTopic()
{
	bettarySub = nh.subscribe<std_msgs::Float32>("/battery", 2, &TCP_Sender::SubBettaryInfoCB, this);

}

void SubSpeedCB(const geometry_msgs::Twist::ConstPtr &msg)
{
	
}