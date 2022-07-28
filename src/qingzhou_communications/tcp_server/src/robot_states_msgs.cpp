//******************************//
//文件名：robot_states_msgs.cpp
//功  能：实现机器人状态信息获取

#include "robot_states_msgs.h"
#include "TCP_Sender.h"

//订阅发送机器人状态信息的话题
void TCP_Sender::SubMsgTopic()
{
	//机器人电压信息发布话题在qingzhou_bringup.cpp
	bettarySub = nh.subscribe<std_msgs::Float32>("/battery", 2, &TCP_Sender::SubBettaryInfoCB, this);
	//机器人速度信息发布话题在L1_controller.cpp
	speedSub = nh.subscribe<ackermann_msgs::AckermannDrive>("/ackermann_cmd", 5, &TCP_Sender::SubSpeedCB, this);
	//机器人位置信息发布话题在qingzhou_bringup.launch可以看到
	odomSub = nh.subscribe("/odom_ekf", 1, &TCP_Sender::SubOdomCB, this);
}



void TCP_Sender::SubBettaryInfoCB(const std_msgs::Float32::ConstPtr &msg)
{
	robotStatesMsgs.battary = msg.get()->data;
}


//获取机器人速度和转弯角度信息
void TCP_Sender::SubSpeedCB(const ackermann_msgs::AckermannDrive::ConstPtr &msg)
{
	robotStatesMsgs.linearSpeed = msg.get()->speed;
	robotStatesMsgs.angularSpeed = msg.get()->steering_angle;
}

void TCP_Sender::SubOdomCB(const nav_msgs::Odometry::ConstPtr &msg)
{
	robotStatesMsgs.locationX = msg.get()->pose.pose.position.x;
	robotStatesMsgs.locationY = msg.get()->pose.pose.position.y;
}