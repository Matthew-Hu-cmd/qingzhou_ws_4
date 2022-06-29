/******************************************
文件名：ackermann_cmd_filter.h

功  能：融合各个话题发布的ackermann控制数据，根据
不同的路段，选择不同话题发布的ackermann_cmd数据

作者：胡杨
******************************************/

#ifndef __ACKERMANN_CMD_FILTER__
#define __ACKERMANN_CMD_FILTER__

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include "qingzhou_locate/RobotLocation.h"
#include <std_msgs/Int32.h>

//用于机器人路段确定
enum RobotLocation : int
{
	Start, Load, TrafficLight ,Unload, RoadLine, 
	RoadLineToStart, StartToLoad, LoadToTrafficLight, TrafficLightToUnload, UnloadToRoadLine,
	Unknown
};

class AckermannCmdFilter
{
private:
	ros::NodeHandle nh;

	//发布AckermannCmdFilted话题
	ros::Publisher ackermannCmdFilted_pub;
	ros::Subscriber location_sub, goal_sub, ackermannFromOdom_sub, ackermannFromVision_sub;

	//客户端
	qingzhou_locate::RobotLocation locate;
	ros::ServiceClient locate_cli;
	
	//由定时器控制控制机器人的频率，控制函数为回调函数
	ros::Timer timer1;

	ackermann_msgs::AckermannDrive ackermannCmdFilted, ackermannCmdFromOdom, ackermannCmdFromVision;

	//RobotLocation robotLocation;机器人位置信息
	RobotLocation robotLocation;

	int controller_freq;

	//Call Back
	void locateCB(const std_msgs::Int32& data);
	void ackermannCmdFromOdomCB(const ackermann_msgs::AckermannDrive::ConstPtr& msgs);
	void ackermannCmdFromVisionCB(const ackermann_msgs::AckermannDrive::ConstPtr& msgs);
    void controlLoopCB(const ros::TimerEvent&);
	void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);

	bool locateCallService();

public:
	AckermannCmdFilter();
	// void DynamicParameters();
};




#endif