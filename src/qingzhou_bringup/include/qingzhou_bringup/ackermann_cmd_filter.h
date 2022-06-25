/******************************************
文件名：ackermann_cmd_filter.h

功  能：融合各个话题发布的ackermann控制数据，根据
不同的路段，选择不同话题发布的ackermann_cmd数据

作者：胡杨
******************************************/
#ifndef __ACKERMANN_CMD_FILTER__
#define __ACKERMANN_CMD_FILTER__

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include "dynamic_reconfigure/client.h"
#include "qingzhou_nav/L1_dynamicConfig.h"

enum RobotLocation 
{
	Start, StartToLoad, Load, LoadToTrafficLight, TrafficLightToUnload,
	Unload, UnloadToRoadLine, RoadLine, RoadLineToStart, Unknown
};

class AckermannCmdFilter
{
private:
	ros::NodeHandle nh;

	//发布AckermannCmdFilted话题
	ros::Publisher ackermannCmdFilted_pub;
	ros::Subscriber odom_sub, ackermannFromOdom_sub, ackermannFromVision_sub;

	ros::Timer timer1;
	ros::Timer timer2;

	dynamic_reconfigure::Client<qingzhou_nav::L1_dynamicConfig>* client;

	ackermann_msgs::AckermannDrive ackermannCmdFilted, ackermannCmdFromOdom, ackermannCmdFromVision;

	//RobotLocation robotLocation;机器人位置信息
	geometry_msgs::Odometry odom;
	geometry_msgs::Point odom_goal_pos;
	RobotLocation robotLocation;

	int controller_freq;

	//Call Back
	void ackermannCmdFromOdomCB(const ackermann_msgs::AckermannDrive::ConstPtr& msgs);
	void ackermannCmdFromVisionCB(const ackermann_msgs::AckermannDrive::ConstPtr& msgs);
    void controlLoopCB(const ros::TimerEvent&);
	void robotLocationCB(const ros::TimerEvent&);
	void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
	void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
	void dynamicCB(const )
public:
	AckermannCmdFilter();
	~AckermannCmdFilter();
	void DynamicParameters();
};




#endif