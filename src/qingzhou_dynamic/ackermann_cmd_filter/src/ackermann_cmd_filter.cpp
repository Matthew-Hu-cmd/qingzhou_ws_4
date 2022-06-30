/******************************************
文件名：ackermann_cmd_filter.cpp

功  能：融合各个话题发布的ackermann控制数据，根据
不同的路段，选择不同话题发布的ackermann_cmd数据
dongtaitiaocan

作者：胡杨
******************************************/

#include "ackermann_cmd_filter.h"

int main(int argc, char** argv)
{
	//初始化ROS节点
	setlocale(LC_ALL,"");
	ros::init(argc, argv, "ackermann_cmd_filted_node");
	AckermannCmdFilter ackermannCmdFilter;
	ROS_INFO("ackermann_cmd_filter Finish");
	return 0;
}

/******************************************
Name ： AckermannCmdFilter
Param: Null
Func : 构造函数
作 者 ：胡杨
******************************************/
AckermannCmdFilter::AckermannCmdFilter()
{
	//Private parameters handler
	ros::NodeHandle pn("~");

    //Controller parameter
	
    pn.param("controller_freq", controller_freq, 20);

	//Publishers and Subscribers
	location_sub = nh.subscribe("/qingzhou_locate", 1, &AckermannCmdFilter::locateCB, this);
	goal_sub = nh.subscribe("/move_base_simple/goal", 1, &AckermannCmdFilter::goalCB, this);
	ackermannFromOdom_sub = nh.subscribe("/ackermann_cmd", 1, &AckermannCmdFilter::ackermannCmdFromOdomCB, this);
	ackermannFromVision_sub = nh.subscribe("/vision_control", 10, &AckermannCmdFilter::ackermannCmdFromVisionCB, this);
	ackermannCmdFilted_pub = nh.advertise<ackermann_msgs::AckermannDrive>("/ackermann_cmd_filted", 1);
	// Rubbish
	// locate_cli = nh.serviceClient<qingzhou_locate::RobotLocation>("qingzhou_locate");
	//Timer
	timer1 = nh.createTimer(ros::Duration((1.0)/controller_freq), &AckermannCmdFilter::controlLoopCB, this); // Duration(0.05) -> 20Hz
	
	ROS_INFO_STREAM("[param] controller_freq: " << controller_freq); 
	ROS_INFO_STREAM("Wait For Services");
	//等待服务端启动
	// locate_cli.waitForExistence();
	ROS_INFO("Filter Start");

	ros::spin();
}

/******************************************
Name ： ackermannCmdFromOdomCB
Param: Null
Func : Call back func, get Ackermann msgs 
       from odom (L1_controller_v3.cpp)
作 者 ：胡杨
******************************************/
void AckermannCmdFilter::ackermannCmdFromOdomCB(const ackermann_msgs::AckermannDrive::ConstPtr& msgs)
{
	ackermannCmdFromOdom = *msgs;
}

/******************************************
Name ： ackermannCmdFromVisionCB
Param: Null
Func : Call back func, get Ackermann msgs 
       from vision (hai mei xie)
作 者 ：0E
******************************************/
void AckermannCmdFilter::ackermannCmdFromVisionCB(const ackermann_msgs::AckermannDrive::ConstPtr& msgs)
{
	ackermannCmdFromVision = *msgs;
	// ROS_INFO("speed: %f", ackermannCmdFromVision.speed);
}

/******************************************
Name ： controlLoopCB
Param: Null
Func : pub Ackermann msgs to bringup according to 
       position
作 者 ：0E
******************************************/
void AckermannCmdFilter::controlLoopCB(const ros::TimerEvent&)
{
	//Use ackermann msgs from odom
	if ((robotLocation != TrafficLight) &&
		(robotLocation != RoadLine))
	{
		ackermannCmdFilted = ackermannCmdFromOdom;
	}
	//Use ackermann msgs from vision
	else if (robotLocation == TrafficLight ||
	 robotLocation == RoadLine)
	{
		ackermannCmdFilted = ackermannCmdFromVision;
		// ROS_INFO("Vision: speed: %f", ackermannCmdFilted.speed);
	}
	//Stop
	else
	{
		ackermannCmdFilted.speed = 0.0;
		ackermannCmdFilted.steering_angle = 0.0;
	}
	//publish ackermann msgs filted
	ackermannCmdFilted_pub.publish(ackermannCmdFilted);
}

/******************************************
Name : goalCB
Param: Null
Func : 得到目标点后，从qingzhou_locate那获取
		一次位置信息
作 者 ：胡杨
******************************************/
void AckermannCmdFilter::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
	goal = *goalMsg;
	ROS_INFO("Goal Received");
	//Whether Traffic Light or Roadline
	if (false)
	{

	}
	else
	{
		
	}

}

/******************************************
Name : locateCB
Param: Null
Func : 得到话题发布的位置信息
作 者 ：胡杨
******************************************/
void AckermannCmdFilter::locateCB(const std_msgs::Int32& data)
{
	robotLocation = RobotLocation(data.data);
	if (ros::param::param("Debug", true))
	{
		ROS_INFO("Receive Location: %d", robotLocation);
	}
}
