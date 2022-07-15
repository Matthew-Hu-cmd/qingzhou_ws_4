/******************************************
文件名：qingzhou_locate.cpp

功  能：机器人当前坐标和目标点坐标

作者：胡杨
******************************************/

/************************************************************************************
 * read HY: 本来是想用服务来提供RobotLocatino信息的，但是需要客户端请求，而客户端都是在得到
 * 新发布的目标点后，请求服务，而RobotLocatino的计算也是在得到新发布的目标点后，不确定谁先执
 * 行，所以改用topic话题通信，RobotLocation改变，发布一次话题，而不是一直发布
************************************************************************************/

#include "qingzhou_locate.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "qingzhou_locate");
	
	Locate locate;

	return 0;
}

Locate::Locate()
{
	ros::NodeHandle pn("~");
	
	pn.setParam("DeBug", false);

	goalLocation = Unknown;
	robotLocation = Unknown;
	lastLocation = Unknown;
	//Publishers and Subscribers
	odom_sub = nh.subscribe("/amcl_pose", 1, &Locate::odomCB, this);
	goal_sub = nh.subscribe("/move_base_simple/goal", 1, &Locate::goalCB, this);
	location_sub =  nh.subscribe("/qingzhou_locate", 1, &Locate::locateCB, this);
	location_pub = nh.advertise<std_msgs::Int32>("/qingzhou_locate", 1);


	locate_srv = nh.advertiseService("qingzhou_locate", &Locate::locateReq, this);
	ROS_INFO("Locate Service Start");

	ros::Rate(1).sleep();

	ros::spin();
}

/******************************************
Name ： odomCB
Param: Null
Func : amcl_pose订阅者的回调函数，可以获取小车位置
		坐标，根据坐标来改变RobotLocation信息
作 者 ：胡杨
******************************************/
void Locate::odomCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odomMsg)
{
	odom = *odomMsg;
	// ROS_INFO("Odom Received!");
	// ROS_INFO_STREAM("Robot x: " << odom.pose.pose.position.x << "Robot y: " << odom.pose.pose.position.y);
	//以下代码可以用定时器另外写一个函数实现，不知道哪种更好
	if (goalLocation == Unknown)
	{
		robotLocation == Unknown;
		locationPub();
		return;
	}

	switch (goalLocation)
	{
		//是否在装货区
		case Load:
			if (odom.pose.pose.position.x > goalPoint.x1[Load] 
			&& odom.pose.pose.position.x < goalPoint.x2[Load] 
			&& odom.pose.pose.position.y > goalPoint.y1[Load] 
			&& odom.pose.pose.position.y < goalPoint.y2[Load])
			{
				robotLocation = goalLocation;
			}	
			else
			{
				robotLocation = RobotLocation(goalLocation + 5);
			}
			break;

		case TrafficLight:
			if (lastLocation != TrafficLightToUnload
			&& odom.pose.pose.position.x > goalPoint.x1[TrafficLight] 
			&& odom.pose.pose.position.x < goalPoint.x2[TrafficLight] 
			&& odom.pose.pose.position.y > goalPoint.y1[TrafficLight] 
			&& odom.pose.pose.position.y < goalPoint.y2[TrafficLight])
			{
				robotLocation = goalLocation;
			}	
			else if(lastLocation != TrafficLightToUnload)
			{
				robotLocation = RobotLocation(goalLocation + 5);
			}
			break;

		//是否在卸货区或路上的其他区域
		case Unload:
			if (lastLocation == Load)
			{
				robotLocation = LoadToTrafficLight;
			}
			else if(lastLocation == LoadToTrafficLight 
			&& odom.pose.pose.position.x > goalPoint.x1[TrafficLight] 
			&& odom.pose.pose.position.x < goalPoint.x2[TrafficLight] 
			&& odom.pose.pose.position.y > goalPoint.y1[TrafficLight]
			&& odom.pose.pose.position.y < goalPoint.y2[TrafficLight])
			{
				robotLocation = TrafficLight;
			}
			// else if(lastLocation == TrafficLight)
			// {
			// 	robotLocation = TrafficLightToUnload;  //TrafficLightToUnload由视觉给
			// }
			else if(lastLocation == TrafficLightToUnload 
			&& odom.pose.pose.position.x > goalPoint.x1[Unload] 
			&& odom.pose.pose.position.x < goalPoint.x2[Unload] 
			&& odom.pose.pose.position.y > goalPoint.y1[Unload]
			&& odom.pose.pose.position.y < goalPoint.y2[Unload])
			{
				robotLocation = Unload;
			}
			break;
		
		//是否在che
		case RoadLine:
			if (lastLocation == Unload)
			{
				robotLocation = UnloadToRoadLine;
			}
			else if(lastLocation == UnloadToRoadLine 
			&& odom.pose.pose.position.x > goalPoint.x1[RoadLine] 
			&& odom.pose.pose.position.x < goalPoint.x2[RoadLine] 
			&& odom.pose.pose.position.y > goalPoint.y1[RoadLine]
			&& odom.pose.pose.position.y < goalPoint.y2[RoadLine])
			{
				robotLocation = RoadLine;
			}
			break;

		//是否在起始区
		case Start:
			if (lastLocation == Unload)
			{
				robotLocation = UnloadToRoadLine;
			}
			else if(lastLocation == UnloadToRoadLine 
			&& odom.pose.pose.position.x > goalPoint.x1[RoadLine] 
			&& odom.pose.pose.position.x < goalPoint.x2[RoadLine] 
			&& odom.pose.pose.position.y > goalPoint.y1[RoadLine]
			&& odom.pose.pose.position.y < goalPoint.y2[RoadLine])
			{
				robotLocation = RoadLine;
			}
			else if(lastLocation == RoadLine
			&& odom.pose.pose.position.x > goalPoint.x1[Start] 
			&& odom.pose.pose.position.x < goalPoint.x2[Start] 
			&& odom.pose.pose.position.y > goalPoint.y1[Start]
			&& odom.pose.pose.position.y < goalPoint.y2[Start])
			{
				robotLocation = Start;
			}
			break;

		default:
			break;
	}
	locationPub();
}

void Locate::locateCB(const std_msgs::Int32& data)
{
	robotLocation = RobotLocation(data.data);
	lastLocation =  robotLocation;
}

/******************************************
Name ： locateReq
Param: Null
Func : 向请求小车位置信息的客户端返回小车位置信息
作 者 ：胡杨
******************************************/
bool Locate::locateReq(qingzhou_locate::RobotLocation::Request& req,
	qingzhou_locate::RobotLocation::Response& resp)
{
	//返回机器人位置
	resp.location = robotLocation;
	return true;
}

/******************************************
Name ： goalCB
Param: Null
Func : move_base/goal订阅者的回调函数，可以获取
		小车目标位置坐标，根据坐标来改变goalLocation
		和robotLocation信息
作 者 ：胡杨
******************************************/
void Locate::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
	goal = *goalMsg;
	ROS_INFO("Goal Received!");
	for (int i = 0; i < 5; ++i)
	{
		//通过判断机器人目标点坐标是否在目标点范围内，若不在目标点范围内，就是未知
		if (goal.pose.position.x > goalPoint.x1[i] && goal.pose.position.x < goalPoint.x2[i] && 
			goal.pose.position.y > goalPoint.y1[i] && goal.pose.position.y < goalPoint.y2[i])
		{
			goalLocation = RobotLocation(i);
			return;
		}
	}
	goalLocation = Unknown;
}

/******************************************
Name ：locationPub
Param: Null
Func : 当位置改变时，发布机器人位置
作 者：胡杨
******************************************/
void Locate::locationPub()
{
	if (robotLocation != lastLocation)
	{
		lastLocation = robotLocation;
		std_msgs::Int32 data;
		data.data = int(robotLocation);
		location_pub.publish(data);
		ROS_INFO("Publish Location: %d", robotLocation);
	}
}
