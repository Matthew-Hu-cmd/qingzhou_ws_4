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

}

/******************************************
Name ： AckermannCmdFilter
Param: Null
Func : constructor
作 者 ：胡杨
******************************************/
AckermannCmdFilter::AckermannCmdFilter()
{
	//Private parameters handler
	ros::NodeHandle pn("~");

    //Controller parameter
    pn.param("controller_freq", controller_freq, 20);

	//Publishers and Subscribers
	odom_sub = nh.subscribe("/odom_ekf", 1, &AckermannCmdFilter::odomCB, this);
	goal_sub = nh.subscribe("/move_base_simple/goal", 1, &AckermannCmdFilter::goalCB, this);
	ackermannFromOdom_sub = nh.subscribe("话题名", 1, &AckermannCmdFilter::ackermannCmdFromOdomCB, this);
	ackermannFromVision_sub = nh.subscribe("话题名", 1, &AckermannCmdFilter::ackermannCmdFromOdomCB, this);
	ackermannCmdFilted_pub = nh.advertise<ackermann_msgs::AckermannDrive>("ackermann_cmd_filted", 1);

	//Timer
	timer1 = nh.createTimer(ros::Duration((1.0)/controller_freq), &AckermannCmdFilter::controlLoopCB, this); // Duration(0.05) -> 20Hz
	timer2 = nh.createTimer(ros::Duration((1.0)/5), &AckermannCmdFilter::robotLocationCB, this);

	client = new dynamic_reconfigure::Client<qingzhou_nav::L1_dynamicConfig> ("/L1_controller_v3", 10, &AckermannCmdFilter::dynamicCB);
	
	ROS_INFO_STREAM("[param] controller_freq: " << controller_freq); 
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
作 者 ：胡杨
******************************************/
void AckermannCmdFilter::ackermannCmdFromVisionCB(const ackermann_msgs::AckermannDrive::ConstPtr& msgs)
{
	ackermannCmdFromVision = *msgs;
}

/******************************************
Name ： controlLoopCB
Param: Null
Func : pub Ackermann msgs according to 
       position
作 者 ：胡杨
******************************************/
void AckermannCmdFilter::controlLoopCB(const ros::TimerEvent&)
{
	//Use ackermann msgs from odom
	if ((robotLocation != LoadToTrafficLight) && (robotLocation != RoadLine))
	{
		ackermannCmdFilted = ackermannCmdFromOdom;
	}
	//Use ackermann msgs from vision
	else if (robotLocation == LoadToTrafficLight || (robotLocation == RoadLine))
	{
		ackermannCmdFilted = ackermannCmdFromVision;
	}
	//Stop
	else
	{
		ackermannCmdFilted.speed = 0;
	}
	//publish ackermann msgs filted
	ackermannCmdFilted_pub.publish(ackermannCmdFilted);
}

/******************************************
Name : robotLocationCB
Param: Null
Func : get robotLocation
作 者 ：胡杨
******************************************/
void AckermannCmdFilter::robotLocationCB(const ros::TimerEvent&)
{
	if (0)
	{
		robotLocation = StartToLoad;
	}
	else if (0)
	{
		robotLocation = LoadToTrafficLight;
	}
	else if (0)
	{
		robotLocation = TrafficLightToUnload;
	}
	else if (0)
	{
		robotLocation = UnloadToRoadLine;
	}
	else if (0)
	{
		robotLocation = RoadLine;
	}
	else if (0)
	{
		robotLocation = RoadLineToStart;
	}
	else
	{
		robotLocation = Unknown;
	}

}

/******************************************
Name : odomCB
Param: Null
Func : get odom msgs from ekf
作 者 ：胡杨
******************************************/
void AckermannCmdFilter::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    odom = *odomMsg;
}

/******************************************
Name : goalCB
Param: Null
Func : get goal msgs from move_base
作 者 ：胡杨
******************************************/
void AckermannCmdFilter::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    try
    {
        geometry_msgs::PoseStamped odom_goal;
        odom_goal_pos = odom_goal.pose.position;
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}


/******************************************
Name : DynamicParameters
Param: Null
Func : get goal msgs from move_base
作 者 ：胡杨
******************************************/
void AckermannCmdFilter::DynamicParameters()
{
	if 
}