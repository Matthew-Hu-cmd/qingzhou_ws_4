/******************************************
文件名：pub_goal.cpp

功  能：相当于在rviz上发布目标点，小工具
		输入3个参数：x，y，w
作者：胡杨
******************************************/

#include <ros/ros.h>
#include <stdlib.h>
#include <tf/tf.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
	if (argc != 4)
    {
        ROS_ERROR("Please 3 number");
        return 1;
    }

	ros::init(argc, argv, "pub_goal_node");
	ros::NodeHandle nh;

	ros::Publisher pub;
	pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

	geometry_msgs::PoseStamped goal;
	goal.header.frame_id = "map";
	goal.header.stamp = ros::Time::now();
	
	goal.pose.position.x = atof(argv[1]);
	goal.pose.position.y = atof(argv[2]);
	goal.pose.position.z = 0.0;
	
	geometry_msgs::Quaternion quaternion;
	quaternion = tf::createQuaternionMsgFromYaw(atof(argv[3]));
	goal.pose.orientation.w = quaternion.w;
	goal.pose.orientation.x = quaternion.x;
	goal.pose.orientation.y = quaternion.y;
	goal.pose.orientation.z = quaternion.z;

	// ROS_INFO("%f %f %f %f %f %f %f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
	// goal.pose.orientation.w, goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z);
	while(ros::ok())
	{
		pub.publish(goal);
		ROS_INFO("Send Goal !!!");
		ros::Rate(1).sleep();
	}

	return 0;
}