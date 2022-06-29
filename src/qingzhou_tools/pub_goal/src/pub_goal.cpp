/******************************************
文件名：pub_goal.cpp

功  能：相当于在rviz上发布目标点，小工具
		输入3个参数：x，y，w
作者：胡杨
******************************************/

#include <ros/ros.h>
#include <stdlib.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/tf.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
	if (argc != 4)
    {
        ROS_ERROR("Please 3 number");
        return 1;
    }

	ros::init(argc, argv, "pub_goal_node");
	ros::NodeHandle nh;

	MoveBaseClient ac("move_base", true);
	
	while(!ac.waitForServer(ros::Duration(5.0)))
	{
        ROS_INFO("Waiting for the move_base action server to come up");
    }

	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	
	goal.target_pose.pose.position.x = atof(argv[1]);
	goal.target_pose.pose.position.y = atof(argv[2]);
	goal.target_pose.pose.position.z = 0.0;
	
	geometry_msgs::Quaternion quaternion;
	quaternion = tf::createQuaternionMsgFromYaw(atof(argv[3]));
	goal.target_pose.pose.orientation.w = quaternion.w;
	goal.target_pose.pose.orientation.x = quaternion.x;
	goal.target_pose.pose.orientation.y = quaternion.y;
	goal.target_pose.pose.orientation.z = quaternion.z;

	ROS_INFO(" Init success!!! ");

	ac.sendGoal(goal);
	ROS_INFO("Send Goal !!!");
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
            ROS_INFO("The Goal achieved success !!!");
    }else
	{
		ROS_WARN("The Goal Planning Failed for some reason"); 
	}

	return 0;
}