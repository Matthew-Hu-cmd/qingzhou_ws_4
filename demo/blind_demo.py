#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

'''根据tf信息获得小车当前位置信息,并根据小车位置信息来控制的参数'''

from platform import node
from time import sleep
import rospy
import tf
import roslib
from  geometry_msgs.msg import Pose, Twist
import dynamic_reconfigure.client
import std_msgs.msg


import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from geometry_msgs.msg import PoseStamped
from math import pi



class navigation_demo:
    def __init__(self):
        # self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        
        #实现实时读取位置信息需要的函数
        self.last_received_pose = Pose()
        self.pose_x = 0.0
        self.pose_y = 0.0
        #设置频率
        self.rate = rospy.Rate(50)
        '''
        x+ <---------------------------
        	|                    x2,y2|
        	|x1,y1                    |
        							  \/ y+
            总之就是前面的坐标点会更大
        '''
        #[[x1,y1,x2,y2],.........],将赛道分成不同区域
		#顺序依次是装货、停止线、卸货区、S道前、原点
        self.location = [[2.3, -3.17, 1.7, -4.2],
                        [2.3, -5.5, 1.7, -5.8], 
						[-2.18, -5.54, -2.4, -6.11],
                        [1.25, -3.98, 0.40, -4.74],
                        [0.5, 0.5, -0.5, -0.5]]
        self.client_local = dynamic_reconfigure.client.Client("/move_base/local_costmap", timeout=5, config_callback=self.callback1)
        #目标点集合
        self.pdX = [1.966, 2.034, -2.213, 0.796, 0]
        self.pdY = [-4.244, -6.200, -5.414, -3.929, 0]
        self.pdth = [0,0,0,0,0]
        self.length = len(pdX)


    def callback1(config1, config2):
        # rospy.loginfo("Config set to {int_param}, {double_param}, {str_param}, {bool_param}, {size}".format(**config2))
        # print(type(config1), type(config2))
        pass


    def goto(self, p):      #goto somewhere
        # goal = MoveBaseGoal()
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = p[0]
        goal.pose.position.y = p[1] 
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2]/180.0*pi)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        #self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        self.pub_goal.publish(goal)
        return True

    def cancel(self):
        self.move_base.cancel_all_goals()
        return True

    def robot_pose_update(self):
        #监听TF关系
        listener = tf.TransformListener()

        while not rospy.is_shutdown():
            #从tf关系获取位置信息
            try:
                (trans,rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            self.pose_x = (round(trans[0],3))
            self.pose_y = (round(trans[1],3))
            # rospy.loginfo("x = %f, y = %f", self.pose_x, self.pose_y)
            self.robot_goal_update()
            self.rate.sleep()


    def robot_goal_update():    #实现在不同的位置区域发布不同的目标点
		#判断小车所处的区域
		for i in range(0,length):
            if self.pose_x < self.location[i][0] and self.pose_x > self.location[i][2] and 
			self.pose_y < self.location[i][1] and self.pose_y > self.location[i][3]:
                j = i
                break
        else:
            j =  0


		if(j == 0):
            # rospy.loginfo("Location 1")

        elif(j==1):

        elif(j==2):
        elif(j==3):
        elif(j==4):
            


    
    

if __name__ == "__main__":
    rospy.init_node('navigation_demo',anonymous=True)
    rospy.loginfo("goto goal...")
    try:
        nav = navigation_demo()     
        nav.robot_pose_update()
    except rospy.ROSInitException:
        pass
