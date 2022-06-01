#! /usr/bin/env python2
# -*- coding: utf-8 -*-

'''根据tf信息获得小车当前位置信息，并根据小车位置信息来控制的参数'''

from platform import node
from time import sleep
import rospy
import tf
import roslib
from  geometry_msgs.msg import Pose, Twist
import dynamic_reconfigure.client
import std_msgs.msg

class PoseNode:
    def __init__(self):
        self.last_received_pose = Pose()
        self.pose_x = 0.0
        self.pose_y = 0.0
        #设置频率
        self.rate = rospy.Rate(50)
        '''
        ---------------------------
        |                    x2,y2|
        |x1,y1                    |
        ---------------------------
        '''
        #[[x1,y1,x2,y2],.........],将赛道分成不同区域
        self.location = [[-1.0, -1.0, 3.0, 1.0],
                        [3.0, -5.0, 8.0, 1.0], [6.0, -6.0, 8.0, 1.0],
                        [3.0, -7.0, 8.0, -5.0],
                        [1.4, -7.0, 3.0, -1.0], 
                        [-1.0, -7.0, 1.4, -1.0], [1.4, -1.0, 3.0, -1.0]]
        self.client_local = dynamic_reconfigure.client.Client("/move_base/local_costmap", timeout=5, config_callback=self.callback1)

    def callback1(config1, config2):
        # rospy.loginfo("Config set to {int_param}, {double_param}, {str_param}, {bool_param}, {size}".format(**config2))
        # print(type(config1), type(config2))
        pass

    # def robot_teb_update(self):
    #     #判断小车所处的区域
    #     for i in range(0,7):
    #         if self.pose_x > self.location[i][0] and self.pose_x < self.location[i][2] and self.pose_y > self.location[i][1] and self.pose_y < self.location[i][3]:
    #             j = i
    #             break
    #     else:
    #         j =  0
            
    #     #根据区域来设置参数
    #     if(j == 0):
    #         # rospy.loginfo("Location 1")
    #         self.client_local.update_configuration({"width":9, "height":3})
    #         self.client_teb.update_configuration({
    #         #    "acc_lim_x":1.5, "acc_lim_theta":1.5, "max_vel_x":0.9, "max_vel_theta":2.3, "min_turning_radius":0.2, "min_obstacle_dist":0.3,
    #             "acc_lim_x":1.5, "acc_lim_theta":1.5, "max_vel_x":0.85, "max_vel_theta":2.3, "min_turning_radius":0.2, "min_obstacle_dist":0.3,
    #            "penalty_epsilon":0.1, "weight_optimaltime":2.0, 
    #            "weight_acc_lim_x":5.5, "weight_acc_lim_theta":15.0, "weight_max_vel_x":15.54, "weight_max_vel_theta":9.3,
    #            "weight_kinematics_turning_radius":1.1
    #         })


    #     elif(j == 1 or j == 2):
    #         # rospy.loginfo("Location 2")
    #         self.client_local.update_configuration({"width":3, "height":3})
    #         self.client_teb.update_configuration({ #acc_lim_theta":1.32  "max_vel_theta":2.12 "min_turning_radius":0.2
    #            "acc_lim_x":0.46, "acc_lim_theta":1.32, "max_vel_x":0.65, "max_vel_theta":2.12, "min_turning_radius":0.2, "min_obstacle_dist":0.3,
    #            "penalty_epsilon":0.1, "weight_optimaltime":2.0, 
    #            "weight_acc_lim_x":2.5, "weight_acc_lim_theta":27.0, "weight_max_vel_x":7.54, "weight_max_vel_theta":9.3,
    #            "weight_kinematics_turning_radius":1.1
    #         })


    #     elif(j == 3):
    #         # rospy.loginfo("Location 3")
    #         self.client_local.update_configuration({"width":8, "height":3})
    #         self.client_teb.update_configuration({
    #         #    "acc_lim_x":0.86, "acc_lim_theta":1.32, "max_vel_x":0.80, "max_vel_theta":2.5, "min_turning_radius":0.2, "min_obstacle_dist":0.3,
    #         #    "penalty_epsilon":0.1, "weight_optimaltime":2.0, 
    #         #    "weight_acc_lim_x":6.5, "weight_acc_lim_theta":20.0, "weight_max_vel_x":10.54, "weight_max_vel_theta":9.3,
    #         #    "weight_kinematics_turning_radius":1.1
    #            "acc_lim_x":0.82, "acc_lim_theta":1.32, "max_vel_x":1.10, "max_vel_theta":2.12, "min_turning_radius":0.2, "min_obstacle_dist":0.3,
    #            "penalty_epsilon":0.1, "weight_optimaltime":2.0, 
    #            "weight_acc_lim_x":2.5, "weight_acc_lim_theta":27.0, "weight_max_vel_x":7.54, "weight_max_vel_theta":9.3,
    #            "weight_kinematics_turning_radius":1.1
    #         })


    #     elif(j == 4):
    #         # rospy.loginfo("Location 4")
    #         self.client_local.update_configuration({"width":3, "height":3})
    #         self.client_teb.update_configuration({
    #             "acc_lim_x":0.56, "acc_lim_theta":1.92, "max_vel_x":0.7, "max_vel_theta":3.12, "min_turning_radius":0.10, "min_obstacle_dist":0.3,
    #            "penalty_epsilon":0.1, "weight_optimaltime":2.0, 
    #            "weight_acc_lim_x":2.5, "weight_acc_lim_theta":40.0, "weight_max_vel_x":6.54, "weight_max_vel_theta":12.3,
    #            "weight_kinematics_turning_radius":10.1
    #         })
    #         #    "acc_lim_x":0.56, "acc_lim_theta":1.32, "max_vel_x":0.70, "max_vel_theta":2.12, "min_turning_radius":0.2, "min_obstacle_dist":0.3,
    #         #    "penalty_epsilon":0.1, "weight_optimaltime":2.0, 
    #         #    "weight_acc_lim_x":2.5, "weight_acc_lim_theta":27.0, "weight_max_vel_x":7.54, "weight_max_vel_theta":9.3,
    #         #    "weight_kinematics_turning_radius":1.1

    #     elif(j == 5 or j == 6):
    #         # rospy.loginfo("Location 5")
    #         self.client_local.update_configuration({"width":3, "height":5})
    #         self.client_teb.update_configuration({
    #            "acc_lim_x":0.46, "acc_lim_theta":1.32, "max_vel_x":0.40, "max_vel_theta":2.12, "min_turning_radius":0.2, "min_obstacle_dist":0.3,
    #            "penalty_epsilon":0.1, "weight_optimaltime":2.0, 
    #            "weight_acc_lim_x":2.5, "weight_acc_lim_theta":20.0, "weight_max_vel_x":7.54, "weight_max_vel_theta":8.3,
    #            "weight_kinematics_turning_radius":1.5
    #         })
    #         #    "acc_lim_x":0.46, "acc_lim_theta":1.32, "max_vel_x":0.40, "max_vel_theta":2.12, "min_turning_radius":0.2, "min_obstacle_dist":0.3,
    #         #    "penalty_epsilon":0.1, "weight_optimaltime":2.0, 
    #         #    "weight_acc_lim_x":2.5, "weight_acc_lim_theta":20.0, "weight_max_vel_x":7.54, "weight_max_vel_theta":8.3,
    #         #    "weight_kinematics_turning_radius":1.5
    #     else:
    #         pass
 
    # def 


    #通过TF获取小车位置信息
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
            rospy.loginfo("x = %f, y = %f", self.pose_x, self.pose_y)
            # self.robot_teb_update()
            self.rate.sleep()




if __name__ == '__main__':
    rospy.init_node("visualize_obstacle_velocity_profile", anonymous=True)
    try:
        node = PoseNode()
        node.robot_pose_update()
    except rospy.ROSInitException:
        pass