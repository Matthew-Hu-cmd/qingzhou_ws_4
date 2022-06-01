#!/usr/bin/env python
# -*- coding: utf-8 -*-
## 添加在车道线入口不停止的参数
import rospy
import numpy
from dynamic_reconfigure import client
from geometry_msgs.msg import Pose
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import tf
from qingzhou_bringup.srv import app

class ParamCondition():
    def __init__(name = "",xmin = 0,xmax = 0,ymin = 0,ymax = 0,goalstatus = GoalStatus(),params = {}):
        self.name = name
        # self.pose = pose
        self.goalstatus = goalstatus
        self.params = params
        


class DynamicParamsClient():
    def __init__(self):
        rospy.init_node("DynamicParamsClient")
        rospy.loginfo("wait for service")
        #self.log("unloadtorlstart","wait for service")
        rospy.wait_for_service("/move_base/DWAPlannerROS/set_parameters",timeout=100)
        rospy.wait_for_service("/TCP_Sender/app",timeout = 100)
        rospy.wait_for_service("/cmdvel_filter_client",timeout = 100)
        rospy.wait_for_service("/move_base/clear_costmaps",timeout = 100)
 
        self.debug = True
        self.DWAClient = client.Client("/move_base/DWAPlannerROS",0.2,self.DWAParamsChangedCallback)
        self.goalStatusSuber = rospy.Subscriber("/TCP_Sender/GoalStatus",GoalStatus,self.GoalStatusCallback,queue_size=1)
        self.poseSuber = rospy.Subscriber("/TCP_Sender/PoseInMap",Pose,self.PoseCallback,queue_size=2)
        self.cmdPuber = rospy.Publisher("/cmd_vel_filted",Twist,queue_size=3)
        self.tcpsenderAppClient = rospy.ServiceProxy("/TCP_Sender/app",app)
        self.cmd_filter_client = rospy.ServiceProxy("/cmdvel_filter_client",app)
        self.clearcostmap_client = rospy.ServiceProxy("/move_base/clear_costmaps",Empty)
        
        self.turnTimer = rospy.Timer(rospy.Duration(1.0), self.turn_timer_callback)
        
        self.normal = {"min_vel_x":0.5  ,"max_vel_x":1.0,"min_vel_theta":0.1,"max_vel_theta":1.0}

        # self.turnTimer.

        self.DWAConfig = None
        self.pose = Pose()
        self.goalStatus = GoalStatus()

        self.openUnloadToRLstart = False
        self.openLoadToUnload = False
        self.DWACondition = [False,False,False]
        self.openStartToLoad = False
        self.openRLouttostart = False
        self.load_move = False
        self.yaw = None
        self.srv = rospy.Service("/DynamicParamsClient",app,self.srvCallback)
        rospy.loginfo("set up dpc")

    def srvCallback(self,req):
        # self.DWACondition[0] = False
        # self.DWACondition[1] = False
        if(req.statue == 1):
            self.DWACondition[0] = False
            self.DWACondition[1] = False
            self.log("","open unload to rl start")
            self.openUnloadToRLstart = True
        if(req.statue == 2):
            self.openLoadToUnload = True
        if(req.statue == 3):
            self.DWACondition[0] = False
            self.DWACondition[1] = False
            self.log("","open openStartToLoad")
            self.openStartToLoad = True
        if(req.statue == 4):
            #出S道
            self.DWACondition[0] = False
            self.DWACondition[1] = False
            self.log("","open openRLouttostart")
            self.openRLouttostart = True
        if(req.statue == 5):#装货区是否前进
            self.load_move = True
        return 0

    def turn_timer_callback(self,_):
        pass

    def RLouttostart(self):
        if(not self.DWACondition[0] and not self.DWACondition[1]):#进入装货区前减速
            # self.DWAConfig = self.DWAClient.get_configuration()#保存旧的配置
            # self.log("DWAParamCallback","old min_vel_x:{}".format(self.DWAConfig["min_vel_x"]))

            # self.DWAClient.update_configuration({"min_vel_x":0.8,"max_vel_x":1.2,"max_vel_theta":2.0})

            self.log("","[RLouttostart]: set max_vel_x: 1.2, min_vel_x:1.2 max_vel_theta:2.0")
            self.DWACondition[0] = True
        if(self.DWACondition[0] and not self.DWACondition[1] and self.pose.position.x>-0.8): #恢复配置

            # self.DWAClient.update_configuration(self.normal)为了调参注释

            self.log("","[RLouttostart]: RESET CONFIG")
            self.DWACondition = [False,False,False]
            self.openRLouttostart = False
    def PoseCallback(self,pose):
        self.pose = pose
        (r,p,self.yaw) = tf.transformations.euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
        self.yaw = self.yaw/numpy.pi*180
        pass
    
    def GoalStatusCallback(self,status):
        self.goalStatus = GoalStatus

    def loadtounload(self):
        if(not self.DWACondition[0] and not self.DWACondition[1] and not self.DWACondition[2]):# 到达起始区区域的时候向前开一点距离
            if(self.pose.position.y >-3.85 ):
                # self.log("","[loadtounload]: stop cmd control and move forward a little")
                self.cmd_filter_client(1) 
                cmd_data = Twist()
                cmd_data.linear.x = 0.8
                cmd_data.angular.z = (-95 - self.yaw)/180*3.14*1.8
                self.log("","[oritation:{} add {}]".format(self.yaw,(cmd_data.angular.z/3.14*180)))
                self.cmdPuber.publish(cmd_data)
            else:
                # 恢复cmd控制
                self.log("","[loadtounload]: recovery cmd control")
                self.cmd_filter_client(2)
                # self.cmd_filter_client(8) #set linear p to 0.15
                self.DWACondition[0] = True  
        elif(self.DWACondition[0] and (not self.DWACondition[1]) and (not self.DWACondition[2]) and (self.pose.position.x < 0.3)):#减速带前减速
            # self.DWAClient.update_configuration({"max_vel_x":0.8})
            self.log("","[loadtounload]: set max_vel_x 0.8")   
            self.DWACondition[1] = True  
        elif(self.DWACondition[0] and  self.DWACondition[1] and not self.DWACondition[2] and self.pose.position.x<-0.7):#减速带后减速
            # self.DWAClient.update_configuration({"max_vel_x":0.6})
            self.log("","[loadtounload]: set max_vel_x 0.6")
            # if(abs(self.yaw - 90)<5):
            #     self.log("","[loadtounload]: stop cmd control and move forward a little")
            #     self.cmd_filter_client(1) 
            #     cmd_data = Twist()
            #     cmd_data.linear.x = 0.8
            #     # cmd_data.angular.z = (-95 - self.yaw)/180*3.14*2.0
            #     self.log("","[oritation:{} add {}]".format(self.yaw,(cmd_data.angular.z/3.14*180)))
            #     self.cmdPuber.publish(cmd_data)
            self.DWACondition[2] = True
        elif(self.DWACondition[0] and self.DWACondition[1] and self.DWACondition[2] and self.pose.position.y >-6.8): #恢复配置

            # self.DWAClient.update_configuration(self.normal)为了调参注释

            self.log("","[loadtounload]: RESET CONFIG")
            self.log("","[loadtounload]: recovery cmd control [{},{},{}]".format(self.pose.position.x,self.pose.position.y,self.yaw))
            self.cmd_filter_client(2)
            clearcostmap_req = Empty._request_class()
            self.clearcostmap_client(clearcostmap_req)
            self.DWACondition = [False,False,False]
            self.openLoadToUnload = False

    # 先固定1.2 转弯前恢复
    def starttoload(self):
        if(not self.DWACondition[0] and not self.DWACondition[1]):#进入装货区前减速

            # self.log("DWAParamCallback","old min_vel_x:{}".format(self.DWAConfig["min_vel_x"]))

            # self.DWAClient.update_configuration({"min_vel_x":1.1,"max_vel_x":1.1,"max_vel_theta":1.1,"min_vel_theta":0.3})为了调参注释

            self.log("","[starttoload]: set min_vel_x 1.2 max_vel_x:1.2 max_vel_theta:1.1 min_vel_theta:0.3")
            self.cmd_filter_client(6) # change angular boost to 1.2
            self.DWACondition[0] = True

        if(self.DWACondition[0] and not self.DWACondition[1] and self.pose.position.y<-1.6): #恢复配置

            # self.DWAClient.update_configuration(self.normal)为了调参注释

            self.log("","[starttoload]: RESET CONFIG")
            self.DWACondition = [False,False,False]
            self.cmd_filter_client(5) # change angcmd_filter_client
            # self.cmd_filter_client(7) # change linear p to 0.4
            self.openStartToLoad = False

    

    def unloadtorlstart(self):
        if(not self.DWACondition[0] and not self.DWACondition[1] and not self.DWACondition[2]):
            # 关闭角度增益
            self.cmd_filter_client(4)
            # 提高速度
            self.DWAConfig = self.DWAClient.get_configuration()#保存旧的配置
            # self.log("DWAParamCallback","old min_vel_x:{}".format(self.DWAConfig["min_vel_x"]))
            # self.DWAClient.update_configuration({"min_vel_x":1.0,"max_vel_x":1.0,"min_vel_theta":0.1,"max_vel_theta":1.5})为了调参注释
            self.log("","[unloadtorlstart]: set min_vel_x :1.0; max_vel_x:1.0 min_vel_theta 0.1； max_vel_theta: 2.0")
            self.DWACondition[0] = True
        elif(self.DWACondition[0] and not self.DWACondition[1] and not self.DWACondition[2] and self.pose.position.x > -0.25):
            self.DWACondition[1] = True
            self.log("","[unloadtorlstart]: prepare tp cancel goal")
            self.tcpsenderAppClient(1)#call service to weak thread
            # self.DWAClient.update_configuration({"min_vel_x":1.0,"max_vel_x":1.0,"min_vel_theta":0.1,"max_vel_theta":1.5})           
            # self.cmd_filter_client(3) # limit angular vel not < 1.5
        elif(self.DWACondition[0] and self.DWACondition[1] and not self.DWACondition[2] and self.pose.position.y >-5.1    ):
            # self.log("","[unloadtorlstart]: unlimit angular speed")
            # self.cmd_filter_client(2) #open cmd filter/cancel all limit
            self.DWACondition[2] = True
        elif(self.DWACondition[1] and self.DWACondition[0] and self.pose.position.y > -3.8):

            # self.DWAClient.update_configuration(self.normal)为了调参注释

            self.log("","[unloadtorlstart]: RESET CONFIG")
            # 开启角度增益
            self.cmd_filter_client(5)
            self.DWACondition = [False,False,False]
            self.openUnloadToRLstart = False

    def DWAParamsChangedCallback(self,config):
        # if(config is None):
        #     self.log("DWAParamsClient","params is none")
        # else:
        #     self.log("DWAParamsClient","params changed")
        pass

    def log(self,name,content):
        if(self.debug):
            rospy.loginfo(content,logger_name=name)

dc = DynamicParamsClient()
r = rospy.Duration(0.03)
while(not rospy.is_shutdown()):
    if(dc.openUnloadToRLstart):
        dc.unloadtorlstart()
    if(dc.openLoadToUnload):
        dc.loadtounload()
    if(dc.openStartToLoad):
        dc.starttoload()
        pass
    if(dc.openRLouttostart):
        dc.RLouttostart()

    rospy.sleep(r)
    