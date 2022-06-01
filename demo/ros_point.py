#from launch_demo import launch_demo
import rospy

import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from geometry_msgs.msg import PoseStamped
from math import pi
from std_msgs.msg import Int16

class navigation_demo:
    def __init__(self):
        # self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        # self.sub_socket = rospy.Subscriber('/socket',Int16,self.socket_cb)
        # self.start = 0
        # self.end = 0
        # self.traffic = True
    # def set_pose(self, p):
    #     if self.move_base is None:
    #         return False

    #     x, y, th = p

    #     pose = PoseWithCovarianceStamped()
    #     pose.header.stamp = rospy.Time.now()
    #     pose.header.frame_id = 'map'
    #     pose.pose.pose.position.x = x
    #     pose.pose.pose.position.y = y
    #     q = transformations.quaternion_from_euler(0.0, 0.0, th/180.0*pi)
    #     pose.pose.pose.orientation.x = q[0]
    #     pose.pose.pose.orientation.y = q[1]
    #     pose.pose.pose.orientation.z = q[2]
    #     pose.pose.pose.orientation.w = q[3]

    #     self.set_pose_pub.publish(pose)
    #     return True

    # def socket_cb(self,msg):
    #     if (msg.data == 1):
    #         self.start = 0
    #         self.end = 2
    #     elif (msg.data == 2):
    #         self.start = 2
    #         self.end = 3

    # def _done_cb(self, status, result):
    #     rospy.loginfo("navigation done! status:%d result:%s"%(status, result))

    # def _active_cb(self):
    #     rospy.loginfo("[Navi] navigation has be actived")

    # def _feedback_cb(self, feedback):
    #     rospy.loginfo("[Navi] navigation feedback\r\n%s"%feedback)

    def goto(self, p):
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

if __name__ == "__main__":
    rospy.init_node('navigation_demo',anonymous=True)
    nav = navigation_demo()
    #pdX = [2,2.511,2.511,0.5,-2.1,0,1.25,1.86,0]
    #pdY = [-2,-4.534,-6.134,-7.934,-6.2,-6.12,-4.12,2.01,0]
    #pdth = [-30,-90,-90,-170,75,-60,90,93,0]
    #pdX = [ 2.113   ,  2.013 , -2.349 , 0.995   , -1.831 , 0   ]
    #pdY = [ -4.231  ,  -5.5,    -6.089 , -5.005 , -1.745 , 0   ]
    #pdth = [-90 , -100 , 90.5 , 91.03 , 91.6 , -30]
    pdX = [1.965, 2.031, 1.902, 1.312, -0.417, -1.902, -2.389, 0.857, -1.025, -1.965, -0.093, 0.308]
    pdY = [-1.286, -2.488, -4.566, -7.588, -7.742, -7.403, -5.408, -4.486, -2.178, -0.754, 0.094, 0.069]
    pdth = [0,0,0,0,0,0,0,0,0,0,0,0]
    #pdX = [-2.571]
    #pdY = [-5.334]
    #pdth = [80]
    length = len(pdX)
   #l aunch_nav = launch_demo(["roslaunch", "pibot_simulator", "nav.launch"])
   # launch_nav.launch()

    #r = rospy.Rate(0.2)
    #r.sleep()

    #rospy.loginfo("set pose...")
    #r = rospy.Rate(1)
    #r.sleep()
   
   # navi.set_pose([2,-1,0])
    print(length)
    navi = navigation_demo()
    rospy.loginfo("goto goal...")
    i = 0
    # if (nav.traffic):
    #     for i in range(nav.start,nav.end):
    #         w = [pdX[i],pdY[i],pdth[i]]
        
    #         navi.goto([w[0],w[1],w[2]])
    #         r = rospy.Rate(0.32)
    #         r.sleep()
    #         i = i + 1
    for i in range(0,length):
        w = [pdX[i],pdY[i],pdth[i]]
        
        navi.goto([w[0],w[1],w[2]])
        print(w,i)
        r = rospy.Rate(0.1)
        r.sleep()
        i = i + 1

    # while not rospy.is_shutdown():
    #     r.sleep()
