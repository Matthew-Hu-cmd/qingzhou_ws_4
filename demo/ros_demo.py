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

if __name__ == "__main__":
    rospy.init_node('navigation_demo',anonymous=True)
    nav = navigation_demo()
    pdX = [1.969, 1.989, -2.150, 1.072, -1.664,0]
    pdY = [-4.149, -5.590, -5.617, -4.270,-1.405, 0]
    pdth = [0,0,0,0,0,0]
    sec = [15.0, 15.0, 15.0, 15.0, 15.0,15.0]
    length = len(pdX)
    print(length)
    navi = navigation_demo()
    rospy.loginfo("goto goal...")
    
    while True:
        i = 0

        for i in range(0,length):
            w = [pdX[i],pdY[i],pdth[i]]
            s = sec[i]
            hz = 1/s
            navi.goto([w[0],w[1],w[2]])
            print(w,i)
            r = rospy.Rate(hz)
            r.sleep()
            i = i + 1
