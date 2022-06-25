# from launch_demo import launch_demo
import rospy
import subprocess
import rosnode
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from geometry_msgs.msg import PoseStamped
from math import pi
from std_msgs.msg import Int16, Bool
import time
from ackermann_msgs.msg import AckermannDrive


class navigation_demo:
    def __init__(self):
        # self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        # nav
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        self.sub_socket = rospy.Subscriber('/socket', Int16, self.socket_cb)
        # traffic light
        self.sub_traffic = rospy.Subscriber('/traffic_light', Bool, self.traffic_light)
        # line check
        self.pub_line = rospy.Publisher('/detector_line',Bool,queue_size=10)
        self.pub_color = rospy.Publisher('/detector_trafficlight',Bool,queue_size=10)
        self.pub_reached = rospy.Publisher('/reached',Bool,queue_size=10)
        self.sub_done = rospy.Subscriber('/done',Bool,self.done_cb)
        
        self.count = 0
        self.judge = 0    
        self.start = 0
        self.end = 0
        self.traffic = False
        self.control = 0
        self.step = 0
        # self.done = False


    def traffic_light(self, color):
        self.traffic = color.data
        # self.traffic = True
        if (self.traffic == False):
            print ("traffic red")
            self.judge = 0
        if (self.traffic == True):
            print ("traffic green")
            self.judge = 1


    def socket_cb(self, msg):
        pdX = [1.965, 2.031, 1.902, 1.832, -0.417, -1.902, -2.389, 0.857, -1.025, -1.965, -1.469, 0.172]
        pdY = [-1.286, -2.488, -4.566, -6.088, -7.742, -7.403, -5.408, -4.086, -2.178, -0.754, -0.125, 0.002]
        pdth = [0,0,0,0,0,0,0,0,0,0,0,0]
        sec = [3.21, 2.22, 4.2, 5.89, 3.26, 3.22, 4.22, 15.38, 4.32, 2.92, 4.05]

        if (msg.data == 1):
            print("Going to loading port:", msg.data)
            self.step = 0
            for self.step in range(0,3):
                w = [pdX[self.step], pdY[self.step], pdth[self.step]]
                self.goto([w[0], w[1], w[2]])
                r = rospy.Rate(1/sec[self.step])
                r.sleep()
                self.step = self.step + 1

        elif (msg.data == 2):
            print("Going to unloading port:", msg.data)
            # print(self.step)
            self.pub_color.publish(True)
            w = [pdX[self.step], pdY[self.step], pdth[self.step]]
            self.goto([w[0], w[1], w[2]])
            r = rospy.Rate(1/sec[self.step])
            r.sleep()
            self.setp = self.step + 1
            print("-------------traffic detector---------------")
            active = True
            while (active):
                if (self.judge == 1):
                    active = False
                    print("you can go")
                    break
                if (self.judge == 0):
                    active = True
            self.pub_color.publish(False)
            for self.step in range(4,7):
                w = [pdX[self.step], pdY[self.step], pdth[self.step]]
                self.goto([w[0], w[1], w[2]])
                r = rospy.Rate(1/sec[self.step])
                r.sleep()
                self.step = self.step + 1

            # w = [pdX[4], pdY[4], pdth[4]]
            # self.goto([w[0], w[1], w[2]])
            # r = rospy.Rate(1/sec[4])
            # r.sleep()
            
            # w = [pdX[5], pdY[5], pdth[5]]
            # self.goto([w[0], w[1], w[2]])
            # r = rospy.Rate(1/sec[5])
            # r.sleep()

            # w = [pdX[6], pdY[6], pdth[6]]
            # self.goto([w[0], w[1], w[2]])
            # r = rospy.Rate(1/sec[6])
            # r.sleep()
    
            

               
        elif (msg.data == 3):
            print("Come back qingzhou_robot:", msg.data)
            w = [pdX[7], pdY[7], pdth[7]]
            self.goto([w[0], w[1], w[2]])
            r = rospy.Rate(1/sec[7])
            r.sleep()
            ark_contrl= AckermannDrive() 
            ark_contrl.steering_angle = 0.0
            ark_contrl.speed = 0.0
            cmd_vel_pub = rospy.Publisher('/ackermann_cmd',AckermannDrive,queue_size=10)
            cmd_vel_pub.publish(ark_contrl)

            print("-------------line detector-------------")
            rosnode.kill_nodes(['L1_controller_v2'])
            r = rospy.Rate(0.5)
            print("OK")
            self.pub_line.publish(True)
            self.pub_reached.publish(True)
            # self.pub_line.publish(True)
            # self.pub_reached.publish(True)
            r = rospy.Rate(1/12.0)
            r.sleep()
            subprocess.Popen(["rosrun","qingzhou_nav","L1_controller_v2","__name:=L1_controller_v2","_angle_gain:=-1.8"])
            r = rospy.Rate(1)
            



        else:
            pass


    def goto(self, p):
        # goal = MoveBaseGoal()
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = p[0]
        goal.pose.position.y = p[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2] / 180.0 * pi)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]
        self.pub_goal.publish(goal)
        return True

    def done_cb(self,msg):
        self.done = msg.data

        print (self.done)
        if self.done is True:
            

            self.pub_reached.publish(False)
            # r = rospy.Rate(1)
            # r.sleep()
            # w = [-1.469, -0.125 , 0]
            # self.goto([w[0], w[1], w[2]])
            # self.pub_line.publish(False)
            w = [-1.880, -0.392 , 0]
            self.goto([w[0], w[1], w[2]])
            self.pub_line.publish(False)
            r = rospy.Rate(1.0)
            r.sleep()
            w = [-0.072, -0.016 , 0]
            self.goto([w[0], w[1], w[2]])
            self.pub_line.publish(False)




def main():
    rospy.init_node('navigation_demo', anonymous=True)
    nav = navigation_demo()
    rospy.spin()


if __name__ == "__main__":
    main()

