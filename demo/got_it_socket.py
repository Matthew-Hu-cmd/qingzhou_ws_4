#coding:utf-8

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf_conversions
import sys
import socket
import threading
from std_msgs.msg import Int16
from time import sleep
import struct
import tf
import signal
###################以下是导航设置##################
import rospy
# import actionlib
# from actionlib_msgs import *
# import geometry_msgs
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose,PoseWithCovarianceStamped,Point ,Quaternion,Twist
# from move_base_msgs.msg import MoveBaseAction ,MoveBaseGoal
#from radom import sample
from math import pow ,sqrt

sock = socket.socket()
 
# move_base = actionlib.SimpleActionClient("move_base",MoveBaseAction)  # Subscribe to the move_base action server
# goal = MoveBaseGoal()
# pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
pub_socket = rospy.Publisher('socket',Int16,queue_size=10)

simple_goal = PoseStamped()

def quit():
    sys.exit()

class SockClient(threading.Thread):
    def __init__(self, host_ip, host_port):
        threading.Thread.__init__(self)
        self.running = False
        #sock.settimeout(30)  # 1min
        try:
            sock.connect((host_ip, host_port))
        except socket.error:
            print("Socket Connect Error")
            exit(1)
        print("connect success")
        self.running = True
        self.error_cnt = 0

    def run(self):
      
        while self.running:
            try:
                #time.sleep(2)
                data = sock.recv(1024)
                data = data[:269] #269
                # print(len(data))
                if len(data) > 0:
                    self.error_cnt = 0
                    recv_data = struct.unpack('4sic256s4s', data)
                    #print('recv:', recv_data)
                    str_len = recv_data[1]-5 #长度
                    para = recv_data[3][:str_len]   #x,y
                    para = struct.unpack('<2f', para)
                    command = recv_data[2]
                    #print('strlen:',str_len)
                    #print('para:',para)
                    #print('command',command)
                    if command == b'\x10':
                        print("where do u want to go?")
                        x=input("shuru:x")
                        y=input("shuru:y")
                        # goal.target_pose.header.frame_id ='map'
                        # goal.target_pose.header.stamp = rospy.Time.now()

                        simple_goal.header.frame_id = 'map'
                        simple_goal.header.stamp = rospy.Time.now()

                        rospy.loginfo("Going to somewhere: " + str(para))
                        # move_base.send_goal(goal)
                        #pub.publish(simple_goal)
                        pub_socket.publish(0)
                        # move_base.wait_for_result()
                        print('go')

                    elif command == b'\x20':
                        # move_base.cancel_goal()  ####我在找暂停的办法，这里是取消目标点，也就是说新开始的时候要再设置一次终点
                        print('stop')
                    elif command == b'\x30':
                        q = tf.transformations.quaternion_from_euler(0,0,0)
                        print("quaternion_trans_from_euler:", q)
                        # goal.target_pose.pose = Pose(Point(para[0],para[1], 0.0), Quaternion(q[0], q[1], q[2], q[3]))############目标点设置
                        # goal.target_pose.header.frame_id = 'map'  
                        # goal.target_pose.header.stamp = rospy.Time.now()###########记录时间戳
                        
                        simple_goal.pose = Pose(Point(para[0],para[1], 0.0), Quaternion(q[0], q[1], q[2], q[3]))############目标点设置
                        simple_goal.header.frame_id = 'map'
                        simple_goal.header.stamp = rospy.Time.now()

                        print(para[0], para[1])
                        rospy.loginfo("Going to yuandian: " + str(para)) 
                        # move_base.send_goal(goal)#######发送目标
                        #pub.publish(simple_goal)
                        pub_socket.publish(3)
                        # move_base.wait_for_result()
                    elif command == b'\x40':
                        q = tf.transformations.quaternion_from_euler(0,0,-1.557)
                        # goal.target_pose.pose = Pose(Point(para[0], para[1], 0.0), Quaternion(q[0], q[1], q[2], q[3]))    
                        # goal.target_pose.header.frame_id = 'map'  
                        # goal.target_pose.header.stamp = rospy.Time.now()  

                        simple_goal.pose = Pose(Point(para[0],para[1], 0.0), Quaternion(q[0], q[1], q[2], q[3]))############目标点设置
                        simple_goal.header.frame_id = 'map'
                        simple_goal.header.stamp = rospy.Time.now()

                        rospy.loginfo("Going to jiehuo: " + str(para)) 
                        print(para[0], para[1])
                        # move_base.send_goal(goal)
                        # pub.publish(simple_goal) 
                        pub_socket.publish(1)
                        # move_base.wait_for_result()
                        
                    elif command == b'\x50':
                        q = tf.transformations.quaternion_from_euler(0,0,1.583)
                        # goal.target_pose.pose = Pose(Point(para[0], para[1], 0.0), Quaternion(q[0], q[1], q[2], q[3]))    
                        # goal.target_pose.header.frame_id = 'map'  
                        # goal.target_pose.header.stamp = rospy.Time.now()  

                        simple_goal.pose = Pose(Point(para[0],para[1], 0.0), Quaternion(q[0], q[1], q[2], q[3]))############目标点设置
                        simple_goal.header.frame_id = 'map'
                        simple_goal.header.stamp = rospy.Time.now()

                        rospy.loginfo("Going to xiehuo: " + str(para)) 
                        print(para[0],para[1])
                        # move_base.send_goal(goal) 
                        # pub.publish(simple_goal)
                        pub_socket.publish(2)
                        # move_base.wait_for_result()
                        
                    else:
                        print('restart')
            except socket.error:
                self.running = False
                print ('socket running error')
                break
        
        print ('SockClient Thread Exit\n')
        ros.spin() ############这里为了保持python持续运行

class Worker(threading.Thread):#副线程
    def __init__(self,parent=None):
        super(Worker, self).__init__(parent)

        super(Worker, self).__init__(parent)
        self.num=0                                 ####这个位置定义类的成员变量，方便传入参数 #int 小车编号
        self.x=0.0                                   #float x坐标
        self.y=0.0                                   #float y坐标
        self.angle=0.0                               #float 角度
        self.angle_speed=0.0                         #float 角速度
        self.speed=0.0                               #float 线速度
############以上定义为了防止调用结束后销毁变量，也许有更好的解决办法##########
        self.status=b''                          ####这两个位置暂时不知道从哪里获取信息，在写
        self.navigation=b'' 
    def run(self): 
        rospy.loginfo("i'm using the socket")
        while (sock_client.running and not rospy.is_shutdown()):
            format = '<4si5f2c4s'
            header = b'\x02 \x02 '
            end = b'\x030\x030'
###############下面这个部分不知道对参数是怎么打包的，如果我定义了成员变量还能不能成功赋值pack,如果不可以就再从成员变量传过来一次######################################
            # '''
            # num = num_s       
            # x =   x_s     
            # y =   y_s          
            # angle =  angle_s        
            # angle_speed =  angle_speed_s  
                  
            # status = b''     #byte 车辆状态 0开机初始化、1到达停车点、2自动导航、3开机等待
            # navigation = b'' #byte 导航状态 0初始化、1交通灯、2升降杆、3S路、4自主导航
    
            # string = 	 struct.pack(format,header,num,x,y,angle,angle_speed,speed,status,navigation,end)
            # '''
###################################################################################################################################################################   
            string = struct.pack(format,header,1,10.0,11.0,12.0,13.0,14.0,b'\x00',b'\x01',end)
            sock.send(string)
            sleep(10)#10s更新一次


class got_it_socket:
    def __init__(self):
        self.tf_listener = tf.TransformListener()              ####创建监听器
        self.dx=0.0                       ###普通的速度求解
        self.dy =0.0
        self.dth=0.0      
        self.Vx=0.0
        self.Vy=0.0
        self.Vth=0.0
        self.x_last=0.0
        self.y_last=0.0
        self.th_last=0.0
        self.last_seconds=0
        self.speed=0.0
        try:
            self.tf_listener.waitForTransform('/map','/base_link',rospy.Time(),rospy.Duration(1.0))   #####等待从baselink 到map坐标的变换响应
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return
    def get_pose(self):   ############位姿获取核心函数
        try:
            (tran,rot)=self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))   #####从baselink 到map坐标的变换
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf Error")
            return None
	 #euler = transformations.euler_from_quaternion(rot)   ####同时适应不同维度的环境
         #x = trans[0]   #x坐标
         #y = trans[1] 
         #th = euler[2] / pi * 180   #角度
         #dt=rospy.get_time()-last_seconds;  ####记录时间差
	 #self.last_seconds = rospy.get_time();   ###得到一个时间戳
         #self.dx = x-x_last                      ###普通的速度求解
         #self.dy = y-y_last
         #self.dth=th-th_last
        x=0.0
        y=0.0
        th=0.0
        dt=0
        #dt=rospy.get_time()-last_seconds;  ####记录时间差
        self.last_seconds = 0   ###得到一个时间戳
        self.dx = 0                      ###普通的速度求解
        self.dy = 0
        self.Vx = 0
        self.Vy=0
        self.Vth = 0
        #self.Vx = dx/dt
        #self.Vy = dy/dt
        #self.Vth = dth/dt
        self.x_last = x      #记录此次位置
        self.y_last = y
        self.th_last=th
        #self.speed =  (Vx*Vx+Vy*Vy)**0.5
        self.speed=0
        return (x, y, th,self.Vx,self.Vy,self.Vth,self.speed)



if __name__ == "__main__":

    rospy.init_node('get_pos_demo',anonymous=True)   ####初始化节点
    #print "asdasd"
#################################以下为ros调度部分###############################
    #cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)  
    rospy.loginfo("Waiting for move_base action server...") 
    # move_base.wait_for_server(rospy.Duration(60))   #############等待初始化成功
    rospy.loginfo("Connected to move base server")  
    # A variable to hold the initial pose of the robot to be set by the user in RViz  
    #initial_pose = PoseWithCovarianceStamped()
    # Variables to keep track of success rate, running time, and distance traveled  
    start_time = rospy.Time.now()  
    running_time = 0  
    #rospy.wait_for_message('initialpose', PoseWithCovarianceStamped) 
    #last_location = Pose() 
    #rospy.Subscriber('initialpose', PoseWithCovarianceStamped, update_initial_pose) 
    rospy.loginfo("Starting navigation test") 

#######################################################################################

    Got_socket = got_it_socket()                     #####创建采集对象
    # print "asdasd"
    loop = rospy.Rate(20)                            #####20HZ频率
    # print "asdasd"
    loop.sleep()
    #print "asdasd"
    #sock_client = SockClient('192.168.31.191', 8888)
    sys.stdout.flush()
    sock_client = SockClient('192.168.1.10', 8888)
    #print "asdasd"
    sock_client.start()
    ros = Worker()
    ros.start()
    #print "asdasd"
    
    try:
        signal.signal(signal.SIGINT,quit)
        signal.signal(signal.SIGTERM,quit)
        while True:
            sleep(1)
            #print "asdasd"
            loop.sleep()        ###########记录时间戳
            (x0,y0,th0,Vx0,Vy0,Vth0,V0) = Got_socket.get_pose()
            ros.x  = x0
            ros.y = y0
            ros.angle = th0
            ros.angle_speed =Vth0
            ros.speed = V0
            if not sock_client.is_alive():
                break
    except KeyboardInterrupt:
        print ('ctrl+c')
        sock_client.running = False
    sock_client.join()
    print ('exit finally')