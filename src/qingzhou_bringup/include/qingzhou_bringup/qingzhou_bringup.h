

#ifndef NODE_EXAMPLE_TALKER_H
#define NODE_EXAMPLE_TALKER_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

#include <iostream>
#include <math.h>
#include <string.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include "ackermann_msgs/AckermannDrive.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

//lib
#include <serial/serial.h>
#include <time.h>

#define CARL 0.285
#define CARW 0.35
#define PI 3.14159265
using namespace std;
using namespace boost::asio; 
using namespace boost;

typedef struct sMartcarControl{
  int TargetAngleDir;     //期望转向角度符号 0:直行；0x10:左转；0x20:右转；
  int TargetAngle;        //期望角度
  int TargetSpeed;        //期望速度
  int TargetModeSelect;   //模式选择
  int TargetShiftPosition;
  bool control;
}sMartcarControl;        

 
class actuator
{
public:
  //! Constructor.
  actuator(ros::NodeHandle nh);
  ~actuator();
  
  void run();
public:
     int m_baudrate;            
     int m_deviceName;         
     int m_runningmode;         //运行模式
     int bdebug;
     int cartype;
     std::string updatelog;
     std::string m_serialport;  //对应USB端口
	 std::string ackermann_cmd_str;	//对应接收控制信息的话题名
     
     int encoderLeft;           //左编码器
     int encoderRight;          //右编码器
     int calibrate_lineSpeed;   //标定线速度
     int calibrate_angularSpeed;//标定角速度
     float ticksPerMeter;       //一米脉冲数
     float ticksPer2PI;         //每圈脉冲数
     float linearSpeed;         //线速度
     float angularSpeed;        //角速度
     float batteryVoltage;      //电池电压
     int line;
     int angle;
     bool reach;
       
     short tempaccelX,tempaccelY,tempaccelZ;     //加速度缓存区
     short tempgyroX,tempgyroY,tempgyroZ;        //角速度缓存区
     short tempmagX,tempmagY,tempmagZ;           //磁力计缓存区
     double accelX,accelY,accelZ;                //加速度 
     double gyroX,gyroY,gyroZ;                   //角速度 
     double magX,magY,magZ;                      //磁力计 
     int distanceA,distanceB,distanceC,distanceD;//超声波距离值
     float imuYaw;                               
	   double roll, pitch, yaw;                
     double velDeltaTime;                        //时间，存放转换成秒的时间
     double detdistance,detth;                   //计算距离和计算角度角度
     double detEncode;                           
     sMartcarControl carParasControl;            //根据之前定义的结构体，声明小车控制数据，未使用？
  
     ros::NodeHandle m_handle;                   //声明ros节点句柄
     serial::Serial ser;                         //声明串口ser
	 
     //msg
     sMartcarControl  moveBaseControl;          
     std_msgs::Float32  currentBattery;          //当前电压
     std_msgs::Float32 currentAngleMsg;          //当前角速度

	 //订阅话题
     ros::Subscriber sub_imudata;        
     ros::Subscriber sub_move_base;      //订阅move_base
     ros::Subscriber sub_movebase_angle; //订阅movebase角度
     ros::Subscriber sub_roadlinePoints; //订阅线速度
     ros::Subscriber sub_reached;
     ros::Subscriber sub_reach;
     
     
	 //发布话题
     ros::Publisher pub_actuator;        
     ros::Publisher pub_odom;            //发布odom
     ros::Publisher pub_imu;             //发布imu
     ros::Publisher pub_mag;             //发布磁力计
     ros::Publisher poly_pub;            //发布顶点
     ros::Publisher pub_battery;         //发布电池
     
     void pub_9250();           //发布9250函数
     void recvCarInfoKernel();  //接收下位机发来数据函数
     void sendCarInfoKernel();  //发送小车数据到下位机函数
  

     void callback_imuData(const sensor_msgs::Imu::ConstPtr &msg);          //imu数据回调函数 未使用
     void callback_move_base(const ackermann_msgs::AckermannDrive::ConstPtr &msg);    //move_base回调数据
     void callback_reached(const std_msgs::Bool::ConstPtr &msg);
     void callback_movebase_angle(const std_msgs::Float32::ConstPtr &msg);  //move_base回调数据
     void reach_cb(const std_msgs::Bool::ConstPtr &msg);
};




#endif // NODE_EXAMPLE_TALKER_H
