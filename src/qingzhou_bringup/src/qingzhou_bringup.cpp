#include "qingzhou_bringup/qingzhou_bringup.h"

long long LeftticksPerMeter = 0;    
long long rightticksPerMeter = 0;   
long long LeftticksPer2PI = 0;      
long long rightticksPer2PI = 0;     

// 构造函数，初始化
actuator::actuator(ros::NodeHandle handle) 
{ 
   m_baudrate = 115200;
   m_serialport = "/dev/ttyUSB0";   
   linearSpeed = 0;
   angularSpeed = 0;
   batteryVoltage = 0;
   ticksPerMeter = 0;
   ticksPer2PI = 0;
   encoderLeft = 0;
   encoderRight = 0;
   velDeltaTime = 0;
   calibrate_lineSpeed = 0;
   calibrate_angularSpeed = 0;
   line = 0;
   angle = 0;
   reach = false;

   memset(&moveBaseControl,0,sizeof(sMartcarControl));
   
   handle.param("mcubaudrate",m_baudrate,m_baudrate);                                  
   handle.param("mcuserialport",m_serialport,std::string("/dev/ttyUSB0"));              
   handle.param("calibrate_lineSpeed",calibrate_lineSpeed,calibrate_lineSpeed);         
   handle.param("calibrate_angularSpeed",calibrate_angularSpeed,calibrate_angularSpeed);
   handle.param("ticksPerMeter",ticksPerMeter,ticksPerMeter);                           
   handle.param("ticksPer2PI",ticksPer2PI,ticksPer2PI);                                 

    try{ 
      std::cout<<"[qingzhou_actuator-->]"<<"Serial initialize start!"<<std::endl;              
      ser.setPort(m_serialport.c_str());                                               
      ser.setBaudrate(m_baudrate);                                                    
      serial::Timeout to = serial::Timeout::simpleTimeout(30);                      
      ser.setTimeout(to);                                                            
      ser.open();                                                                     
    }
    catch (serial::IOException& e){
      std::cout<<"[qingzhou_actuator-->]"<<"Unable to open port!"<<std::endl;                
    }
    if(ser.isOpen()){
      std::cout<<"[qingzhou_actuator-->]"<<"Serial initialize successfully!"<<std::endl;       
    }
    else{
      std::cout<<"[qingzhou_actuator-->]"<<"Serial port failed!"<<std::endl;                  
    } 

    sub_move_base = handle.subscribe("ackermann_cmd",1,&actuator::callback_move_base,this);   
    sub_reached = handle.subscribe("/detector_line",1,&actuator::callback_reached,this);
    sub_reach = handle.subscribe("/reached",1,&actuator::reach_cb,this);
    pub_imu = handle.advertise<sensor_msgs::Imu>("raw", 5);	                                                 
    pub_mag = handle.advertise<sensor_msgs::MagneticField>("imu/mag", 5);                                    
    pub_odom = handle.advertise<nav_msgs::Odometry>("odom", 5);                                               
    pub_battery = handle.advertise<std_msgs::Float32>("battery",10);                                          
}

//析构函数
actuator::~actuator() 
{
    
}

void actuator::reach_cb(const std_msgs::Bool::ConstPtr &msg)
{
    reach = msg->data;
}
void actuator::callback_reached(const std_msgs::Bool::ConstPtr &msg)
{
    if (msg->data == true){line = 20;}
    else {line = 0;}
}

void actuator::callback_move_base(const ackermann_msgs::AckermannDrive::ConstPtr &msg)//对应cmd_vel话题，ackermann_msgs/AckermannDrive
{
   memset(&moveBaseControl,0,sizeof(sMartcarControl));                       //清零movebase数据存储区
   


   moveBaseControl.TargetSpeed =  msg->speed*32/0.41;
   moveBaseControl.TargetSpeed =  moveBaseControl.TargetSpeed - line;                                 //计算目标线速度
   moveBaseControl.TargetAngle =  msg->steering_angle;                 //计算目标角度
   moveBaseControl.TargetAngle+=60;                                          //stm32 program has subtract 60
   if (reach == false){
        if(moveBaseControl.TargetAngle >= 75 || moveBaseControl.TargetAngle <= 45){
            moveBaseControl.TargetSpeed = moveBaseControl.TargetSpeed - 16;
            angle = 16;
            }
        else if(moveBaseControl.TargetAngle >= 70 || moveBaseControl.TargetAngle <= 50){
                moveBaseControl.TargetSpeed = moveBaseControl.TargetSpeed - 14;
            angle = 14;
            }
   }


   if(moveBaseControl.TargetAngle < 25){
      moveBaseControl.TargetAngle = 25;            //角度小于30
   }
   if(moveBaseControl.TargetAngle > 95){
      moveBaseControl.TargetAngle = 95;           //角度最大90
   }

   printf("%d,%d       %d,%d\n",abs(moveBaseControl.TargetSpeed),abs(moveBaseControl.TargetAngle),angle,line);	   

}


  
void actuator::run()
{
    int run_rate = 50;              
    ros::Rate rate(run_rate);      

    double x = 0.0;                 //x坐标                       
    double y = 0.0;                 //y坐标
    double th = 0.0;

    ros::Time current_time, last_time;                    
	
    while(ros::ok()){
    ros::spinOnce();                                  
	current_time = ros::Time::now();                  //获得当前时间
	velDeltaTime = (current_time - last_time).toSec();//转换成秒
	last_time = ros::Time::now();                    
		
    recvCarInfoKernel();                              //接收stm32发来的数据
	pub_9250();                                       //发布imu数据
		
	currentBattery.data = batteryVoltage;            
	pub_battery.publish(currentBattery);             

    #if 1
	if(encoderLeft > 220 || encoderLeft < -220) encoderLeft = 0;
	if(encoderRight > 220 || encoderRight < -220) encoderRight = 0;
        //encoderLeft = -encoderLeft;
	encoderRight = -encoderRight;                          

	detEncode = (encoderLeft + encoderRight)/2;            
	detdistance = detEncode/ticksPerMeter;                 
	detth = (encoderRight - encoderLeft)*2*PI/ticksPer2PI; //计算当前角度,通过标定获得ticksPer2PI

	linearSpeed = detdistance/velDeltaTime;                
	angularSpeed = detth/velDeltaTime;                     
		
	if(detdistance != 0){
	    x += detdistance * cos(th);                        //x坐标
	    y += detdistance * sin(th);                        //y坐标
	}
	if(detth != 0){
	    th += detth;                                       //总角度
	}
       
	if(calibrate_lineSpeed == 1){
		printf("x=%.2f,y=%.2f,th=%.2f,linearSpeed=%.2f,,detEncode=%.2f,LeftticksPerMeter = %lld,rightticksPerMeter = %lld,batteryVoltage = %.2f\n",x,y,th,linearSpeed,detEncode,LeftticksPerMeter,rightticksPerMeter,batteryVoltage);
	}
		
	//send command to stm32
	sendCarInfoKernel();                                                     
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th); 

	nav_msgs::Odometry odom;                               //创建nav_msgs::Odometry类型的消息odom
	odom.header.stamp = current_time;                     
	odom.header.frame_id = "odom";                         
 	odom.child_frame_id = "base_link";                     

	//set the position
	odom.pose.pose.position.x = x;                       
	odom.pose.pose.position.y = y;                        
	odom.pose.pose.position.z = 0.0;                       
	odom.pose.pose.orientation = odom_quat;               
	
	odom.twist.twist.linear.x = linearSpeed;               //线速度
	odom.twist.twist.linear.y = 0;
	odom.twist.twist.linear.z = 0;
	odom.twist.twist.angular.x = 0;
	odom.twist.twist.angular.y = 0;
	odom.twist.twist.angular.z = angularSpeed;             //角速度
	if(encoderLeft == 0 && encoderRight == 0){
	    odom.pose.covariance = {1e-9, 0, 0, 0, 0, 0,       
				                0, 1e-3, 1e-9, 0, 0, 0,
				                0, 0, 1e6, 0, 0, 0,
				                0, 0, 0, 1e6, 0, 0,
				                0, 0, 0, 0, 1e6, 0,
				                0, 0, 0, 0, 0, 1e-9};

		odom.twist.covariance = {1e-9, 0, 0, 0, 0, 0,      
						        0, 1e-3, 1e-9, 0, 0, 0,
						        0, 0, 1e6, 0, 0, 0,
						        0, 0, 0, 1e6, 0, 0,
						        0, 0, 0, 0, 1e6, 0,
						        0, 0, 0, 0, 0, 1e-9};
		}
	else{
		odom.pose.covariance = {1e-3, 0, 0, 0, 0, 0,       
						        0, 1e-3, 0, 0, 0, 0,
						        0, 0, 1e6, 0, 0, 0,
						        0, 0, 0, 1e6, 0, 0,
						        0, 0, 0, 0, 1e6, 0,
						        0, 0, 0, 0, 0, 1e3};

		odom.twist.covariance = {1e-3, 0, 0, 0, 0, 0,      
						        0, 1e-3, 0, 0, 0, 0,
						        0, 0, 1e6, 0, 0, 0,
						        0, 0, 0, 1e6, 0, 0,
						        0, 0, 0, 0, 1e6, 0,
						        0, 0, 0, 0, 0, 1e3};
		}
	pub_odom.publish(odom);                                                            
	
#endif
    rate.sleep();
    }
}

//发送小车数据到下位机
void actuator::sendCarInfoKernel()
{
    unsigned char buf[23] = {0};
    buf[0] = 0xa5;	                                        
    buf[1] = 0x5a;	                                        
    buf[2] = 0x06;	                                        

    buf[3] = (int)moveBaseControl.TargetAngleDir;	    //targetangleDirection 0-->go straight,0x10-->turn left,0x20-->turn right (not used)
    buf[4] = (int)abs(moveBaseControl.TargetAngle);	    //targetangle
    buf[5] = (int)abs(moveBaseControl.TargetSpeed);	    //targetSpeed
    buf[6] = (int)moveBaseControl.TargetModeSelect;	    //0-->person control,1-->auto control (not used)
    buf[7] = (int)moveBaseControl.TargetShiftPosition;  //targetshiftposition  0-->P stop;1-->R;2-->D. (not used)
  
    buf[8] = 0;		                                        
    unsigned char sum = 0;
    for(int i = 2; i < 19; ++i)                             
        sum += buf[i];
    buf[9] = (unsigned char)(sum);                      
    size_t writesize = ser.write(buf,10);
}

//接收下位机发送来的数据
void actuator::recvCarInfoKernel()
{
    std::string recvstr;                                             
    unsigned char tempdata,lenrecv;                                  
    unsigned char count,last_data,last_last_data,last_last_last_data;
    unsigned char str[100];                                          
    bool recvflag = false;                                          
    bool recvd_flag = false;                                         
    memset(&str,0,sizeof(str));                                      
    ros::Time begin_time = ros::Time::now();                         
    double clustering_time = 0;                                      

    while(1){
	    clustering_time = (ros::Time::now () - begin_time).toSec (); //计算时间差，转换成秒
	    if(clustering_time > 1){                                    
	        recvd_flag = false;                                      
	        break;                                                   
	    }
		
	    recvstr = ser.read(1);                                      		
	    if((int)recvstr.size() != 1)                                 
	        continue;                                               
		
	    tempdata = recvstr[0];                                       
	    if(last_last_last_data == 0xa5 && last_last_data == 0x5a){   
	        lenrecv = last_data;                                     
	        recvflag = true;                                         
	        count = 0;                                               
	    }
	    if(recvflag){                                               
	        str[count] = tempdata;                                   
	        count++;                                                
	        if(count == lenrecv){                                   
		        recvflag = false;                                    
		        recvd_flag = true;                                   
		    break;                                                  
	        }
	    }
	    last_last_last_data = last_last_data;                        
	    last_last_data = last_data;                                  
	    last_data = tempdata;                                        
    }

    if(recvd_flag){                                                  //数据解析，接收到的数据转存
	    memcpy(&encoderLeft,str,4);                                  
	    memcpy(&encoderRight,str+4,4);                              
	    memcpy(&batteryVoltage,str+8,4);                           
 	
	    memcpy(&tempaccelX,str+12,2);                               
	    memcpy(&tempaccelY,str+14,2);                               
		memcpy(&tempaccelZ,str+16,2);                                
		
		memcpy(&tempgyroX,str+18,2);                                 
		memcpy(&tempgyroY,str+20,2);                                
		memcpy(&tempgyroZ,str+22,2);                                 
		
		memcpy(&tempmagX,str+24,2);                                 
		memcpy(&tempmagY,str+26,2);                                
		memcpy(&tempmagZ,str+28,2);                                

		accelX = (float)tempaccelX/2048*9.8;                         //线加速度处理	
		accelY = (float)tempaccelY/2048*9.8;
		accelZ = (float)tempaccelZ/2048*9.8;

		
		gyroX = (float)tempgyroX/16.4/57.3;                          //角速度处理
		gyroY = (float)tempgyroY/16.4/57.3;
		gyroZ = (float)tempgyroZ/16.4/57.3;
		
		magX = (float)tempmagX*0.14;                                 //磁力计处理
		magY = (float)tempmagY*0.14;
		magZ = (float)tempmagZ*0.14;

        if(encoderLeft > 220 || encoderLeft < -220) encoderLeft = 0; //判断编码器脉冲数是否在正确范围
        if(encoderRight > 220 || encoderRight < -220) encoderRight = 0;
        LeftticksPerMeter += encoderLeft;                            //获得左轮总脉冲数
        rightticksPerMeter += encoderRight;                          //获得右轮总脉冲数
    }
}

//发布imu函数
void actuator::pub_9250(){                         
    sensor_msgs::Imu imuMsg;                       
    sensor_msgs::MagneticField magMsg;           
	
    ros::Time current_time = ros::Time::now();    
	         
    imuMsg.header.stamp = current_time;            
    imuMsg.header.frame_id = "imu_link";          
    imuMsg.angular_velocity.x = gyroX;             
    imuMsg.angular_velocity.y = gyroY;             
    imuMsg.angular_velocity.z = gyroZ;             
    imuMsg.angular_velocity_covariance = {         
      0.04,0.0,0.0,
      0.0,0.04,0.0,
      0.0,0.0,0.04
    };
    
    imuMsg.linear_acceleration.x = accelX;        
    imuMsg.linear_acceleration.y = accelY;         
    imuMsg.linear_acceleration.z = accelZ;        
    imuMsg.linear_acceleration_covariance = {      
      0.04,0.0,0.0,
      0.0,0.04,0.0,
      0.0,0.0,0.04
    };
    pub_imu.publish(imuMsg);                       //发布imuMsg 
     
    magMsg.header.stamp = current_time;            
    magMsg.header.frame_id = "base_link";          
    magMsg.magnetic_field.x = magX;                
    magMsg.magnetic_field.y = magY;                
    magMsg.magnetic_field.z = magZ;                
    magMsg.magnetic_field_covariance = {           
      0.0,0.0,0.0,
      0.0,0.0,0.0,
      0.0,0.0,0.0
    };
    pub_mag.publish(magMsg);                       //发布magMsg
}
