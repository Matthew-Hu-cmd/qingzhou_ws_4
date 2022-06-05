//******************************
//文件名：robot_states_msgs.h
//功  能：定义发送给客户端的数据
//作  者：胡杨
//******************************
#ifndef __ROBOT_STATES_MSGS__
#define __ROBOT_STATES_MSGS__


//存放机器人状态信息的结构体
typedef struct RobotStatesMsgs
{
	float battary;			//电池电压
	float linearSpeed;		//线速度
    float angularSpeed;		//角速度
    float locationX;		//X坐标
    float locationY;		//Y坐标
	RobotStatesMsgs() : battary(0), 
	linearSpeed(0), 
	angularSpeed(0), 
	locationX(0), 
	locationY(0) {}
}RobotStatesMsgs;


#endif