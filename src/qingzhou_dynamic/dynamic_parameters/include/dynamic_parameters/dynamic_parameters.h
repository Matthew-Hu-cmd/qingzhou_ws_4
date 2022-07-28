/******************************************
文件名：dynamic_parameters.h

功  能：进行动态调参

作者：胡杨
******************************************/

#ifndef __DYNAMIC_PARAMETERS__
#define __DYNAMIC_PARAMETERS__

#include <ros/ros.h>
#include <stdio.h>
#include <boost/function.hpp>
#include <std_srvs/Empty.h>
#include <std_msgs/Int32.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include "qingzhou_locate/RobotLocation.h"
#include "dynamic_reconfigure/config_tools.h"
#include "dynamic_reconfigure/client.h"
#include "qingzhou_nav/L1_dynamicConfig.h"


struct ParametersConfig
{
	qingzhou_nav::L1_dynamicConfig config1;
	qingzhou_nav::L1_dynamicConfig config2;
	qingzhou_nav::L1_dynamicConfig config3;
	qingzhou_nav::L1_dynamicConfig config4;
	qingzhou_nav::L1_dynamicConfig config5;
	ParametersConfig(){
		config1.L = 0.23;
		config1.Lrv = 1.0;
		config1.Vcmd = 0.7;
		config1.Lfw = 1.0;
		config1.lfw = 0.13;
		config1.lrv = 10.0;   
		config1.controller_freq = 20.0;
		config1.angle_gain = -1.65;
		config1.gas_gain = 1.0;
		config1.base_speed = 1.2;
		config1.base_angle = 0.0;
		config1.goal_dist = 0.3;

		config2.L = 0.26;
		config2.Lrv = 1.0;
		config2.Vcmd = 0.7;
		config2.Lfw = 1.0;
		config2.lfw = 0.13;
		config2.lrv = 10.0;
		config2.controller_freq = 20.0;
		config2.angle_gain = -1.8;
		config2.gas_gain = 1.0;
		config2.base_speed = 1.0;
		config2.base_angle = 0.0;
		config2.goal_dist = 0.3;

		config3.L = 0.26;
		config3.Lrv = 1.0;
		config3.Vcmd = 0.7;
		config3.Lfw = 1.0;
		config3.lfw = 0.13;
		config3.lrv = 10.0;
		config3.controller_freq = 20.0;
		config3.angle_gain = -1.8;
		config3.gas_gain = 1.0;
		config3.base_speed = 1.2;
		config3.base_angle = 0.0;		
		config3.goal_dist = 0.3;
		
		config4.L = 0.3;
		config4.Lrv = 0.8;
		config4.Vcmd = 0.7;
		config4.Lfw = 0.8;
		config4.lfw = 0.13;
		config4.lrv = 10.0;
		config4.controller_freq = 20.0;
		config4.angle_gain = -1.7;
		config4.gas_gain = 1.2;
		config4.base_speed = 1.2;
		config4.base_angle = 0.0;
		config4.goal_dist = 0.01;

		config5.L = 0.3;
		config5.Lrv = 1.21;
		config5.Vcmd = 0.7;
		config5.Lfw = 0.825;
		config5.lfw = 0.13;
		config5.lrv = 10.0;
		config5.controller_freq = 20.0;
		config5.angle_gain = -1.8;
		config5.gas_gain = 1.0;
		config5.base_speed = 1.2;
		config5.base_angle = 0.0;
		config5.goal_dist = 0.3;

	}
};



enum RobotLocation : int
{
	Start, Load, TrafficLight ,Unload, RoadLine, 
	RoadLineToStart, StartToLoad, LoadToTrafficLight, TrafficLightToUnload, UnloadToRoadLine,
	Unknown
};


class DynamicParameters
{
private:
	ros::NodeHandle nh;
	ros::Subscriber locate_sub;
	ros::ServiceClient map_client;

	RobotLocation robotLocation;
	ParametersConfig paramConfig;
	dynamic_reconfigure::Config costmapConfLoad;
	dynamic_reconfigure::Config costmapConfOther;
	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;


	std_srvs::Empty empty;
	dynamic_reconfigure::Client<qingzhou_nav::L1_dynamicConfig>* client;

	void locateCB(const std_msgs::Int32& data);

public:
	DynamicParameters();
	~DynamicParameters();
	void setParameters(qingzhou_nav::L1_dynamicConfig& config);
	void initCostmapConf();
	void setCostmapConf(dynamic_reconfigure::Config& conf);
};

void dynCallBack(const qingzhou_nav::L1_dynamicConfig &data);


#endif
