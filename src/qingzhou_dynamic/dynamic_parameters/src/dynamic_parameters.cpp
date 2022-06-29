/******************************************
文件名：dynamic_parameters.cpp

功  能：进行动态调参

作者：胡杨
******************************************/
#include "dynamic_parameters.h"

int main(int argc, char** argv)
{
	setlocale(LC_ALL,"");

	ros::init(argc, argv, "dynamic_parameters");

	DynamicParameters dynamicParameters;
	ros::spin();

	return 0;
}

DynamicParameters::DynamicParameters()
{
	client = new dynamic_reconfigure::Client<qingzhou_nav::L1_dynamicConfig> ("/L1_controller_v3", dynCallBack);
	// paramConfig = new ParametersConfig;

	setParameters(paramConfig.config1);
}

DynamicParameters::~DynamicParameters()
{
	delete client;
}

void DynamicParameters::setParameters(qingzhou_nav::L1_dynamicConfig& config)
{
	if (ros::param::param("DeBug", true) && client->setConfiguration(config))
	{
		ROS_INFO("Parameters Set L : %f, Lrv : %f, Vcmd : %f, lfw : %f, lrv : %f, controller_freq : %f, angle_gain : %f, gas_gain : %f, base_speed : %f, base_angle : %f", 
			config.L, config.Lrv, config.Vcmd, config.lfw, config.lrv, 
			config.controller_freq, config.angle_gain, config.gas_gain, config.base_speed, config.base_angle);
	}
	else if(ros::param::param("DeBug", true))
	{
		ROS_ERROR("Parameters Set Failed!");
	}
}

void dynCallBack(const qingzhou_nav::L1_dynamicConfig &data)
{
	ROS_INFO("Set Parameters");
}

