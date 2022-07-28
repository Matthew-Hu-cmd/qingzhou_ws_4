/******************************************
文件名：dynamic_parameters.cpp

功  能：进行动态调参

作者：胡杨
******************************************/

/************************************************************************************
read HY: 由于动态参数的C++ API没有说明书，这里说明一下
创建了动态参数的客户端之后，是不能直接使用getCurrentConfiguration获取当前参数的，要使用
getCurrentConfig首先需要使用setConfiguration更新一下客户端内的数据，具体为什么可以研究一下
它的源码，然后初始化客户端时，参数为：
1.参数服务器名
2.ROS句柄
3.回调函数（可不写）
4.回调函数（可不写）
当没有提供ROS句柄时，ROS句柄自动为：参数服务器名（由参数1提供）
注意：这边ROS句柄要和创建参数服务器的服务端句柄相同，否则设置不了参数
这边使用官方给的函数修改参数，同时也可以使用话题发布信息来修改参数
************************************************************************************/

#include "dynamic_parameters.h"

int main(int argc, char** argv)
{
	setlocale(LC_ALL,"");

	ros::init(argc, argv, "dynamic_parameters");
	
	DynamicParameters dynamicParameters;
	ros::spin();

	return 0;
}

/******************************************
Name ：DynamicParameters
Param: Null
Func : 构造函数
作 者 ：胡杨
******************************************/
DynamicParameters::DynamicParameters()
{
	client = new dynamic_reconfigure::Client<qingzhou_nav::L1_dynamicConfig> ("/L1_controller_v3", dynCallBack);

	//先设置初始参数，使其他函数可用
	ros::service::waitForService("/move_base/clear_costmaps");
	ros::service::waitForService("/L1_controller_v3/set_parameters");
	ros::service::waitForService("/move_base/global_costmap/inflation/set_parameters");

	map_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
	locate_sub = nh.subscribe("/qingzhou_locate", 1, &DynamicParameters::locateCB, this);

	setParameters(paramConfig.config1);
	initCostmapConf();
}

/******************************************
Name ：~DynamicParameters
Param: Null
Func : 析构函数
作 者 ：胡杨
******************************************/
DynamicParameters::~DynamicParameters()
{
	delete client;
}

/******************************************
Name ：setParameters
Param: 参数数据config
Func : 向动态参数服务器发送修改的参数
作 者 ：胡杨
******************************************/
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

/******************************************
Name ：dynCallBack
Param: Null
Func : 意义不明，还不知道怎么用
作 者 ：胡杨
******************************************/
void dynCallBack(const qingzhou_nav::L1_dynamicConfig &data)
{
	ROS_INFO("Set Parameters");
}

/******************************************
Name ：locateCB
Param: Null
Func : 当机器人位置改变时，根据改变的位置来修改
		参数
作 者 ：胡杨
******************************************/
void DynamicParameters::locateCB(const std_msgs::Int32& data)
{
	switch(data.data)
	{
		case Start:
			setParameters(paramConfig.config1);
			setCostmapConf(costmapConfOther);
			map_client.call(empty);
			break;
		case Load:
			setParameters(paramConfig.config2);
			setCostmapConf(costmapConfLoad);
			break;
		case TrafficLightToUnload:
			setParameters(paramConfig.config3);
			break;
		case UnloadToRoadLine:
			setParameters(paramConfig.config4);
			setCostmapConf(costmapConfOther);
			break;
		case RoadLineToStart:
			setParameters(paramConfig.config5);
			break;
		default:
			ROS_INFO("Keep!");
			break;
	}
}

void DynamicParameters::initCostmapConf()
{

	dynamic_reconfigure::DoubleParameter doubleParam;

	doubleParam.name = "inflation_radius";
	doubleParam.value = 1.0;
	costmapConfLoad.doubles.push_back(doubleParam);

	doubleParam.value = 0.5;
	costmapConfOther.doubles.push_back(doubleParam);
	ROS_INFO("Initialize Costmap Config");

	doubleParam.name = "cost_scaling_factor";
	doubleParam.value = 5.0;
	costmapConfOther.doubles.push_back(doubleParam);

	doubleParam.value = 30.0;
	costmapConfLoad.doubles.push_back(doubleParam);

	srv_req.config = costmapConfOther;

	if (ros::service::call("/move_base/global_costmap/inflation/set_parameters", srv_req, srv_resp)) 
	{
    	ROS_INFO("Set Costmap Config");
	}
}

void DynamicParameters::setCostmapConf(dynamic_reconfigure::Config& conf)
{
	srv_req.config = conf;

	if (ros::service::call("/move_base/global_costmap/inflation/set_parameters", srv_req, srv_resp)) 
	{
    	ROS_INFO("Set Costmap Config");
	}
	else
	{
		ROS_ERROR("Call to set costmap parameters failed");
	}
}
