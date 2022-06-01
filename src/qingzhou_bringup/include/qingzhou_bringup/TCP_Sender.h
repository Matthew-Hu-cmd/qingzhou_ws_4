#ifndef __TCP_SENDER_H__
#define __TCP_SENDER_H__
#include "ros/ros.h"
#include "sys/socket.h"
#include "sys/types.h"
#include <vector>
#include "errno.h"
#include "netinet/in.h"
#include "arpa/inet.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Point.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "actionlib_msgs/GoalStatus.h"
#include <actionlib/client/simple_action_client.h>
#include "tf/tf.h"
#include "tf2_ros/transform_listener.h" 
#include "fcntl.h"
#include "boost/thread.hpp"
#include "qingzhou_bringup/app.h"
#include "std_srvs/Empty.h"
#define SERVER_IP "127.0.0.1"
#define PORT 6666



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseActionClient;
// typedef actionlib::SimpleActionClietn<move_base_msgs::MoveBa


enum TRAFFICLIGHT{red = 0,green = 1,yellow = 2};
enum GOALSTATE{lost = 0,active = 1,reach = 2 ,aborted = 3};
enum ROBOTLOCATION
{
    start,
    starttoload,
    load,
    loadingtotfl,
    tfl,
    tfltounload,
    loadtounload,
    unload,
    unloadtoroadline,
    roadlinestart,
    roadline,
    roadlineout,
    unloadtostart,
    unknow
};
enum ROBOTGOALPOINT
{
    goal_start,
    goal_load,
    goal_tfl,
    goal_unload,
    goal_roadlinestart,
    goal_unkonw
};


typedef struct Point3D{
    float x;
    float y;
    float z;
    Point3D(float x = 0,float y = 0,float z = 0):x(0),y(0),z(0)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }
}point3d;


typedef struct SpeedPlanSub{
    ros::Duration _dua;//花多长时间
    geometry_msgs::Point _point;//或者到什么地点为止
    geometry_msgs::Twist _speed;
    SpeedPlanSub(float duaTime,float pointx,float pointy,float speedx,float speedz){
        _dua = ros::Duration(duaTime);
        _point.x = pointx;
        _point.y = pointy;
        _speed.linear.x = speedx;
        _speed.linear.y = _speed.linear.z = 0;
        _speed.angular.x = _speed.angular.y =  0;
        _speed.angular.z = speedz;
        _point.z = 0;
    }
}SpeedPlanPoint;

//准备废弃使用
typedef struct RobotState
{
    GOALSTATE goalstate;
    ROBOTLOCATION robotlocation;
    bool autoGoalControl;//控制是否自动连续发布目标点
    bool openTflDet;

    RobotState():
        goalstate(lost),
        robotlocation(start),
        // autoGoalControl(false),
        autoGoalControl(true),
        openTflDet(false){
        //init
    }
}robotstate;


typedef struct RobotLocalState
{
    point3d goalList[5];
    ROBOTLOCATION location;
    ROBOTGOALPOINT curruentGoal;
    GOALSTATE goalState;
    TRAFFICLIGHT trafficLightColor;
    bool autoGoalControl; //控制是否自动连续发布目标点
    bool openTflDet;
    bool openRoadLineDet;
    // RobotLocalState():location(start),curruentGoal(goal_load),goalState(lost),autoGoalControl(false),openTflDet(false),openRoadLineDet(false),
    RobotLocalState():location(start),curruentGoal(goal_load),goalState(lost),autoGoalControl(true),openTflDet(false),openRoadLineDet(false),
    goalList({point3d(0,0,0),point3d(0,0,0),point3d(0,0,0),point3d(0,0,0),point3d(0,0,0)})
    {
    }
} rls;

typedef struct RobotControlMsg //ros 发送过来的消息结构体
{
    /* data */
    float linearSpeed;//线速度，x
    float angularSpeed;//角速度
    int controlFunctionSelector;
    point3d goalList[5];
    ROBOTLOCATION location;
    ROBOTGOALPOINT curruentGoal;
    RobotControlMsg() : controlFunctionSelector(0), location(start),goalList({point3d(0,0,0),point3d(0,0,0),point3d(0,0,0),point3d(0,0,0),point3d(0,0,0)})
    {
        //init
    }
}rcm;

typedef struct RobotStatusMsg//？？？
{
    /* data */
    float bettary;//电池电量、
    float linearSpeed;//线速度，x
    float angularSpeed;//角速度
    float locationX;
    float locationY;
    float ekfX;
    float ekfY;
    float locationInMapX;
    float locationInMapY;
    float circleTime;
    float partTime;

    int trafficLight;//红绿灯 红色0 绿色1 未知-1
    int roadlineStatus;//车道线 还未检测到0 抵达车道线起点1 视觉接手控制2 退出车道线3

    GOALSTATE goalstate;
    ROBOTLOCATION robotlocation;
    bool autoGoalControl;//控制是否自动连续发布目标点
    bool openTflDet;
    bool openRoadLineDet;

    RobotStatusMsg():bettary(0),
        linearSpeed(0),
        angularSpeed(0),
        locationX(0),
        locationY(0),
        ekfX(0),
        ekfY(0),
        locationInMapX(0),
        locationInMapY(0),
        circleTime(0),
        partTime(0),
        trafficLight(-1),
        goalstate(lost),
        robotlocation(start),
        // autoGoalControl(false),
        autoGoalControl(true),
        openTflDet(false),
        openRoadLineDet(false)
        {
            //init
        }
}rsm;

class TCP_Sender
{
private:

    const std::string ipaddr = "127.0.0.1";
    const short port = 6666;
    int shSrv;//SocketHandler
    int shCli;


    // std::string recvBuff;//数据接受缓冲区
    // std::string sendBUff;//数据发送缓冲区
    
    ros::NodeHandle nh;
    ros::Subscriber bettarySuber;
    ros::Subscriber speedSuber;//suber to /cmd_vel
    // ros::Subscriber locationSuber;
    // ros::Subscriber ekfPoseSuber;
    ros::Subscriber trafficLightSuber;
    ros::Subscriber pianyiSuber;
    // ros::Subscriber locationInMapSuber;
    // ros::Subscriber movebasePoseFeedbackSuber;
    ros::Duration sleepDur;
    ros::ServiceClient visioncontrolclient;
    ros::ServiceClient clearCostmapFirstlyClient;//清    除costmap 在起点的时候
    ros::ServiceClient clearCostmapClient;//清除costmap
    ros::ServiceClient dynamicparamsclient;
    ros::ServiceClient cmdvelFilterClient;

    ros::ServiceServer appServiceServer;

    ros::Publisher _initialposePuber;
    ros::Publisher _currentGoalPuber;
    ros::Publisher _goalStatusPuber;
    ros::Publisher _locationInMapPuber;

    ros::Timer updataStateTimer;
    ros::Timer delayOneSecondTimer;
    // ros::Timer countCircleTimer;

    geometry_msgs::Pose robotPose;

    robotstate robotState;  
    sockaddr_in addrClient; 
    
    rsm robotStatusMsg;//发送到上位机的机器人状态数据
    rcm robotControlMsg;

    bool haveDetectedRedTfl;
    bool haveDetectedGreenTfl;
    bool haveDetectRL;   //是否检测到车道线 用于在进行过程中更改目标点
    bool inRoadLine;     //保证当视觉开启之后就不再请求开启视觉控制的service
    int outcount;//计算偏移量为0的次数
    float pianyibefore;
    float roadLinePianyi; //记录当前车道线的偏移量

    geometry_msgs::Pose _roadlinestartPose;

    float _ppstime1;
    float _ppstime2;
    float _ppsspeed1x;
    float _ppsspeed1z;
    float _ppsspeed2x;
    float _ppsspeed2z;

    bool _open_debug = false;

    ros::Time _startTime;
    ros::Time _tmpStartTime;
    ros::Time _redStartTime;

    std::vector<double> _countTimeList;
    std::vector<double> _circleTimeList;

    boost::thread *_tfListenThread;
    boost::thread *_watchRLStart;
    // boost::thread *_sendMsgThread;

    boost::condition_variable_any _watchRLCond;

    // int _entTime;

    void updataStateTimerCB();
    // void CountCircleTimerCB();

    void _PrintCurruentLocation();

    //监听机器人是否到达RL起始点的线程🔓
    boost::recursive_mutex watchRLMutex;

    std::string _EmumTranslator(ROBOTLOCATION value);

public:
    TCP_Sender(const ros::NodeHandle &nodeHandler);
    ~TCP_Sender();
    void _RunSpeedPlan();
    MoveBaseActionClient * moveBaseActionClientPtr;
    ros::Publisher cmdvelPuber;
    ros::Publisher cmdvelfiltedPuber;
    rls robot_local_state;
    // move_base_msgs::MoveBaseGoal startPoint;
    // move_base_msgs::MoveBaseGoal getGoodsPoint;
    // move_base_msgs::MoveBaseGoal throwGoodsPoint;
    // move_base_msgs::MoveBaseGoal userPoint;
    // move_base_msgs::MoveBaseGoal trafficLightStopLine;
    // move_base_msgs::MoveBaseGoal lineStart;

    // std::vector<move_base_msgs::MoveBaseGoal> goalList(5);

    //**********回调函数***************
    void SubBettaryInfoCB(const std_msgs::Float32::ConstPtr &msg);
    void SubSpeedCB(const geometry_msgs::Twist::ConstPtr &msg);
    void SubLocCB(const nav_msgs::Odometry::ConstPtr &msg);
    void SubEkfPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void SubLcationMapCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void SubTrafficLightCB(const geometry_msgs::Vector3::ConstPtr &msg);
    void SubLineCB(const  geometry_msgs::Vector3::ConstPtr &msg);
    void movebasePoseFeedbackCB(const move_base_msgs::MoveBaseActionFeedbackConstPtr &msg);
    void GoalDoneCB(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result);
    void GoalActiveCB();//目标点激活的回调函数
    bool AppServiceCB(qingzhou_bringup::app::Request &req,qingzhou_bringup::app::Response &res);

    //********************************




    bool SocketInit();//初始化socket
    bool SendMsg(const void* dataPtr,size_t dataSize);//封装
    bool SendRobotStatusInfo();//向上位机发送数据
    void RunGoal();//根据位置和状态选择目标点
    bool ConvertToUnlocked();
    bool RetriveMsg(void * buff, size_t buffSize);
    void roadLineControl(); //车道线控制函数
    void CloseSocket();//关闭socket
    void WaitServices()
    {
        ROS_INFO("wait for movebase action server");
        moveBaseActionClientPtr->waitForServer();//等待    服务
        ROS_INFO("Ready!");
    }
    void StopVisonControl();//请求停止视觉控制
    void ClearCostmapFirstly();//清除costmap firstly
    bool SwitchAutoGoalControlFlag();
    bool SwitchVisionControl();//开启视觉控制
    bool SwitchTflControl();

    /**
     * @description: 
     * @param {rcm} &data
     * @return {*}
     */
    void updataLocation(rcm &data);//0x0?
    void ExecUserGoalAndupdataLocation(rcm & data);//0x03



    //*************new today**************

    /**
     * @description: 
     * @param {*}
     * @return {*}
     */
    void ExecGoal();

    /**
     * @description: 
     * @param {ROBOTLOCATION} value
     * @return {*}
     */
    void updataRobotLocation(ROBOTLOCATION value);

    /**
     * @description: 
     * @param {ROBOTGOALPOINT} goal
     * @return {*}
     */
    void updataRobotCurruentGoal(ROBOTGOALPOINT goal);

    /**
     * @description: 
     * @param {*}
     * @return {*}
     */
    void RunGoal_v2();

    /**
     * @description: 
     * @param {*}
     * @return {*}
     */
    bool OpenTflDet();

    /**
     * @description: 
     * @param {*}
     * @return {*}
     */
    bool StopTflDet();

    /**
     * @description: 
     * @param {*}
     * @return {*}
     */
    bool OpenRLDet();

    /**
     * @description: 
     * @param {*}
     * @return {*}
     */
    bool StopRLDet();

    bool ClearCostmapAndWait();

    /**
     * @description: 
     * @param {point3d} *goalList
     * @return {*}
     */
    void updataRobotGoalList(point3d *goalList);


    /**
     * @description: 
     * @param {ROBOTGOALPOINT} goal
     * @return {*}
     */
    std::string _EmumTranslator(ROBOTGOALPOINT goal);
    // bool RequestVisionControl(qingzhou_bringup::app::Request &req, qingzhou_bringup::app::Response &res);

    /**
     * @brief 监听并且更新机器人的位置
     * @param robotPose 用来存储更新的位置
     * @return void
     * */
    void ListenRobotPose(geometry_msgs::Pose &robotPose);

    //监视机器人是否到达车道线起点并取消goal让视觉接管控制
    void WatchRLStartAndCancleGoal(MoveBaseActionClient *client, ros::Publisher *cmdpuber);
    void updataToReachLocation();//goaldonnecb里面reachsuccess的分支部分，为了方便手动判断目标点是否到达，当手动判断目标点到达之后就调用这个函数

    void RunGoalManually();
    // void InitializePose();

    void RunGoalManually(ROBOTLOCATION location);
};

#endif