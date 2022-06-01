#include "TCP_Sender.h"
#include "math.h"
#include <chrono>
#include <thread>

TCP_Sender::TCP_Sender(const ros::NodeHandle &nodeHandler)
{
    nh = nodeHandler;//获取节点句柄

    nh.param("time1",_ppstime1,4.0f);
    nh.param("time2",_ppstime2,4.0f);
    nh.param("speed1_x",_ppsspeed1x,0.5f);
    nh.param("speed1_z",_ppsspeed1z,-0.8f);
    nh.param("speed2_x",_ppsspeed2x,0.5f);
    nh.param("speed2_z",_ppsspeed2z,0.8f);

    _redStartTime = ros::Time(0);
    //订阅
    bettarySuber = nh.subscribe<std_msgs::Float32>("/battery",2,&TCP_Sender::SubBettaryInfoCB,this);//电池电量
    speedSuber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel_filted",5,&TCP_Sender::SubSpeedCB,this);//速度
    trafficLightSuber = nh.subscribe<geometry_msgs::Vector3>("/pianyi",1,&TCP_Sender::SubTrafficLightCB,this);//红绿灯
    pianyiSuber = nh.subscribe<geometry_msgs::Vector3>("/pianyi",1,&TCP_Sender::SubLineCB,this);
    //service client
    visioncontrolclient = nh.serviceClient<qingzhou_bringup::app>("/vision_control");
    dynamicparamsclient = nh.serviceClient<qingzhou_bringup::app>("/DynamicParamsClient");
    clearCostmapFirstlyClient = nh.serviceClient<std_srvs::Empty>("/clear_cost_map");
    clearCostmapClient = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    cmdvelFilterClient = nh.serviceClient<qingzhou_bringup::app>("/cmdvel_filter_client");
    //service server
    appServiceServer = nh.advertiseService("/TCP_Sender/app",&TCP_Sender::AppServiceCB,this);
    //Publisher
    cmdvelPuber = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    cmdvelfiltedPuber = nh.advertise<geometry_msgs::Twist>("/cmd_vel_filted",1);
    _goalStatusPuber = nh.advertise<actionlib_msgs::GoalStatus>("/TCP_Sender/GoalStatus", 1);
    _locationInMapPuber = nh.advertise<geometry_msgs::Pose>("/TCP_Sender/PoseInMap", 2);
    _initialposePuber = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    //Action Client
    moveBaseActionClientPtr = new MoveBaseActionClient("move_base");
    //Timre
    updataStateTimer = nh.createTimer(ros::Duration(0.065),boost::bind(&TCP_Sender::updataStateTimerCB,this),false,false);

    ROS_INFO_NAMED("TCP_Sender", "Waiting services");
    clearCostmapClient.waitForExistence();
    cmdvelFilterClient.waitForExistence();
    clearCostmapFirstlyClient.waitForExistence();
    dynamicparamsclient.waitForExistence();
    visioncontrolclient.waitForExistence();

    //Thread
    this->_tfListenThread = new boost::thread(boost::bind(&TCP_Sender::ListenRobotPose, this,boost::ref(this->robotPose))); //开启监听pose的线程
    this->_watchRLStart = new boost::thread(boost::bind(&TCP_Sender::WatchRLStartAndCancleGoal, this, this->moveBaseActionClientPtr,&this->cmdvelfiltedPuber));
    
    robot_local_state = rls();

    this->haveDetectedRedTfl = false;
    haveDetectedGreenTfl = false;
    this->_circleTimeList.clear();

    //*********init point position************
    this->_roadlinestartPose.position.x = 0.2725;
    this->_roadlinestartPose.position.y = -4.338;


    // sleepDur = ros::Duration(1);
    this->_open_debug = true;
}

TCP_Sender::~TCP_Sender()
{
    delete moveBaseActionClientPtr;
    delete _tfListenThread;
}

bool TCP_Sender::SocketInit()//初始化SOCKET
{
    shSrv = socket(AF_INET,SOCK_STREAM,0);//TCP STREAM
    if(shSrv == -1)
    {
        std::cout<<"error: socket init failed"<<std::endl;
        return false;
    }
    else
    {
        std::cout<<"log: socket inited"<<std::endl;
    }

    sockaddr_in addrSrv;

    addrSrv.sin_family = AF_INET;

    addrSrv.sin_port = htons(PORT);
   
    addrSrv.sin_addr.s_addr = INADDR_ANY;

    //reuse addr 防止服务端终止后的 timewait状态
    int optval = 1;
    setsockopt(shSrv, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    
    if(bind(shSrv,(sockaddr*)&addrSrv,sizeof(sockaddr)))
    {
        std::cout<<"error: socket bind failed"<<std::endl;
        return false;
    }
    else
    {
        //
    }
    std::cout<<"log: socket listening port 6666..."<<std::endl;
    if(listen(shSrv,5))
    {
        std::cout<<"error: socket listen failed"<<std::endl;
        return false;
    }

    size_t sockaddrSize = sizeof(sockaddr_in);
    shCli = accept(shSrv,(sockaddr*)&addrClient,(socklen_t*)&sockaddrSize);
    if(-1 == shCli)
    {
        std::cout<<"error: socket accept failed"<<std::endl;
        return false;
    }
    std::cout<<"log: connected to client"<<std::endl;
    this->updataStateTimer.start();
    if (ConvertToUnlocked()) //转化成非阻塞
        return true;
    else
        return false;
}

void TCP_Sender::CloseSocket(){
    close(shCli);
    close(shSrv);

}
bool TCP_Sender::SendMsg(const void* dataPtr,size_t dataSize)
{
    if(send(shCli,dataPtr,dataSize,0) == -1)
    {
        //std::cout<<"error: socket send msg failed"<<std::endl;
        return false;
    }
    else
    {
        //std::cout<<"log: socket send msg successed"<<std::endl;
        return true;
    }
}
bool TCP_Sender::ConvertToUnlocked()
{
    if(fcntl(shCli, F_SETFL, fcntl(shCli, F_GETFL, 0) | O_NONBLOCK) == -1)
    {
        std::cout<<"error: convert to unblocked failed"<<std::endl;
        return false;
    }
    else
    {
        return true;
    }
}

bool TCP_Sender::RetriveMsg(void *buff,size_t buffSize)//从sicket取回信息
{
    auto err = recv(shCli,buff,buffSize,0);
    if(err == -1 || err == 0)
    {
        //std::cout<<"error: none msg to retrive"<<std::endl;
        return false;
    }
    else
    {
        std::cout<<"log: retrive "<<err<<" bytes"<<std::endl;
        return true;
    }
}


bool TCP_Sender::AppServiceCB(qingzhou_bringup::app::Request &req,qingzhou_bringup::app::Response &res)
{
    if(req.statue == 1)
    {
        ROS_INFO_STREAM_COND(_open_debug, "weak _watchRLCond");
        this->_watchRLCond.notify_one();
        return true;
    }
    if(req.statue == 2)//cancel all goal
    {
        this->moveBaseActionClientPtr->cancelAllGoals();
        ROS_WARN_STREAM_COND(_open_debug, "CANCEL ALL GOAL AT ["<<robotPose.position.x<<","<<robotPose.position.y<<"]");
        return true;
    }
    if(req.statue == 3)//pub goal to tfl but not change location
    {
        updataRobotCurruentGoal(goal_tfl);
        move_base_msgs::MoveBaseGoal tmp;
        // move_base_msgs::MoveBaseActionGoal tmpwgoalid;
        // tmpwgoalid.header.frame_id = "map";
        // tmpwgoalid.header.stamp = ros::Time::now();
        // tmpwgoalid.goal_id.id = this->_EmumTranslator(robot_local_state.curruentGoal);
        // tmpwgoalid.goal_id.stamp = ros::Time::now();
        tmp.target_pose.header.frame_id = "map";
        tmp.target_pose.pose.position.x = robot_local_state.goalList[goal_tfl].x;
        tmp.target_pose.pose.position.y = robot_local_state.goalList[goal_tfl].y;
        tmp.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot_local_state.goalList[goal_tfl].z);
        // tmpwgoalid.goal = tmp;
        ROS_INFO_STREAM_NAMED("TCP_Sender", "goal setting: "<<this->_EmumTranslator(goal_tfl)<<" and detail:"<<"x:"<<tmp.target_pose.pose.position.x<<" y:"<<tmp.target_pose.pose.position.y<<" z:"<<robot_local_state.goalList[goal_tfl].z<<"qx:"<<tmp.target_pose.pose.orientation.x<<" qy:"<<tmp.target_pose.pose.orientation.y<<" qz"<<tmp.target_pose.pose.orientation.z<<" qw"<<tmp.target_pose.pose.orientation.w);
        tmp.target_pose.header.stamp = ros::Time::now();
        this->moveBaseActionClientPtr->sendGoal(tmp);
        return true;
    }
    if(req.statue == 4)//pub goal to curruent goal
    {
        updataRobotCurruentGoal(goal_unload);
        move_base_msgs::MoveBaseGoal tmp;
        // move_base_msgs::MoveBaseActionGoal tmpwgoalid;
        // tmpwgoalid.header.frame_id = "map";
        // tmpwgoalid.header.stamp = ros::Time::now();
        // tmpwgoalid.goal_id.id = this->_EmumTranslator(robot_local_state.curruentGoal);
        // tmpwgoalid.goal_id.stamp = ros::Time::now();
        tmp.target_pose.header.frame_id = "map";
        tmp.target_pose.pose.position.x = robot_local_state.goalList[robot_local_state.curruentGoal].x;
        tmp.target_pose.pose.position.y = robot_local_state.goalList[robot_local_state.curruentGoal].y;
        tmp.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot_local_state.goalList[robot_local_state.curruentGoal].z);
        // tmpwgoalid.goal = tmp;
        ROS_INFO_STREAM_NAMED("TCP_Sender", "goal setting: "<<this->_EmumTranslator(robot_local_state.curruentGoal)<<" and detail:"<<"x:"<<tmp.target_pose.pose.position.x<<" y:"<<tmp.target_pose.pose.position.y<<" z:"<<robot_local_state.goalList[robot_local_state.curruentGoal].z<<"qx:"<<tmp.target_pose.pose.orientation.x<<" qy:"<<tmp.target_pose.pose.orientation.y<<" qz"<<tmp.target_pose.pose.orientation.z<<" qw"<<tmp.target_pose.pose.orientation.w);
        tmp.target_pose.header.stamp = ros::Time::now();
        this->moveBaseActionClientPtr->sendGoal(tmp, boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2), boost::bind(&TCP_Sender::GoalActiveCB, this));
        return true;

    }
    return false;



}

//一直唤醒
void TCP_Sender::SubBettaryInfoCB(const std_msgs::Float32::ConstPtr &msg)
{
    robotStatusMsg.bettary = msg.get()->data;
   // std::cout<<"log :set bettary value"<<std::endl;

}
//一直唤醒
void TCP_Sender::SubSpeedCB(const geometry_msgs::Twist::ConstPtr &msg)
{
    robotStatusMsg.linearSpeed =msg.get()->linear.x;
    robotStatusMsg.angularSpeed=msg.get()->angular.z;
}


//v2 只有开启的时候才唤醒
void TCP_Sender::SubTrafficLightCB(const geometry_msgs::Vector3::ConstPtr &msg)
{
    // robotStatusMsg.trafficLight = (int)(msg.get()->x);
    
    this->robot_local_state.trafficLightColor = (TRAFFICLIGHT)((int)(msg.get()->x));
    // ROS_INFO_STREAM_NAMED("SubTrafficLightCB", "traffic light:" << this->robot_local_state.trafficLightColor);
    if ((this->robot_local_state.trafficLightColor == red || this->robot_local_state.trafficLightColor == yellow) && (!this->haveDetectedRedTfl))
    {
        ROS_INFO_STREAM("det red");


        this->haveDetectedRedTfl = true;//不要频繁发布目标点
        haveDetectedGreenTfl = false;
        this->updataRobotLocation(loadingtotfl);
        this->updataRobotCurruentGoal(goal_tfl);
        // this->robot_local_state.goalState = active;
        this->ExecGoal();
    }
    //检测到绿灯 或者 停车时间超过6s
    else if((this->robot_local_state.trafficLightColor == green) && (!this->haveDetectedGreenTfl) ) //如果是绿灯，不管车在哪里都直接前往下一个目标点
    {
        std::cout <<"now time:"<<ros::Time::now()<< "start time:"<<_redStartTime<<"  " << (ros::Time::now().toSec() - _redStartTime.toSec()) << std::endl;
        _redStartTime = ros::Time(0);
        //频繁发布绿灯的目标点会怎么样？ 會開不起來
        ROS_INFO_STREAM("det green");
        this->haveDetectedGreenTfl = true;
        haveDetectedRedTfl = false;
        this->updataRobotLocation(loadtounload);
        this->updataRobotCurruentGoal(goal_unload);
        this->robot_local_state.goalState = active;
        this->ExecGoal();

        //******减速带前减速***********
        qingzhou_bringup::app req;
        req.request.statue = 2;
        this->dynamicparamsclient.call(req);
        ROS_INFO_NAMED("TCP_Sender", "DYNAMIC PARAPMS OPEN");
        //等到了unload在关闭红绿灯探测吧

    }
}

//v2 v2 只有开启的时候才唤醒
void TCP_Sender::SubLineCB(const geometry_msgs::Vector3::ConstPtr &msg)
{

    //注意，只有发送请求之后，这个话题在有值，这个回调函数才会被调用，并且这个函数是为了检测有没有退出赛道

    int pianyi = (msg.get()->y);
    // std::cout << "painyi: " << pianyi << std::endl;
    if (int(pianyi) && this->robot_local_state.openRoadLineDet && this->robot_local_state.location == roadline) //pianyi >0 才进入 因为和红绿灯共同使用一个话题，默认情况下painyi=0
    {

        if(pianyi >998 || robotPose.position.y > -1.43){//或却y超出赛道就判断
            this->updataRobotLocation(roadlineout);
            if (this->StopRLDet())
            {
                qingzhou_bringup::app req;
                req.request.statue = 4;
                this->dynamicparamsclient.call(req);
                ROS_INFO_NAMED("TCP_Sender", "DYNAMIC PARAPMS OPEN");     
            }
            else{
                ROS_INFO_NAMED("TCP_Sender", "Oh no! robot cant stop!!");
            }
            //
            
        }
    }
    else{
    }
}
//目标点完成的回调函数用于更改 goalstate v2
void TCP_Sender::GoalDoneCB(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
    ROS_INFO("goan done result: [%s]", state.toString().c_str());
    // first clear costmap
    
    //ROS_INFO("Answer: %i", result->sequence.back());
    //出发点-转载点 -> 装载点； 装载点 - 交通灯->交通灯； 交通灯 - 卸货点->卸货点； 卸货点 - 车道线出发点->车道线出发点 ->
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED && abs(robotPose.position.x-robot_local_state.goalList[robot_local_state.curruentGoal].x)<0.5 &&abs(robotPose.position.y-robot_local_state.goalList[robot_local_state.curruentGoal].y)<0.5)
    {
        ROS_INFO_STREAM_COND(_open_debug, "GOAL DONE SUCCESS, goal and location updata may have been done manually!");
        // ROS_INFO_STREAM_COND(this->_open_debug, "robot pose: x:" << robotPose.position.x << " y:" << robotPose.position.y << " goal_x:" << robot_local_state.goalList[robot_local_state.curruentGoal].x << " goal_y:" << robot_local_state.goalList[robot_local_state.curruentGoal].y);
        // if (this->robot_local_state.location != unloadtostart)
        //     this->ClearCostmapAndWait();

        // // haveDetectedTfl = false;
        // this->robot_local_state.goalState = reach;
        // switch (this->robot_local_state.location)
        // {
        // case starttoload: {
            
        //     this->updataRobotLocation(load);
        //     break;
        //     }
            
        // case tfltounload:{this->updataRobotLocation(unload);break;}
        // case loadtounload:{this->updataRobotLocation(unload);break;}
        // case unloadtostart:{
        //     this->updataRobotLocation(start);
        //     this->_circleTimeList.push_back((ros::Time::now()-this->_startTime).toSec());
        //     std::cout<<"|||||||||CIRCUL TIME:"<<this->_circleTimeList.back()<<"|||||||||"<<std::endl;
        //     std::cout << "-----------------TIME COUNTER----------------"<<std::endl;
        //     this->_startTime = ros::Time::now();
        //     for (int i = 0; i < this->_circleTimeList.size();i++)
        //         std::cout << " | " << i << " circle:" << this->_circleTimeList.at(i) << std::endl;
        //     std::cout << "-----------------============----------------"<<std::endl;
        //     break;
        //     }
        // case loadingtotfl:{this->updataRobotLocation(tfl);break;}
        // case unloadtoroadline:
        // {
        //     this->updataRobotLocation(roadlinestart);
        //     break;
        // }
        // default:{ROS_WARN("cant get positonstate");break;}
        // }
    }
    else if(state == actionlib::SimpleClientGoalState::ABORTED )
    {
        this->ClearCostmapAndWait();
        this->robot_local_state.goalState = aborted;
    }
    else if(state == actionlib::SimpleClientGoalState::PREEMPTED)
    {

    }
    else
    {
        
    }
}
void TCP_Sender::GoalActiveCB()
{    
    std::cout<<"goal active"<<std::endl;
}

bool TCP_Sender::SendRobotStatusInfo()//发送机器人当前的状态
{
    return SendMsg(&robotStatusMsg,sizeof(robotStatusMsg));
}



// void TCP_Sender::roadLineControl()
// {
//     float k = 0.80837;//系数
//     float b = 3.48970;
//     // float angle = robotState.roadLinePianyi*k + b;
//     geometry_msgs::Twist cmdData;
//     cmdData.linear.x = 0.3;
//     // cmdData.angular.z = angle/180*3.1415926;
//     cmdvelPuber.publish(cmdData);     
// }



void TCP_Sender::ClearCostmapFirstly() {
    std_srvs::Empty req;
    if (clearCostmapFirstlyClient.call(req))
    {
        ROS_INFO_NAMED("TCP_Sender", "Clear Costmap success!");
    }
    else
    {
        ROS_INFO_NAMED("TCP_Sender", "Clear Costmap failed!");
    }

}

void TCP_Sender::_PrintCurruentLocation() {
    switch (this->robot_local_state.location)
    {
    case  load: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is load");
    break;
    }
    case loadingtotfl: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is [load to tfl]");break;}
    case tfltounload: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is [tfl to unload]");break;}
    case unload: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is [unload]");break;}
    case tfl: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is [tfl]");break;}
    case start: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is [start]");break;}
    case starttoload:{ROS_INFO_NAMED("TCP_Sender","TCP_Sender:Curruent location is [start to load]");break;}
    case unknow: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is [unknow]");break;}
    case unloadtoroadline: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is [unload to rl start]");break;}
    case roadline: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is [roadline]");break;}
    case unloadtostart: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is [unload to start]");break;}
    case roadlineout: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is [road line out]");break;}
    default:
    {
        ROS_INFO_NAMED("TCP_Sender", "robot go to somewhere unkown");
        break;
    }
    }
}    

std::string TCP_Sender::_EmumTranslator(ROBOTLOCATION value){
    // std::cout << "location value" << std::endl;
    if (true)
    {
        switch (value)
        {
        case  load: 
        {
            return "[load]";
            break;
        }                  
        case loadingtotfl: {return "[load to tfl]";break;}
        case tfltounload: {return "[tlf to unload]" ;break;}
        case unload: {return "[unload]";break;}
        case tfl: {return "[tfl]";break;}
        case start: {return "[start]";break;}
        case starttoload: {return "[start to load]";break;}
        case unknow:{return "[unknow]";break;}
        case unloadtoroadline: {return "[unload to rl start]";break;}
        case roadline: {return "[road line]";break;}
        case unloadtostart: {return "[unload to start]";break;}
        case roadlineout: {return "[road line out]";break;}
        case loadtounload: {return "[road to unload]";break;}
        case roadlinestart:{return "[roadlinestart]";break;}
        default:
            {return "[???]";break;}
            break;
        }
    }
}
std::string TCP_Sender::_EmumTranslator(ROBOTGOALPOINT goal)
{
    switch (goal)
    {
    case  goal_load:
    {
        return "[load]";
        break;
    }
    case goal_unload: {return "[unload]";break;}
    case goal_tfl: {return "[tfl]";break;}
    case goal_start: {return "[start]";break;}
    case goal_roadlinestart: {return "[road line start]";break;}
    default:
    {return "[???]";break;}
        break;
    }
}

//v2
bool TCP_Sender::SwitchAutoGoalControlFlag()
{
    if (this->robot_local_state.autoGoalControl)
    {
        ROS_INFO_NAMED("TCP_Sender", "auto goal control shutdown!");
        this->robot_local_state.autoGoalControl = false;
    }
    else{
        ROS_INFO_NAMED("TCP_Sender", "auto goal control active");
        this->robot_local_state.autoGoalControl = true;
    }
    return this->robot_local_state.autoGoalControl;
}

//v2
bool TCP_Sender::SwitchVisionControl(){
    //todo 添加上位机log消息

    if(this->robot_local_state.openRoadLineDet)
    {
        ROS_INFO_NAMED("TCP_Sender", "att: RL control will be shutdown");
        this->StopRLDet();
        this->_PrintCurruentLocation();
        return true;
    }
    else{
        ROS_INFO_NAMED("TCP_Sender", "att: RL control will be active");
        this->OpenRLDet();
        this->_PrintCurruentLocation();
        return true;
    }
}
//v2
bool TCP_Sender::SwitchTflControl()
{

    if(this->robot_local_state.openTflDet)
    {
        // ROS_INFO_NAMED("TCP_Sender", "att: TFL control will be shutdown");
        this->StopTflDet();
        return true;
    }
    else
    {
        // ROS_INFO_NAMED("TCP_Sender", "att: TFL control will be active");
        this->OpenTflDet();
        return true;
    }
}

bool TCP_Sender::OpenTflDet()
{
    ROS_INFO_NAMED("TCP_Sender","OPENING TFL detector...");
    qingzhou_bringup::app req;
    req.request.statue = 3;
    if (visioncontrolclient.call(req))
    {
        ROS_INFO_NAMED("RCP_Sender", "open success");
        this->robot_local_state.openTflDet = true;
        return true;
    }
    else
    {
        ROS_INFO_NAMED("RCP_Sender", "open failed");
        this->robot_local_state.openTflDet = false;
        return false;
    }
}

bool TCP_Sender::StopTflDet()
{
    ROS_INFO_NAMED("TCP_Sender","STOPING TFL detector...");
    qingzhou_bringup::app req;
    req.request.statue = 4;
    if (visioncontrolclient.call(req))
    {
        ROS_INFO_NAMED("RCP_Sender", "stop success");
        this->robot_local_state.openTflDet = false;
        return true;
    }
    else
    {
        ROS_INFO_NAMED("RCP_Sender", "stop failed");
        this->robot_local_state.openTflDet = true;
        return false;
    }
}

bool TCP_Sender::OpenRLDet()
{
    qingzhou_bringup::app cmd_filter_req;
    cmd_filter_req.request.statue = 1; //close cmd_filter_puber
    if(cmdvelFilterClient.call(cmd_filter_req))
    {
        ROS_INFO("closed cmdflter puber ");
        qingzhou_bringup::app req;
        req.request.statue = 1;
        if(visioncontrolclient.call(req))
        {
            this->robot_local_state.openRoadLineDet = true;
            this->updataRobotLocation(roadline);
            ROS_INFO("open RL control success");
            return true;
        }
        else
        {
            this->robot_local_state.openRoadLineDet = false;
            ROS_WARN("open RL control failed");
            return false;
        }
    }
    else
    {
        ROS_WARN("cant close cmdfiltet puber, RL world not be open");
    }   
}   

// 先将目标点发布到起始区，但是请求cmdelfilter的服务，解除cmdel的控制，等到visioncontrolclient的statue==0的时候再启动控制
bool TCP_Sender::StopRLDet()
{
    qingzhou_bringup::app req;
    req.request.statue = 2;
    qingzhou_bringup::app cmdvel_server_req;
    cmdvel_server_req.request.statue = 1;//stop cmdvel control
    if (visioncontrolclient.call(req)&& cmdvelFilterClient.call(cmdvel_server_req))// 
    {
        this->robot_local_state.goalState = reach;//发布去起始点的目标点
        ROS_INFO("stop RL control success");
        this->robot_local_state.openRoadLineDet = false;
        //等待一段时间后 返回导航控制状态
        ROS_INFO_NAMED("TCP_Sender_stopRLDet","wait 1s (make robot turn right)");
        // ros::Duration(1).sleep();
        this->delayOneSecondTimer = nh.createTimer(ros::Duration(1.0), [=](auto t)
                                                   {
                                                       qingzhou_bringup::app req;
                                                       req.request.statue = 0;
                                                       qingzhou_bringup::app cmdvel_server_req;
                                                       cmdvel_server_req.request.statue = 2;//open cmdvel control
                                                       if (visioncontrolclient.call(req)&&cmdvelFilterClient.call(cmdvel_server_req))// && 
                                                       {
                                                           //say something
                                                       }
                                                       
                                                   },true,true);

        return true;
    }
    else
    {
        ROS_INFO("stop RL control failed");
        this->robot_local_state.openRoadLineDet = true;
        //如果请求服务失败，robotState.inRoadLine 依然被设置为true，依然会进行退出车道线的判断
        return false;
    }
}

void TCP_Sender::ExecGoal()
{

    if(robot_local_state.curruentGoal == goal_unload)
    {
            // 动态参数调整
        qingzhou_bringup::app req;
        req.request.statue = 2;
        this->dynamicparamsclient.call(req);
        ROS_INFO_NAMED("TCP_Sender", "DYNAMIC PARAPMS OPEN");
    }

    move_base_msgs::MoveBaseGoal tmp;

    tmp.target_pose.header.frame_id = "map";
    tmp.target_pose.pose.position.x = robot_local_state.goalList[robot_local_state.curruentGoal].x;
    tmp.target_pose.pose.position.y = robot_local_state.goalList[robot_local_state.curruentGoal].y;
    tmp.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot_local_state.goalList[robot_local_state.curruentGoal].z);
    // tmpwgoalid.goal = tmp;
    ROS_INFO_STREAM_NAMED("TCP_Sender", "goal setting: "<<this->_EmumTranslator(robot_local_state.curruentGoal)<<" and detail:"<<"x:"<<tmp.target_pose.pose.position.x<<" y:"<<tmp.target_pose.pose.position.y<<" z:"<<robot_local_state.goalList[robot_local_state.curruentGoal].z<<"qx:"<<tmp.target_pose.pose.orientation.x<<" qy:"<<tmp.target_pose.pose.orientation.y<<" qz"<<tmp.target_pose.pose.orientation.z<<" qw"<<tmp.target_pose.pose.orientation.w);

    tmp.target_pose.header.stamp = ros::Time::now();
    this->robot_local_state.goalState = active;
    // this->moveBaseActionClientPtr->sendGoal(tmp, boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2), boost::bind(&TCP_Sender::GoalActiveCB, this));
    this->moveBaseActionClientPtr->sendGoal(tmp, boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2), boost::bind(&TCP_Sender::GoalActiveCB, this));
    // this->moveBaseActionClientPtr->sendGoal()
    ROS_INFO_NAMED("TCP_Sender", "send goal point success");




    // this->moveBaseActionClientPtr->cancelAllGoals();
}

void TCP_Sender::updataRobotLocation(ROBOTLOCATION value)
{
    this->robot_local_state.location = value;
    ROS_INFO_STREAM_NAMED("TCP_Sender", "LOCATION updata: "<<this->_EmumTranslator(this->robot_local_state.location)<<" success");
}

void TCP_Sender::updataRobotCurruentGoal(ROBOTGOALPOINT goal)
{
    this->robot_local_state.curruentGoal = goal;
    ROS_INFO_STREAM_NAMED("TCP_Sender", "GOAL updata: "<<this->_EmumTranslator(this->robot_local_state.curruentGoal)<<"success");
}

void TCP_Sender::RunGoal_v2()
{
    //如果成功到达某个目标点那就前往下一个目标点
    //出发点 -> 装载点 -> 交通灯停止线 -> （由话题回调函数控制，在交通灯停止线等待绿灯） -> 卸载区 ->
    //(车道线 -> 发送请求视觉控制 ->（若请求成功则开始检测是否退出车道线） -> 车道线外面: 这个过程中状态都是reach) -> 出发点
    // ROS_INFO_STREAM("run goal state:"<<robotState.goalstate);
    static bool isFirstRun = false; //用来控制只运行一次的代码
    if (this->robot_local_state.goalState == reach) //成功到达某个目标点
    {
        ROS_INFO_STREAM("********robot reach goal point success!*********");


        switch (this->robot_local_state.location)
        {
        case start:
        {
            if(this->robot_local_state.autoGoalControl)//只有开启autogoalcontrol之后才可以一到卸货区就发布去roalline的goal，否则要我们手动发布
            {
                // this->_countTimeList.push_back((ros::Time::now() - this->_tmpStartTime).toSec());
                robotStatusMsg.partTime = (ros::Time::now() - this->_tmpStartTime).toSec();
                // std::cout << "|||||||||PART TIME([unload to start]):" << this->_countTimeList.back() << "|||||||||" << std::endl;
                this->_tmpStartTime = ros::Time::now();
                // for (int i = 0; i < this->_countTimeList.size();i++)
                //     std::cout << " | " << i << " part:" << this->_countTimeList.at(i) << std::endl;

                // this->_countTimeList.clear();

                //前往getGood point
                this->updataRobotLocation(starttoload);
                this->updataRobotCurruentGoal(goal_load);
                this->ExecGoal();

                qingzhou_bringup::app req;
                req.request.statue = 3;
                this->dynamicparamsclient.call(req);
                ROS_INFO_NAMED("TCP_Sender", "DYNAMIC PARAPMS OPEN： start to load");
                this->robot_local_state.goalState = active;//reach状态只能进入一次（每次goalreach）
            }
            else{
                // ROS_INFO_NAMED("TCP_Sender", "robot have reached start, pls pub next goal");
            }
            break;
        }
        case load:
        {

            if (this->robot_local_state.autoGoalControl == true) //只有开启autogoalcontrol之后才可以一到达装货区就发布去停止线的goal，否则要我们手动发布
            {
                // this->_countTimeList.push_back((ros::Time::now() - this->_tmpStartTime).toSec());
                robotStatusMsg.partTime = (ros::Time::now() - this->_tmpStartTime).toSec();
                // std::cout<<"|||||||||PART TIME([start to load]):"<<this->_countTimeList.back()<<"|||||||||"<<std::endl;
                this->_tmpStartTime = ros::Time::now();

                this->updataRobotLocation(loadtounload);
                this->updataRobotCurruentGoal(goal_unload);
                this->ExecGoal();
                
                haveDetectedRedTfl = false; //没有检测到红灯
                this->OpenTflDet(); //开启红绿灯探测
                this->robot_local_state.goalState = active;//reach状态只能进入一次（每次goalreach）
            }
            else{
                // ROS_INFO_NAMED("TCP_Sender", "robot have been to load, pls pub next goal");
            }

            break;

        }
        case tfl:
        {
            //todo
            //如果到达停止线说明现在一定是红灯，没有别的情况会这样了
            // ROS_INFO_NAMED("TCP_Sender", "robot have reached tfl，waiting green...");
            _redStartTime = ros::Time::now(); //
            
            // this->robot_local_state.goalState = active;//reach状态只能进入一次（每次goalreach）
            break;
        }
        case unload:
        {

            if(this->robot_local_state.autoGoalControl)//只有开启autogoalcontrol之后才可以一到卸货区就发布去roalline的goal，否则要我们手动发布
            {
                // this->_countTimeList.push_back((ros::Time::now() - this->_tmpStartTime).toSec());
                robotStatusMsg.partTime = (ros::Time::now() - this->_tmpStartTime).toSec();
                // std::cout<<"|||||||||PART TIME([load to unload]):"<<this->_countTimeList.back()<<"|||||||||"<<std::endl;
                this->_tmpStartTime = ros::Time::now();

                this->updataRobotLocation(unloadtoroadline);
                this->updataRobotCurruentGoal(goal_roadlinestart);
                this->ExecGoal();
                
                qingzhou_bringup::app req;
                req.request.statue = 1;
                this->dynamicparamsclient.call(req);
                ROS_INFO_NAMED("TCP_Sender", "DYNAMIC PARAPMS OPEN");
                //关闭红绿灯探测
                
                this->StopTflDet();
                haveDetectedRedTfl = false;//没有检测到红灯
                this->robot_local_state.goalState = active;//reach状态只能进入一次（每次goalreach）
            }
            else{
                // ROS_INFO_NAMED("TCP_Sender", "robot have reached unload, pls pub next goal");
            }
            break;
        }
        case roadlinestart:
        {
            //开启车道线探测
            this->OpenRLDet();//顺便改状态
            // this->robot_local_state.goalState = active;//reach状态只能进入一次（每次goalreach）
            break;
        }
        //todo...............
        case roadline:
        {
            if(isFirstRun)
            {
                ROS_INFO_NAMED("TCP_Sender", "robot is controlled by vision...");
                isFirstRun = true;
            }
            // this->robot_local_state.goalState = active;//reach状态只能进入一次（每次goalreach）
            //视觉接管控制
            break;
        }
        
        case roadlineout:
        {
            this->updataRobotLocation(unloadtostart);
            this->updataRobotCurruentGoal(goal_start);
            this->ExecGoal();
            isFirstRun = false;

            this->robot_local_state.goalState = active;//reach状态只能进入一次（每次goalreach）
            // this->ExecGoal();
            break;
        }

        default:
        {
            //print curruent location todo
            ROS_ERROR_NAMED("TCP_Sender", "something worong happened, robot have reach an unknown area!");
            break;
        }
        }   
        
    }
    else if(this->robot_local_state.goalState == aborted) //如果目标在中途异常停止
    {
        
        // ROS_INFO_STREAM("abort:");
        this->robot_local_state.goalState = active;//reach状态只能进入一次（每次goalreach）
        ROS_INFO_STREAM_NAMED("TCP_Sender", "can' reach goal point, will send goal again");
        this->_PrintCurruentLocation();//打印当前的坐标

        //正常情况下这个情况下机器人的location和goal都不会发生变化，应该不需要更新，直接再发布一次目标点就行

        this->ExecGoal();
    }
    else if(this->robot_local_state.goalState == active)
    {
        //正在执行goal
        //检测机器人的位置，当他到达goal的时候如果没有reach手动reach
        auto goalPose = robot_local_state.goalList[robot_local_state.curruentGoal];
        if ((abs(this->robotPose.position.x - goalPose.x) < 0.2 && abs(this->robotPose.position.y - goalPose.y)<0.2||(robot_local_state.curruentGoal == goal_load && this->robotPose.position.y < goalPose.y+0.2 && robotPose.position.x > goalPose.x -1.0)||(robot_local_state.curruentGoal == goal_unload && robotPose.position.y > goalPose.y && robotPose.position.x < -1.0)||(robot_local_state.curruentGoal == goal_start && robotPose.position.x > goalPose.x +0.2 && robotPose.position.y > -0.6)))
        {
            ROS_WARN_STREAM_COND(_open_debug, "robot may have reach goal but move base DO NOT cancel goal, canceling manually...@["<<robotPose.position.x<<","<<robotPose.position.y<<"]");
            if(robot_local_state.location != unloadtoroadline)
            {
                
                    // auto cmdData = geometry_msgs::Twist();
                    // cmdData.linear.x = 0;
                    // cmdvelPuber.publish(cmdData);
                    // cmdvelfiltedPuber.publish(cmdData);
                    // ROS_WARN_STREAM("STOPING: " << robotStatusMsg.linearSpeed);
                
                moveBaseActionClientPtr->cancelAllGoals();
                // this->robot_local_state.goalState = reach;
                updataToReachLocation();//相当与之前个的goaldone callback中的success部分
            }
            else
            {
                ROS_WARN_COND(_open_debug, "location is unloadtoroadline: NOT updata REACH LOCATION (and not cancel costmap)");
            }
        }
        //当机器人开出去车道线但一直无法停下来，可能会一直处于active的状态，需要进行处理
    }
    else if(this->robot_local_state.goalState == lost)
    {
        qingzhou_bringup::app req;
        req.request.statue = 3;
        this->dynamicparamsclient.call(req);
        ROS_INFO_NAMED("TCP_Sender", "DYNAMIC PARAPMS OPEN： start to load");

        this->robot_local_state.goalState = active;
        ROS_INFO("this is the first goal point, robot will go to the load area");
        this->_PrintCurruentLocation();//打印当前的区域
        this->updataRobotLocation(starttoload);
        this->updataRobotCurruentGoal(goal_load);
        this->_PrintCurruentLocation();//打印当前的区域
        this->_startTime = ros::Time::now();
        this->_tmpStartTime = this->_startTime;
        this->ExecGoal();
        }
    else{
        ROS_ERROR_NAMED("TCP_Sender", "robot curruent goal point known, this should not happen!"); 
    }
}

bool TCP_Sender::ClearCostmapAndWait()
{
    // auto cmdData = geometry_msgs::Twist();
    // cmdData.linear.x = 0;
    // cmdvelfiltedPuber.publish(cmdData);
    
    std_srvs::Empty req;
    if (clearCostmapClient.call(req)) {
        ROS_INFO_NAMED("TCP_Sender", "clear costmap success!");
        //在转角的时候路径规划没有考虑障碍物？
        auto req=  qingzhou_bringup::app();
        req.request.statue = 1;
        ROS_WARN_STREAM("Sleep 0.75s wating costmap @[" << robotPose.position.x << "," << robotPose.position.y << "," << robotPose.position.z / 3.14159 * 180 << "]");
        if(cmdvelFilterClient.call(req))
        {
            sleepDur = ros::Duration(0.05);
            auto cmdData = geometry_msgs::Twist();
            cmdData.linear.x = 0;
            for (int i = 0; i < 15;i++)
            {
                
                cmdvelfiltedPuber.publish(cmdData);
                sleepDur.sleep();
                // ROS_WARN_STREAM("stop! @[" << robotPose.position.x << "," << robotPose.position.y << "," << robotPose.position.z / 3.14159 * 180 << "]");
            }
        }
        req.request.statue = 2;
        cmdvelFilterClient.call(req);
        // sleepDur = ros::Duration(0.5);
        // sleepDur.sleep();
        ROS_WARN_STREAM("weak up @["<<robotPose.position.x<<","<<robotPose.position.y<<","<<robotPose.position.z/3.14159*180<<"]");
        return true;
    }
    else
    {
        ROS_INFO_NAMED("TCP_Sender", "clear costmap failed");
        return false;
    }
}


void TCP_Sender::updataRobotGoalList(point3d *goalList)
{

    //todo 添加判断是否更新了列表
    ROS_INFO_NAMED("TCP_Sender", "robot are trying to updata goal List...");
    for (int i = 0; i < 5;i++)
    {
        this->robot_local_state.goalList[i] = goalList[i];

        // ROS_INFO_STREAM("goal list information:" << goalList[i].x << " " << goalList[i].y << " " << goalList[i].z);
        ROS_INFO_STREAM("goal list information:" << this->robot_local_state.goalList[i].x << " " << this->robot_local_state.goalList[i].y << " " << this->robot_local_state.goalList[i].z);
    }
    ROS_INFO_NAMED("TCP_Sender", "updata success!");
}

void TCP_Sender::updataStateTimerCB()
{
    robotStatusMsg.goalstate = robot_local_state.goalState;
    robotStatusMsg.robotlocation = robot_local_state.location;
    robotStatusMsg.autoGoalControl = robot_local_state.autoGoalControl;
    robotStatusMsg.openTflDet = robot_local_state.openTflDet;
    robotStatusMsg.openRoadLineDet = robot_local_state.openRoadLineDet;
    robotStatusMsg.trafficLight = robot_local_state.trafficLightColor;
    robotStatusMsg.locationInMapY = this->robotPose.position.y;
    robotStatusMsg.locationInMapX = this->robotPose.position.x;

    actionlib_msgs::GoalStatus gs;
    gs.goal_id.stamp = ros::Time::now();
    gs.goal_id.id = this->_EmumTranslator(robot_local_state.curruentGoal);
    gs.status = robot_local_state.goalState;

    this->_goalStatusPuber.publish(gs);
    this->_locationInMapPuber.publish(this->robotPose);

    // ros::Duration
    // ROS_INFO_STREAM_NAMED("tcp_sender:", "robotpose: x:" << this->robotPose.position.x << " y:" << this->robotPose.position.y);
}

// void TCP_Sender::_RunSpeedPlan()
// {
//     std::vector<SpeedPlanPoint> _sp;
//     _sp.push_back(SpeedPlanPoint(_ppstime1,0,0,_ppsspeed1x,_ppsspeed1z));
//     _sp.push_back(SpeedPlanPoint(_ppstime2,0,0,_ppsspeed2x,_ppsspeed2z));
//     _sp.push_back(SpeedPlanPoint(0.1,0,0,0,0));

//     for(auto spp: _sp)
//     {
//         cmdvelPuber.publish(spp._speed);
//         spp._dua.sleep();
//     }
// }

void TCP_Sender::ListenRobotPose(geometry_msgs::Pose &robotPose)
{

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listen(tfBuffer);
    geometry_msgs::TransformStamped tfs;
    ros::Rate rate(15);
    boost::mutex robotposeMutex;
    // robotposeMutex.unlock();
    std::string err;
    while (ros::ok())
    {

        if(tfBuffer.canTransform("base_link", "map",ros::Time(0),&err))
        {
            tfs = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
            robotposeMutex.lock();
            robotPose.position.x = tfs.transform.translation.x;
            robotPose.position.y = tfs.transform.translation.y;
            robotPose.orientation = tfs.transform.rotation;
            robotPose.position.z = tf::getYaw(robotPose.orientation);
            robotposeMutex.unlock();
            // ROS_INFO_STREAM_NAMED("tcp_sender:", "robotpose_thread: x:" << tfs.transform.translation.x << " y:" << tfs.transform.translation.y);

        }
        else
        {
            //cant trasform
            // std::cout << "tferr:" << err<<std::endl;
        }
        rate.sleep();
    }
}

void TCP_Sender::WatchRLStartAndCancleGoal(MoveBaseActionClient* client,ros::Publisher* cmdpuber)
{
    ros::Rate rate(20);
    geometry_msgs::Twist speed;
    speed.linear.x = 0.5;
    speed.angular.z = 0;
    static bool firstRun;
    boost::unique_lock<boost::recursive_mutex> lock(watchRLMutex);
    while (ros::ok())
    {
        //当机器人的目标点没有位于unloadtoroadline的时候就让该线程休眠，进入unloadtoroadline的时候weak才有用
        while (this->robot_local_state.location != unloadtoroadline)
        {
            _watchRLCond.wait(lock);
            firstRun = true;
        }
        // std::cout << "watch position" << std::endl;
        if (robotPose.position.y > -4.2 && robotPose.position.y < -3.9 && abs(robotPose.position.x - 1.10) < 0.15)
        {
            if(firstRun)
            {
                qingzhou_bringup::app cmd_filter_req;
                cmd_filter_req.request.statue = 1; //close cmd_filter_puber
                if(cmdvelFilterClient.call(cmd_filter_req))
                {
                    ROS_INFO("closed cmdflter puber ");
                }
                else
                {
                    ROS_INFO("cant closed cmdflter puber ");
                }
                ROS_WARN_STREAM_COND(this->_open_debug, "TCP_Sender: canceling all goal, goal switch to goal_start @["<<robotPose.position.x<<","<<robotPose.position.y<<","<<robotPose.position.z/3.1415926*180<<"]");
                client->cancelAllGoals();
                robot_local_state.curruentGoal = goal_start;
                ROS_INFO_STREAM_COND(this->_open_debug, "TCP_Sender: cancel all goal and open RL det robotpose[" << robotPose.position.x << "," << robotPose.position.y << "]");
                firstRun = false;         
            }
            speed.linear.x = 0.5;
            speed.angular.z = -(robotPose.position.z - (90.0/180.0*3.1415926))*1.3;
            ROS_WARN_STREAM("RL START ANGULAR :" << robotPose.position.z / 3.1415926 * 180 << ">>" << speed.angular.z / 3.14 * 180);
            cmdpuber->publish(speed);
            // std::cout << "move forward with x = 0.5" << std::endl;
        }
        else if(robotPose.position.y > -3.9)
        {
            firstRun = true;
            ROS_WARN_STREAM_COND(this->_open_debug, "TCP_Sender: Updata pose to open RL @[" << robotPose.position.x << "," << robotPose.position.y <<","<<robotPose.position.z/3.1415926*180<<"]");
            this->updataRobotLocation(roadlinestart);
            // speed.linear.x = 0.0;
            // speed.angular.z = 0.0;
            // cmdpuber->publish(speed);
            this->robot_local_state.goalState = reach;
        }
        // rate.sleep() 不是ros的线程，不知道开rate.sleep没有有用
        // sleep(0.05);
        // std::cout << "sleep begin" << std::endl;
        boost::this_thread::sleep_for(boost::chrono::milliseconds(25));
        // std::cout << "sleep end" << std::endl;
    }
}







// void TCP_Sender::InitializePose()
// {
//     auto initialPose = geometry_msgs::PoseWithCovarianceStamped();
//     initialPose.header.frame_id = "map";
//     initialPose.header.stamp = ros::Time::now();
//     initialPose.pose.pose.position.x = 0;
//     initialPose.pose.pose.position.y = 0;
//     initialPose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

//     this->_initialposePuber.publish(initialPose);
//     ROS_INFO_STREAM_COND(_open_debug, "Initialize robot pose!");
// }


void TCP_Sender::RunGoalManually(ROBOTLOCATION location)
{
    switch (location)
    {
    case unloadtoroadline:
    {
        qingzhou_bringup::app req;
        req.request.statue = 1;
        this->dynamicparamsclient.call(req);
        ROS_INFO_NAMED("TCP_Sender", "DYNAMIC PARAPMS OPEN");
        //关闭红绿灯探测
        this->StopTflDet();
        haveDetectedRedTfl = false;//没有检测到红灯
        this->ExecGoal();
        this->robot_local_state.goalState = active;//reach状态只能进入一次（每次goalreach）
        break;
    }
    case starttoload:
    {
        qingzhou_bringup::app req;
        req.request.statue = 3;
        this->dynamicparamsclient.call(req);
        ROS_INFO_NAMED("TCP_Sender", "DYNAMIC PARAPMS OPEN： start to load");
        this->ExecGoal();
        this->robot_local_state.goalState = active;//reach状态只能进入一次（每次goalreach）
        break;
    }
    case loadtounload:
    {
        haveDetectedRedTfl = false; //没有检测到红灯
        this->OpenTflDet(); //开启红绿灯探测
        this->ExecGoal();
        this->robot_local_state.goalState = active;//reach状态只能进入一次（每次goalreach）
        break;
    }
    default:
    {
        ROS_WARN_STREAM_COND(_open_debug, "Robot IN a error location");
        break;

    }
    }
}

void TCP_Sender::updataToReachLocation()
{
    ROS_INFO_STREAM_COND(this->_open_debug, "robot pose: x:" << robotPose.position.x << " y:" << robotPose.position.y << " goal_x:" << robot_local_state.goalList[robot_local_state.curruentGoal].x << " goal_y:" << robot_local_state.goalList[robot_local_state.curruentGoal].y);
    if (this->robot_local_state.location == unloadtostart)
    {
        ROS_INFO_STREAM_COND(_open_debug, " OVER SLEEP 0.5s");
        sleep(1.0);
    
    }
    else if(this->robot_local_state.location == loadingtotfl)
    {
        ROS_INFO_STREAM_COND(_open_debug, "tfl DO NOT clear costmap");
        ROS_INFO_NAMED("TCP_Sender", "robot have reached tfl，waiting green...");
    }
    else
    {
        this->ClearCostmapAndWait();  //不要直接在线程里面sleep，可能会导致车车溜走，因为现在的线速度不会马上为0
    }
    // haveDetectedTfl = false;
    this->robot_local_state.goalState = reach;
    switch (this->robot_local_state.location)
    {
    case starttoload: {
        
        this->updataRobotLocation(load);
        break;
        }
    case tfltounload:{this->updataRobotLocation(unload);break;}
    case loadtounload:{this->updataRobotLocation(unload);break;}
    case unloadtostart:{
        this->updataRobotLocation(start);
        robotStatusMsg.circleTime = (ros::Time::now() - this->_startTime).toSec();

        // this->_circleTimeList.push_back((ros::Time::now()-this->_startTime).toSec());
        // std::cout<<"|||||||||CIRCUL TIME:"<<this->_circleTimeList.back()<<"|||||||||"<<std::endl;

        // std::cout << "-----------------TIME COUNTER----------------" << std::endl;
        this->_startTime = ros::Time::now();
        // for (int i = 0; i < this->_circleTimeList.size();i++)
        //     std::cout << " | " << i << " circle:" << this->_circleTimeList.at(i) << std::endl;
        // std::cout << "-----------------============----------------"<<std::endl;
        break;
        }
    case loadingtotfl:{this->updataRobotLocation(tfl);break;}
    case unloadtoroadline:
    {
        this->updataRobotLocation(roadlinestart);
        break;
    }
    default:{ROS_WARN("cant get positonstate");break;}
    }
}