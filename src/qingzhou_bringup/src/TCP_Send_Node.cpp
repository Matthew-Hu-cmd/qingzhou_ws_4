#include "TCP_Sender.h"

TCP_Sender *tcpsender;
bool acitonSetedGoal = false;//stop 0 start 1 getGood 2 throwGood 
rcm robotControlMsg = rcm();
// void TimerCB(const ros::TimerEvent &e)
// {
//     tcpsender->SendRobotStatusInfo();
// }
void SendMsg(TCP_Sender *tcpsender)
{
    ros::Rate rate(20);
    while (ros::ok())
    {
        tcpsender->SendRobotStatusInfo();
        rate.sleep();
    }
}


int main(int argc, char **  argv)
{
    ros::init(argc,argv,"TCP_Send_node");
    ros::NodeHandle nh("~");
    // ros::Timer timer = nh.createTimer(ros::Duration(0.05),TimerCB);
    ros::Rate rate(25);
    ros::Duration dsleep(5);
    ros::Publisher cmdpub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    tcpsender = new TCP_Sender(nh);
    bool haveNewMsg = false;

    if(tcpsender->SocketInit()) //暂时注释
    // if(true)
    {
        tcpsender->WaitServices();
        boost::thread sendMsgThread(boost::bind(&SendMsg,tcpsender));
        // boost::thread retriveMsgThread(boost::bind(&RetriveRobotMsg, &robotControlMsg, tcpsender,&haveNewMsg));
        // tcpsender->InitializePose();
        tcpsender->ClearCostmapFirstly();
        while (ros::ok())
        {
            ros::spinOnce();

   
            if(tcpsender->RetriveMsg(&robotControlMsg,sizeof(robotControlMsg)))
            {
                switch(robotControlMsg.controlFunctionSelector)
                {
                    case 0x01:
                    {
                        ROS_INFO("set speed");
                        geometry_msgs::Twist tmp;
                        tmp.linear.x = robotControlMsg.linearSpeed;
                        tmp.angular.z = robotControlMsg.angularSpeed;
                        tcpsender->cmdvelPuber.publish(tmp);
                        break;
                    }
                    case 0x02:
                    {
                        ROS_INFO_NAMED("TCP_Sender_Node","set goal");
                        // ROS_INFO_STREAM_NAMED("TCP_Sender_Node","goal list size:"<<typeid(robotControlMsg.goalList).name());
                        tcpsender->updataRobotGoalList((point3d*)&robotControlMsg.goalList);
  
                        ROS_INFO_NAMED("TCP_Sender_Node","seted goal");

                        break;
                        
                    }
                    case 0x03://改变目标 location 并且执行goal
                    {
                        ROS_INFO_NAMED("TCP_Sender_Node","updata goal");
                        tcpsender->updataRobotCurruentGoal(robotControlMsg.curruentGoal);
                        ROS_INFO_NAMED("TCP_Sender_Node","updatad!");
                        tcpsender->updataRobotLocation(robotControlMsg.location);
                        tcpsender->RunGoalManually(robotControlMsg.location);
                        break;
                    }
                    case 0x04:
                    {
                        ROS_INFO("actiong setted goal");
                        if(acitonSetedGoal == false)
                        {
                            acitonSetedGoal = true;
                        }
                        else
                        {
                            acitonSetedGoal = false;
                        }
                        break;
                    }
                    case 0x05:
                    {
                        ROS_INFO_STREAM_NAMED("TCP_Sender_Node","Swtich auto goal control flag to"<<tcpsender->SwitchAutoGoalControlFlag());
                        break;
                    }   
                    
                    case 0x06:
                    {
                        if(tcpsender->SwitchVisionControl())
                            ROS_INFO_STREAM_NAMED("TCP_Sender_Node","Swtich vision control success");
                        else
                            ROS_INFO_STREAM_NAMED("TCP_Sender_Node","Swtich vision control failed");
                        break;
                    }
                    case 0x07:
                    {
                        if(tcpsender->SwitchTflControl())
                            ROS_INFO_STREAM_NAMED("TCP_Sender_Node","Swtich tfl control success");
                        else
                            ROS_INFO_STREAM_NAMED("TCP_Sender_Node","Swtich tfl control failed");
                        break;
                    }
                    case 0x08://updata locaiton
                    {
                        ROS_INFO_STREAM_NAMED("TCP_Sender_Node","try to change robot location");
                        tcpsender->updataRobotLocation(robotControlMsg.location);
                        break;
                    }
                    // case 0x09://initialize pose
                    // {
                    //     tcpsender->InitializePose();
                    //     break;
                    // }
                    case 0x99://pause
                    {
                        tcpsender->moveBaseActionClientPtr->cancelAllGoals();
                        cmdpub.publish(geometry_msgs::Twist());
                    }
                    default:
                    {
                        break;
                    }

                }
                //处理接受到的控制消息

            }
            if(acitonSetedGoal)
                tcpsender->RunGoal_v2();
            rate.sleep();
            //std::cout<<"check new msg"<<std::endl;

        }
        std::cout<<"quit TCP SEND NODE"<<std::endl;
        tcpsender->CloseSocket();
    }
    else
    {
        std::cout<<"socket inited failed \n exit"<<std::endl;
        return -1;
    }
    
}
