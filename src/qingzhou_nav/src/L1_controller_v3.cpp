/*

An renforcement of tianbot_racecar package
latest editor:Matthew_hu
Gmail: matthewhu26@gmail.com
no rights reserved, do whatever the fuck you want!

Todo: 
1. add dynamicreconfigure feature inplace of the param server(DONE 5.31 2022)

2. solve the problem that can't reach the final goal
|---maybe no extra function is needed to achieve that

3. speed changed too abrptly while stopping & startting
|---maybe a good thing 
|---but can still be changed by 1/(（1/car2goal_dist）* speed)

*/

#include <iostream>
#include "ros/ros.h"

#include "dynamic_reconfigure/server.h"
#include "qingzhou_nav/L1_dynamicConfig.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <visualization_msgs/Marker.h>

#define PI 3.14159265358979

/********************/
/* CLASS DEFINITION */
/********************/
class L1Controller
{
    public:
        L1Controller();
        ~L1Controller();
        void initMarker();
        bool isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose);
        bool isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos);
        double getYawFromPose(const geometry_msgs::Pose& carPose);        
        double getEta(const geometry_msgs::Pose& carPose);
        double getCar2GoalDist();
        double getL1Distance(const double& _Vcmd);
        double getSteeringAngle(double eta);
        double getGasInput(const float& current_v);
        geometry_msgs::Point get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose);

    private:
        ros::NodeHandle n_;
        ros::Subscriber odom_sub, path_sub, goal_sub;
        ros::Publisher pub_, marker_pub;
        ros::Timer timer1, timer2;
        tf::TransformListener tf_listener;
        // dynamic_server 
        // dynamic_reconfigure::Server<qingzhou_nav::L1_dynamicConfig> dynamic_server;
        dynamic_reconfigure::Server<qingzhou_nav::L1_dynamicConfig> *dsrv_;

        visualization_msgs::Marker points, line_strip, goal_circle;
        ackermann_msgs::AckermannDrive ackermann_cmd;
        geometry_msgs::Point odom_goal_pos;
        nav_msgs::Odometry odom;
        nav_msgs::Path map_path, odom_path;

        double L, Lfw, Lrv, Vcmd, lfw, lrv, steering, u, v;
        double gas_gain, base_angle, base_speed, angle_gain, goal_radius;
        int controller_freq;
        bool foundForwardPt, goal_received, goal_reached;

        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void goalReachingCB(const ros::TimerEvent&);
        void controlLoopCB(const ros::TimerEvent&);
        void dynamicCB(qingzhou_nav::L1_dynamicConfig &config, uint32_t level);

}; // end of class


L1Controller::L1Controller()
{
    //Private parameters handler
    ros::NodeHandle pn("~");

    //Publishers and Subscribers
    odom_sub = n_.subscribe("/odom_ekf", 1, &L1Controller::odomCB, this);
    path_sub = n_.subscribe("/move_base/NavfnROS/plan", 1, &L1Controller::pathCB, this);
    goal_sub = n_.subscribe("/move_base_simple/goal", 1, &L1Controller::goalCB, this);
    marker_pub = n_.advertise<visualization_msgs::Marker>("car_path", 10);
    pub_ = n_.advertise<ackermann_msgs::AckermannDrive>("ackermann_cmd", 1);

    //Timer
    timer1 = n_.createTimer(ros::Duration((1.0)/controller_freq), &L1Controller::controlLoopCB, this); // Duration(0.05) -> 20Hz
    timer2 = n_.createTimer(ros::Duration((0.5)/controller_freq), &L1Controller::goalReachingCB, this); // Duration(0.05) -> 20Hz

    //dynamic_server

    // dynamic_reconfigure::Server<qingzhou_nav::L1_dynamicConfig> dynamic_server;
    // dynamic_server.setCallback(boost::bind(&L1Controller::dynamicCB,this,_1,_2));
    dsrv_ = new dynamic_reconfigure::Server<qingzhou_nav::L1_dynamicConfig> (pn);
    dynamic_reconfigure::Server<qingzhou_nav::L1_dynamicConfig>::CallbackType cb = boost::bind(&L1Controller::dynamicCB,this,_1,_2);
    dsrv_->setCallback(cb);

        //Init variables
    Lfw = goal_radius = getL1Distance(Vcmd);
    foundForwardPt = false;
    goal_received = false;
    goal_reached = false;
    ackermann_cmd.speed = 0.0;
    ackermann_cmd.steering_angle = 0.0;

    //Show info
    ROS_INFO("[param] base_speed: %f", base_speed);
    ROS_INFO("[param] base_angle: %f", base_angle);
    ROS_INFO("[param] angle_gain: %f", angle_gain);
    ROS_INFO("[param] Vcmd: %f", Vcmd);
    ROS_INFO("[param] Lfw: %f", Lfw);

    //Visualization Marker Settings
    initMarker();
}

L1Controller::~L1Controller()
{
    delete dsrv_;
}


void L1Controller::initMarker()
{
    points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "odom";
    points.ns = line_strip.ns = goal_circle.ns = "Markers";
    points.action = line_strip.action = goal_circle.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    goal_circle.id = 2;
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle.type = visualization_msgs::Marker::CYLINDER;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    //LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    goal_circle.scale.x = goal_radius;
    goal_circle.scale.y = goal_radius;
    goal_circle.scale.z = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    //goal_circle is yellow
    goal_circle.color.r = 1.0;
    goal_circle.color.g = 1.0;
    goal_circle.color.b = 0.0;
    goal_circle.color.a = 0.5;
}


void L1Controller::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    odom = *odomMsg;
}


void L1Controller::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
    map_path = *pathMsg;
}


void L1Controller::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    try
    {
        geometry_msgs::PoseStamped odom_goal;
        tf_listener.transformPose("odom", ros::Time(0) , *goalMsg, "map" ,odom_goal);
        odom_goal_pos = odom_goal.pose.position;
        goal_received = true;
        goal_reached = false;

        /*Draw Goal on RVIZ*/
        goal_circle.pose = odom_goal.pose;
        marker_pub.publish(goal_circle);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

double L1Controller::getYawFromPose(const geometry_msgs::Pose& carPose)
{
    float x = carPose.orientation.x;
    float y = carPose.orientation.y;
    float z = carPose.orientation.z;
    float w = carPose.orientation.w;

    double tmp,yaw;
    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3 quaternion(q);
    quaternion.getRPY(tmp,tmp, yaw);

    return yaw;
}

bool L1Controller::isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)
{
    float car2wayPt_x = wayPt.x - carPose.position.x;
    float car2wayPt_y = wayPt.y - carPose.position.y;
    double car_theta = getYawFromPose(carPose);

    float car_car2wayPt_x = cos(car_theta)*car2wayPt_x + sin(car_theta)*car2wayPt_y;
    float car_car2wayPt_y = -sin(car_theta)*car2wayPt_x + cos(car_theta)*car2wayPt_y;

    if(car_car2wayPt_x > 0) /*is Forward WayPt*/
        return true;
    else
        return false;
}


bool L1Controller::isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos)
{
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx*dx + dy*dy);

    if(dist < 0.6)
        return false;
    else if(dist >= 0.6)
        return true;
    //  if(dist < Lfw)
    //     return false;
    // else if(dist >= Lfw)
    //     return true;
}

geometry_msgs::Point L1Controller::get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;
    double carPose_yaw = getYawFromPose(carPose);
    geometry_msgs::Point forwardPt;
    geometry_msgs::Point odom_car2WayPtVec;
    foundForwardPt = false;

    if(!goal_reached){
        for(int i =0; i< map_path.poses.size(); i++)
        {
            geometry_msgs::PoseStamped map_path_pose = map_path.poses[i];
            geometry_msgs::PoseStamped odom_path_pose;

            try
            {
                tf_listener.transformPose("odom", ros::Time(0) , map_path_pose, "map" ,odom_path_pose);
                geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
                bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt,carPose);

                if(_isForwardWayPt)
                {
                    bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt,carPose_pos);
                    if(_isWayPtAwayFromLfwDist)
                    {
                        forwardPt = odom_path_wayPt;
                        foundForwardPt = true;
                        break;
                    }
                }
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }
        
    }
    else if(goal_reached)
    {
        forwardPt = odom_goal_pos;
        foundForwardPt = false;
        //ROS_INFO("goal REACHED!");
    }

    /*Visualized Target Point on RVIZ*/
    /*Clear former target point Marker*/
    points.points.clear();
    line_strip.points.clear();
    
    if(foundForwardPt && !goal_reached)
    {
        points.points.push_back(carPose_pos);
        points.points.push_back(forwardPt);
        line_strip.points.push_back(carPose_pos);
        line_strip.points.push_back(forwardPt);
    }

    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    
    odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    return odom_car2WayPtVec;
}


double L1Controller::getEta(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);

    double eta = atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x);
    return eta;
}


double L1Controller::getCar2GoalDist()
{
    geometry_msgs::Point car_pose = odom.pose.pose.position;
    double car2goal_x = odom_goal_pos.x - car_pose.x;
    double car2goal_y = odom_goal_pos.y - car_pose.y;

    double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);

    return dist2goal;
}

double L1Controller::getL1Distance(const double& _Vcmd)
{
    double L1 = 0;
    if(_Vcmd < 1.34)
        L1 = 3 / 3.0;
    else if(_Vcmd > 1.34 && _Vcmd < 5.36)
        L1 = _Vcmd*2.24 / 3.0;
    else
        L1 = 12 / 3.0;
    return L1;
}

double L1Controller::getSteeringAngle(double eta)
{
    double steering_angle = -atan2((L*sin(eta)),(Lfw/2+lfw*cos(eta)))*(180.0/PI);
    //ROS_INFO("Steering Angle = %.2f", steering_angle);
    return steering_angle;
}

double L1Controller::getGasInput(const float& current_v)
{
    double u = (Vcmd - current_v)*gas_gain;
    //ROS_INFO("velocity = %.2f\tu = %.2f",current_v, u);
    return u;
}


void L1Controller::goalReachingCB(const ros::TimerEvent&)
{

    if(goal_received)
    {
        double car2goal_dist = getCar2GoalDist();
        if(car2goal_dist < 0.2)
        {
            goal_reached = true;
            goal_received = false;
            ROS_INFO("Goal Reached !");
        }
    }
}

void L1Controller::controlLoopCB(const ros::TimerEvent&)
{

    geometry_msgs::Pose carPose = odom.pose.pose;
    geometry_msgs::Twist carVel = odom.twist.twist;
    ackermann_cmd.speed = 0;
    ackermann_cmd.steering_angle = 0;


    if(goal_received)
    {
        /*Estimate Steering Angle*/
        double eta = getEta(carPose);  
        if(foundForwardPt)
        {
            ackermann_cmd.steering_angle = getSteeringAngle(eta)*angle_gain;
            /*Estimate Gas Input*/
            if(!goal_reached)
            {
                //double u = getGasInput(carVel.linear.x);
                //cmd_vel.linear.x = baseSpeed - u;
                ackermann_cmd.speed = base_speed;
                ROS_INFO("\nGas = %.2f\nSteering angle = %.2f",ackermann_cmd.speed,ackermann_cmd.steering_angle);
                //ROS_DEBUG("\nGas = %.2f\nSteering angle = %.2f",ackermann_cmd.speed,ackermann_cmd.steering_angle);
            }
        }
    }
    pub_.publish(ackermann_cmd);
}

void L1Controller::dynamicCB(qingzhou_nav::L1_dynamicConfig &config, uint32_t level)
{
    L = config.L;
    Lrv = config.Lrv;
    Vcmd = config.Vcmd;
    lfw = config.lfw;
    lrv = config.lrv;

    controller_freq = config.controller_freq;
    angle_gain = config.angle_gain;
    gas_gain = config.gas_gain;
    base_speed = config.base_speed;
    base_angle = config.base_angle;
    ROS_INFO("-----params have been changed----");
    ROS_INFO("base_speed:%f",base_speed);

}

/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "L1Controller_v3");
    L1Controller controller;
    ros::spin();
    return 0;
}
