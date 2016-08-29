#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
//#include "turtlesim/Velocity.h>

// Function declarations
void ComPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
void CurPoseCallback(const turtlesim::Pose::ConstPtr& msg);
float GetErrorLin(turtlesim::Pose curpose, geometry_msgs::Pose2D despose);
float GetErrorAng(turtlesim::Pose curpose, geometry_msgs::Pose2D despose);
double getDistance(double x1, double y1, double x2, double y2);
// Global variables
bool STOP = true;                                                       // to hold stop flag, wait till first command given
//turtlesim::Velocity CmdVel;
turtlesim::Pose CurPose;                                                // to hold current pose
geometry_msgs::Pose2D DesPose;                                          // variable to hold desired pose
ros::Publisher pub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TurtlesimPositionController_pubsub");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

    // register sub to get desired position/pose commands
    ros::Subscriber ComPose_sub = nh.subscribe("/turtle1/PositionCommand", 5, ComPoseCallback);
    // register sub to get current position/pose
    ros::Subscriber CurPose_sub = nh.subscribe("/turtle1/pose", 5, CurPoseCallback);
    // register pub to send twist velocity (cmd_vel)
    //ros::Publisher Twist_pub = n.advertise<turtlesim::Velocity>("/turtle1/command_velocity", 100);

    ros::Rate rate(10);
    float ErrorLin = 0;
    float ErrorAng = 0;

    ROS_INFO("Ready to send position commands");                        // let user know we are ready and good
    while (ros::ok() && nh.ok() )                                        // while ros and the node are ok
    {
        ros::spinOnce();
        if (STOP == false)                                              // and no stop command
        {
            geometry_msgs::Twist msg;
            msg.linear.x = 0.5*getDistance(CurPose.x, CurPose.y, DesPose.x, DesPose.y);
            msg.linear.y = 0;
            msg.linear.z = 0;
            msg.angular.x = 0;
            msg.angular.y = 0;
            msg.angular.z = 2*(atan2(DesPose.y - CurPose.y, DesPose.x - CurPose.x)-CurPose.theta);
            pub.publish(msg);
            if (getDistance(CurPose.x, CurPose.y, DesPose.x, DesPose.y)<1)
            {
                STOP=true;
            }
        }
        else
        {
            printf("Waiting...\n");
        }
        rate.sleep();
    }
}

double getDistance(double x1, double y1, double x2, double y2){
    return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
}

// call back to send new desired Pose msgs
void ComPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)            
{
    STOP = false;                                                       // start loop
    DesPose.x = msg->x;                                                 // copy msg to varible to use elsewhere
    DesPose.y = msg->y;
    //CmdVel.angular += 1;
    return;
}

// call back to send new current Pose msgs
void CurPoseCallback(const turtlesim::Pose::ConstPtr& msg)          
{
    CurPose.x = msg->x;
    CurPose.y = msg->y;
    CurPose.theta = msg->theta;                                         // copy msg to varible to use elsewhere
    return;
}/*
float GetErrorAng(turtlesim::Pose curpose, geometry_msgs::Pose2D despose)
{
    // create error vector
    float Ex = despose.x - curpose.x;                                   // Error X. X component 
    float Ey = despose.y - curpose.y;                                   // Error Y. Y component 

    // get desire angle
    float dest = atan2f(Ey, Ex);                                        // use float version to get arc tangent

    // get angle error
    float Et = dest - curpose.theta;

    //~ ROS_INFO("Ex: %f, Ey: %f, Et: %f", Ex, Ey, Et);
    return Et;
}

// function to get linear error from the turtles perspective. Error only along turtle X axis
float GetErrorLin(turtlesim::Pose curpose, geometry_msgs::Pose2D despose)
{
    // create error vector
    float Ex = despose.x - curpose.x;                                   // Error X. X component
    float Ey = despose.y - curpose.y;                                   // Error Y. Y component 
    float Et = GetErrorAng(curpose, despose);                           // get angle between vectors

    // project error onto turtle x axis
    //~ float Etx =  pow( pow(Ex,2.0) + pow(Ey,2.0), 0.5 )*cos(Et);
    float Etx = hypotf(Ex, Ey)*cos(Et); // improved c function

    return Etx;
}*/