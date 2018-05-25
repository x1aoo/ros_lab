/**
 * \file
 * \brief 
 * \author 
 * \version 0.1
 * \date 
 * 
 * \param[in] 
 * 
 * Subscribes to: <BR>
 *    ° 
 * 
 * Publishes to: <BR>
 *    ° 
 *
 * Description
 *
 */


//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

//ROS
#include "ros/ros.h"

// Include here the ".h" files corresponding to the topic types you use.
// ...
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>



// You may have a number of globals here.
//...

std_msgs::Float32 radius;

geometry_msgs::Pose2D pose_error;

bool receiveE = false;

// Callback functions...

//void myFirstCallback(/* Define here the variable which hold the message */){
//    // ... Callback function code
//}
void rCallback(std_msgs::Float32 r)
{
    radius = r;

}

void errorCallback(geometry_msgs::Pose2D error)
{
    pose_error = error;
    receiveE = true;
}
#define DEFAULTKY 5.0
#define DEFAULTKTHETA 1.0
#define DEFAULTVREF 1.0

int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "Put the node default name here");

    // Define your node handles
    // ...
    ros::NodeHandle nh_("~");

    // Read the node parameters if any
    // ...
    radius.data = 1.0;
    pose_error.x = 0;
    pose_error.y = 0;
    pose_error.theta = 0;
    double Ky, Ktheta, vref;


    nh_.param("Ky", Ky, DEFAULTKY);
    nh_.param("Ky", Ktheta, DEFAULTKTHETA);
    nh_.param("vref", vref, DEFAULTVREF);


    // Declare your node's subscriptions and service clients
    // ...
    ros::Subscriber raduisSub = nh_.subscribe<std_msgs::Float32>("/radius",1,rCallback) ;
    ros::Subscriber errorSub = nh_.subscribe<geometry_msgs::Pose2D>("/pose_error",1,errorCallback) ;

    ros::Publisher  velPub  = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1) ;


    // Declare you publishers and service servers
    // ...

    ros::Rate rate(50);   // Or other rate.
	while (ros::ok()){
		ros::spinOnce();
                geometry_msgs::Twist tval;
//                if(receiveE)
//                {
                tval.angular.z = vref/radius.data + Ky * vref * pose_error.y + Ktheta * pose_error.theta;
                tval.angular.x = 0;
                tval.angular.y = 0;
                tval.linear.x = vref;
                tval.linear.y = 0;
                tval.linear.z = 0;
//                }
                velPub.publish(tval);



        // Your node's code goes here.

		rate.sleep();
    }
}
