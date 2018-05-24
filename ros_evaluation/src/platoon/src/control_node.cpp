/**
 * \file control_node
 * \brief a control node to control the front and rear vehicles
 * \author xiao
 * \version 1.0
 * \date 24/05/2018
 *
 * \param[in] ... Describe node parameters
 *
 * Subscribes to: <BR>
 *    ° Give name and type of topics to which the node subscribes <BR>
 *    ° ... <BR>
 *
 * Publishes to: <BR>
 *    ° Give name and type of topics to which the node publishes <BR>
 *    ° ... <BR>
 *
 * Comment about the node.
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

//Namespaces
using namespace std;

// Include here the ".h" files corresponding to the topic type you use.
// e.g. #include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

//--------------------------------------------------------------------------------
// You may have a number of globals here.
//...

//font
geometry_msgs::Pose2D front_pose;
geometry_msgs::Twist front_twist;
//rear
geometry_msgs::Pose2D rear_pose;
geometry_msgs::Twist rear_twist;
//control
geometry_msgs::Twist control_twist;

//--------------------------------------------------------------------------------


// Callback functions...

void getFPoseValue(geometry_msgs::Pose2D pose2d){
    front_pose = pose2d;
}

void getFTwistValue(geometry_msgs::Twist twist){
    front_twist = twist;

}
void getRPoseValue(geometry_msgs::Pose2D pose2d){
    rear_pose = pose2d;
}

void getRTwistValue(geometry_msgs::Twist twist){
    rear_twist = twist;
}

//--------------------------------------------------------------------------------

#define DEFAULT 1.0
int main (int argc, char** argv)
{

    //ROS Initialization
    ros::init(argc, argv, "control");

    // Define your node handles
    // ...
    ros::NodeHandle nh_glob, nh_loc("~");

    // Read the node parameters if any
    // ...
    double dist, Kp;
    nh_loc.param("dist",dist,DEFAULT);
    nh_loc.param("Kp",Kp,DEFAULT);

    // Declare your node's subscriptions. The form is:
    /* ros::Subscriber subscriberName = ...
       nodeHandler.subscribe<messageCategory::Type>("topicName",bufferSize,callBackFunction); */
    // ...

    ros::Subscriber front_pose_S
            = nh_glob.subscribe<geometry_msgs::Pose2D> ("front_posture",1,getFPoseValue);
    ros::Subscriber front_twist_S
            = nh_glob.subscribe<geometry_msgs::Twist> ("front_twist",1,getFTwistValue);
    ros::Subscriber rear_pose_S
            = nh_glob.subscribe<geometry_msgs::Pose2D> ("rear_posture",1,getRPoseValue);
    ros::Subscriber rear_twist_S
            = nh_glob.subscribe<geometry_msgs::Twist> ("rear_twist",1,getRTwistValue);

    // Declare your publishers. General form:
    // ros::Publisher publisherName = nodeHandler.advertise<messageCategory::Type>("topicName",bufferSize);
    // ...

    ros::Publisher PubControlTwist
            = nh_glob.advertise<geometry_msgs::Twist>("control",1);
    // Put here your node-specific initializations if any.
    // ...


    // rate(frequence - hz)
    ros::Rate rate(20.0);
    while (ros::ok()){
        ros::spinOnce();

        // Your node's code goes here.
        // ...

        geometry_msgs::Pose2D pval;
        geometry_msgs::Twist tval1;
        geometry_msgs::Twist tval2;
        //equations:
        pval.x = rear_pose.x + dist * cos( rear_pose.theta );
        pval.y = rear_pose.y + dist * sin( rear_pose.theta );

        tval1.angular.x = front_twist.linear.x * cos( front_pose.theta ) + Kp * (front_pose.x - pval.x);
        tval2.angular.x = front_twist.linear.x * sin( front_pose.theta ) + Kp * (front_pose.y - pval.y);
        //线速度使用x，角速度使用z
        control_twist.linear.x = tval1.angular.x * cos( rear_pose.theta ) + tval2.angular.x * sin (rear_pose.theta );
        control_twist.angular.z = ( 1 / dist ) * ( -tval1.angular.x * sin(rear_pose.theta ) + tval2.angular.x * cos(rear_pose.theta ));
        control_twist.linear.y = 0;
        control_twist.linear.z = 0;
        control_twist.angular.y = 0;
        control_twist.angular.x = 0;
        PubControlTwist.publish(control_twist);


        rate.sleep();
    }
}





