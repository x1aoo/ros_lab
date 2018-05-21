/**
 * \file Filename here
 * \brief Typically a one line description
 * \author ...
 * \version ...
 * \date ...
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

//--------------------------------------------------------------------------------


// Callback functions...


//--------------------------------------------------------------------------------
int main (int argc, char** argv)
{

    //ROS Initialization
    ros::init(argc, argv, "Put the node default name here");

    // Define your node handles
    // ...

    // Read the node parameters if any
    // ...

    // Declare your node's subscriptions. The form is:
    /* ros::Subscriber subscriberName = ...
       nodeHandler.subscribe<messageCategory::Type>("topicName",bufferSize,callBackFunction); */
    // ...

    // Declare your publishers. General form:
    // ros::Publisher publisherName = nodeHandler.advertise<messageCategory::Type>("topicName",bufferSize);
    // ...
    
    // Put here your node-specific initializations if any.
    // ...


    ros::Rate rate(/* */);
    while (ros::ok()){
        ros::spinOnce();

        // Your node's code goes here.
        // ...

        rate.sleep();
    }
}





