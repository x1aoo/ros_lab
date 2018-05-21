/**
 * \file control_node.cpp
 * \A node to control a virtual vehicle in a simplistic 
 * \platoon simulation.
 * \author Gaetan Garcia
 * \version 0.1
 * \date April 2017
 * 
 *
 * Implements a very simple control. A point at distance "dist" from the
 * reference point of the vehicle is controlled to coindice with the poition
 * of the front vehicle.
 */

//ROS
#include "ros/ros.h"

//ROS msgs
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

//Namespaces
using namespace std;

//Global variables

ros::Publisher pubMarker ;
void publishMarkerAt( geometry_msgs::Pose2D markerPosture) {    
    visualization_msgs::Marker marker;
    std::string nodeName ;
    marker.header.stamp = ros::Time::now();
    nodeName = ros::this_node::getName();
    marker.header.frame_id = "/map";
    marker.ns = nodeName;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = markerPosture.x ;
    marker.pose.position.y = markerPosture.y ;
    marker.pose.position.z = 0.0             ;
    marker.pose.orientation.x = 0.0 ;
    marker.pose.orientation.y = 0.0 ;
    marker.pose.orientation.z = sin(markerPosture.theta/2.0) ;
    marker.pose.orientation.w = cos(markerPosture.theta/2.0) ;
    marker.scale.x = 0.5 ;
    marker.scale.y = 0.2 ;
    marker.scale.z = 0.2 ;
    marker.color.r = 0.0 ;
    marker.color.g = 0.0 ;
    marker.color.b = 1.0 ;
    marker.color.a = 0.5 ;
    marker.lifetime = ros::Duration();
    pubMarker.publish(marker);
}

geometry_msgs::Twist vw ; // Latest control twist received

bool controlReceived = false ;

void controlCallback(geometry_msgs::Twist control){
    // Copy control to a global variable.
    vw = control ;
    controlReceived = true ;
}

#define FREQ 20.0

int main (int argc, char** argv)
{

    //ROS Initialization
    ros::init(argc, argv, "vehicle_node");
    ROS_INFO("vehicle_node connected.");
    ros::NodeHandle nh_glob, nh_loc("~");
	
	double x0, y0, theta0 ;
    nh_loc.param("x0"     , x0    , 0.0);
    nh_loc.param("y0"     , y0    , 0.0);
    nh_loc.param("theta0" , theta0, 0.0);
    
    // Initialize vehicle posture from parameters.
    geometry_msgs::Pose2D vehiclePosture ;
    vehiclePosture.x     = x0     ;
    vehiclePosture.y     = y0     ;
    vehiclePosture.theta = theta0 ;

    // Subscribing to the control twist
    ros::Subscriber subControl
        = nh_glob.subscribe<geometry_msgs::Twist> ("control",1,controlCallback);
	
    // Advertising the vehicle posture and twist and a marker of its posture for rviz.
    // The vehicle node is intended to be used withing a group including control node
    // and vehicle node. In this group, the vehicle is the rear vehicle.
    ros::Publisher pubVehiclePosture
        = nh_glob.advertise<geometry_msgs::Pose2D>("rear_posture",1);
    ros::Publisher pubVehicleTwist
        = nh_glob.advertise<geometry_msgs::Twist>("rear_twist",1);
    pubMarker = nh_loc.advertise<visualization_msgs::Marker>("/visualization_marker",1) ;

    publishMarkerAt(vehiclePosture) ;
    
    ros::Rate rate(FREQ) ;
    ROS_INFO("vehicle_node spinning @ 20Hz");

    while (ros::ok()){

        // Attend callbacks
        ros::spinOnce();

        if( controlReceived ){ 
            double theta, deltaD, deltaTheta ;
            theta = vehiclePosture.theta ;
            deltaD     = vw.linear.x *(1/FREQ) ;
            deltaTheta = vw.angular.z*(1/FREQ) ;
            vehiclePosture.x += deltaD*cos(theta) ;
            vehiclePosture.y += deltaD*sin(theta) ;
            theta            += deltaTheta        ;
            vehiclePosture.theta = theta ;
            controlReceived = false ;
        }
        publishMarkerAt(vehiclePosture) ;
        pubVehiclePosture.publish(vehiclePosture) ;
        pubVehicleTwist.publish(vw) ;

        rate.sleep();
    } 

    ROS_INFO("ROS-Node Terminated\n");
}
