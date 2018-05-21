/**
 * \file leader_node.cpp
 * \A node to generate pose and velocity signal from a vehicle which
 * follows a predefined trajectory: a circle of radius R at a given 
 * speed. The node is used to lead a platoon of vehicles in simulation.
 * \author Gaetan Garcia
 * \version 0.1
 * \date April 2017
 * 
 * \param[in] 
 * 
 * Subscribes to: 
 * 
 * Publishes to: 
 *
 */

//ROS
#include "ros/ros.h"

//ROS msgs
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <algorithm>

//Namespaces
using namespace std;

int kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	return ch;
}


ros::Publisher pubMarker ;
void publishMarkerAt( geometry_msgs::Pose2D markerPosture) {    
    visualization_msgs::Marker marker;
    std::string nodeName ;
    marker.header.stamp = ros::Time::now();
    nodeName = ros::this_node::getName();
    marker.header.frame_id = "/map";
    marker.ns = nodeName;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = markerPosture.x ;
    marker.pose.position.y = markerPosture.y ;
    marker.pose.position.z = 0.0             ;
    marker.pose.orientation.x = 0.0 ;
    marker.pose.orientation.y = 0.0 ;
    marker.pose.orientation.z = 0.0 ;
    marker.pose.orientation.w = 1.0 ;
    marker.scale.x = 0.2 ;
    marker.scale.y = 0.2 ;
    marker.scale.z = 0.2 ;
    marker.color.r = 1.0 ;
    marker.color.g = 0.0 ;
    marker.color.b = 0.0 ;
    marker.color.a = 0.5 ;
    marker.lifetime = ros::Duration();
    pubMarker.publish(marker);
}

#define FREQ 20.0

int main (int argc, char** argv)
{
    int key ;

    //ROS Initialization
    ros::init(argc, argv, "leader_node");
    ROS_INFO("leader_node connected.");
    ros::NodeHandle nh_glob, nh_loc("~");
	
	double radius, v ;
    nh_loc.param("/radius" , radius , 2.0);
    nh_loc.param("/speed"  , v      , 1.0);
    
    // Initialize vehicle posture from parameters.
    geometry_msgs::Pose2D leaderPosture ;
    geometry_msgs::Twist  leaderTwist   ;
    
    leaderPosture.x     = radius   ;
    leaderPosture.y     = 0.0      ;
    leaderPosture.theta = M_PI/2.0 ;
    double rho          = 0.0      ;  // Polar angle.
    double w            = v/radius ;

    // Advertising the vehicle posture and twist.
    ros::Publisher pubLeaderPosture
        = nh_glob.advertise<geometry_msgs::Pose2D>("rear_posture",1);
    ros::Publisher pubLeaderTwist
        = nh_glob.advertise<geometry_msgs::Twist>("rear_twist",1);
    pubMarker = nh_loc.advertise<visualization_msgs::Marker>("/visualization_marker",1) ;

    //publishMarkerAt(leaderPosture) ;
    
    ros::Rate rate(FREQ) ;
    ROS_INFO("leader_node spinning @ 20Hz");
    //int key ;

    while (ros::ok()){

        // Attend callbacks (none here).
        ros::spinOnce();

        key = kbhit() ;
		switch(key){
		    case (int)'+':
		        radius = fmin( radius+1 , 5 ) ;
		    break;
		    case (int)'-':
		        radius = fmax( radius-1 , 1 ) ;
		    break;
		    case (int)'s':
		        v = fmax( radius-1 , 1 ) ;
		    break;
		    case (int)'f':
		        v = fmin( v+1 , 5 ) ;
		    break;
		}
		
		w    = v/radius ;
		rho += w*(1/FREQ) ;
        leaderPosture.x     = radius*cos(rho)  ;
        leaderPosture.y     = radius*sin(rho)  ;
        leaderPosture.theta = rho+M_PI/2.0     ;
        leaderTwist.linear.x = v ;
        leaderTwist.linear.y  = leaderTwist.linear.z  = 0 ;
        leaderTwist.angular.x = leaderTwist.angular.y = 0 ;
        leaderTwist.angular.z = w ;

        pubLeaderPosture.publish(leaderPosture) ;
        publishMarkerAt(leaderPosture) ;
        pubLeaderTwist.publish(leaderTwist) ;   
        
        rate.sleep();
    } 

    ROS_INFO("ROS-Node Terminated\n");
}
