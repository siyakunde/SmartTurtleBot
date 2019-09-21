#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/BumperEvent.h>

#include <iostream>
#include <sstream>

#include "driver.h"

//Constructor
Driver::Driver(){
	//subscribe to /scan topic to receive laser scanner data. This is used to detect obstacles.
	scan_sub_ = n_.subscribe("scan", 1000, &Driver::onScan, this);
	//subscribe to /odom topic to receive odometry data. This is used to calculate total distance travelled.
	pose_sub_ = n_.subscribe("odom", 1000, &Driver::onPose, this);
	//subscribe to the /mobile_base/events/bumper topic. This is used to detect if the robot ran head-first into an obstacle.
	bumper_sub_ = n_.subscribe("mobile_base/events/bumper", 1000, &Driver::onBumperHit, this);
	//register to advertise over /cmd_vel_mux/input/navi topic. This is used to turn the robot away from obstacles.
	pub_move_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1000);

	RANGE_MIN = 0;
	RANGE_MAX = 1;

	currPoseX = 0;
	currPoseY = 0;
	//distance travelled so far out of the totalDistance argument.
	distanceDriven = 0;

	angle_[0]=0; distance_[0]=0.4;
	angle_[1]=1.2; distance_[1]=0;
	angle_[10]=1.57; distance_[10]=0;
	angle_[11]=1.57; distance_[11]=-0.2;
	angle_[100]=-1.2; distance_[100]=0;
	angle_[101]=3; distance_[101]=0;
	angle_[110]=-1.57; distance_[110]=-0.2;
	angle_[111]=3; distance_[111]=-0.5;
	angle_[3]=3; distance_[3]=-0.5;
}

//Getter for distanceDriven
double Driver::getDistanceDriven (){
	// ROS_ERROR_STREAM("distanceDriven: "<<distanceDriven);
	return distanceDriven;
}

//Callback for the scan_sub_ subscriber, so that once laser scanner has some data, we can process it to avoid obstacles.
void Driver::onScan (const sensor_msgs::LaserScan::ConstPtr& scanData){
	RANGE_MIN = scanData->range_min;
	RANGE_MAX = scanData->range_max;
	// ROS_ERROR_STREAM("RANGE_MIN: "<<RANGE_MIN<<" RANGE_MAX: "<<RANGE_MAX);

	isObstacleInZone(scanData);
	move();
}

//The robot's vision is divided into 3 zones
//Checks in which zone the data is present
void Driver::isObstacleInZone(const sensor_msgs::LaserScan::ConstPtr& scanData){
	int i = 0, r_size = (scanData->ranges).size();
	float r = 0.0, check_distance = 0.5;

	for(i=0; i<r_size; i++){
		r = scanData->ranges[i];

		if(r > RANGE_MIN && r < RANGE_MAX){
			zone1 = (i > r_size/2 && i < r_size && r <= check_distance) ? 1 : 0;
			zone2 = (i >= r_size/3 && i <= r_size/2 && r <= check_distance) ? 1 : 0;
			zone3 = (i > 0 && i < r_size/3 && r <= check_distance) ? 1 : 0;
		}
	}
}

//Based on zone data, move the robot
void Driver::move(){
	// ROS_ERROR_STREAM("z1: "<<zone1<<" z2: "<<zone2<<" z3: "<<zone3);
	int zone_code = StringToInt(Driver::IntToString(zone1) + Driver::IntToString(zone2) + Driver::IntToString(zone3));
	geometry_msgs::Twist move;
	std::cout << zone_code;
	move.linear.x = distance_[zone_code];
	move.angular.z = angle_[zone_code];
	// ROS_ERROR_STREAM("linear.x: "<<distance_[zone_code]<<" angular.z: "<<angle_[zone_code]);
	pub_move_.publish(move);

	reset();
}

//Callback for pose_sub_ subsciber. It stores the previous and current positions
void Driver::onPose (const nav_msgs::Odometry::ConstPtr& poseData){
	prevPoseX = currPoseX;
	currPoseX = poseData->pose.pose.position.x;

	prevPoseY = currPoseY;
	currPoseY = poseData->pose.pose.position.y;

	Driver::onDistanceDriven();
}

//Calculates the distance driven by the robot
void Driver::onDistanceDriven (){
	float x = prevPoseX - currPoseX;
	float y = prevPoseY - currPoseY;
	double currentDistanceDriven = sqrt(x*x + y*y);

	distanceDriven = distanceDriven + currentDistanceDriven;
}

//Callback for bumper_sub_ subscriber. If bumper is pressed, GET THE ROBOT OUT OF THERE!
void Driver::onBumperHit (const kobuki_msgs::BumperEvent::ConstPtr& bumperData){
	if(bumperData->state == kobuki_msgs::BumperEvent::PRESSED){
		int k;
		for(k=0; k<=5000; k++){
			zone1 = 0;
			zone2 = 0;
			zone3 = 3;
			move();
		}	
	}
}

//Reset the zone values
void Driver::reset(){
	zone1 = 0;
	zone2 = 0;
	zone3 = 0;
}

//Converts integer to string
std::string Driver::IntToString(int a){
    std::ostringstream temp;
    temp<<a;
    return temp.str();
}

//Converts string to integer
int Driver::StringToInt(std::string s){
	int i;
	std::istringstream ss(s);
	ss >> i;
	return i;
}