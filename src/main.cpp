#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

//#include "driver.h"
#include "driver.cpp"

int main(int argc, char **argv){

	ros::init(argc, argv, "main");

	Driver neelesh;
	double totalDistance;
	ros::param::param("totalDistance", totalDistance, 5.55);
	// ROS_ERROR_STREAM("totalDistance: "<<totalDistance);
	
	// ros::spin();
	ros::Rate loop_rate(1000);
	while (ros::ok() && (neelesh.getDistanceDriven() <= totalDistance)){
		// ROS_ERROR_STREAM("distanceDriven: "<<neelesh.getDistanceDriven()<<" totalDistance: "<<totalDistance);
		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}