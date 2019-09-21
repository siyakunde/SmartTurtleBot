#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>

class Driver{

private:
	//NodeHandle
	ros::NodeHandle n_;
	//Laser Scanner subscriber
	ros::Subscriber scan_sub_;
	//Odometry subsciber
	ros::Subscriber pose_sub_;
	//BumperEvent subscriber
	ros::Subscriber bumper_sub_;
	//Linear and angular velocity publisher
	ros::Publisher pub_move_;

	//The values from ranges array, returned by scan data, should fall between these values
	float RANGE_MIN, RANGE_MAX;

	//Divides the robots field of vision into 3 zones to detect where the obstacle is
	int zone1, zone2, zone3;
	//Record the current and previous positions of the robot
	double prevPoseX, prevPoseY, currPoseX, currPoseY;
	//Record how much distance the robot has travelled
	double distanceDriven;

	//These maps define how the robot will act if there is/isn't an obstacle
	std::map<int,float> angle_;
	std::map<int,float> distance_;

public:

	//Constructor
	Driver();
	//Getter for distanceDriven
	double getDistanceDriven ();
	//Callback for the scan_sub_ subscriber, so that once laser scanner has some data, we can process it to avoid obstacles.
	void onScan (const sensor_msgs::LaserScan::ConstPtr& scanData);
	//The robot's vision is divided into 3 zones
	//Checks in which zone the data is present
	void isObstacleInZone(const sensor_msgs::LaserScan::ConstPtr& data);
	//Based on zone data, move the robot
	void move();
	//Callback for pose_sub_ subsciber. It stores the previous and current positions
	void onPose (const nav_msgs::Odometry::ConstPtr& poseData);
	//Calculates the distance driven by the robot
	void onDistanceDriven ();
	//Callback for bumper_sub_ subscriber. If bumper is pressed, GET THE ROBOT OUT OF THERE!
	void onBumperHit (const kobuki_msgs::BumperEvent::ConstPtr& bumperData);
	//Reset the zone values
	void reset();
	//Converts integer to string
	std::string IntToString (int a);
	//Converts string to integer
	int StringToInt(std::string s);
};