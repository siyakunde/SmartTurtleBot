
# Goal</br>

The goal of this project is to apply the skills from architecting, implementing, deploying, and simulating a robotic system.</br>

# Task</br>

Extend ROS TurtleBot so that it is able to use its scanner to detect and avoid obstacles.</br>

The extension should consists of a package containing the necessary build, header, sources, and launch files.  </br>

Smart TurtleBot should navigate in a straight line until it detects an obstacle within 0.5m</br>

Once an obstacle is detected, your Smart TurtleBot should take an evasive measure. You have freedom to explore different evasive strategies to maximize coverage.</br>

Smart TurtleBot should take two launch parameters</br>

(X,Y): starting location</br>

M: distance to cover (in meters) before stopping</br>

To showcase your system, create two Gazebo worlds:</br>

An office bounded by four walls and a few obstacles</br>

Two offices connected by a hallway that is narrower than the offices</br>

-------------------------------------------------------------------------
# To Run: </br>

Checkout the git project in your workspace src directory with,</br>

Go back to your workspace directory and build the packages with,</br>

`catkin_make`</br>

To demo the turtlebot in an office use the following,</br>

`roslaunch ser_assignment3 simple_office.launch `</br>

To demo the turtlebot in a setting with two offices use the the following,</br>

`roslaunch ser_assignment3 multi_office.launch `</br>

Following commandline arguments are present,</br>

totalDistance : Provide the total distance, after travelling which, the robot should exit</br>
initial_pose_x : Initial position x-coordinate to place the robot</br>
initial_pose_y : Initial position y-coordinate to place the robot</br>

Default values have already been provided for the arguments. However, any of the above mentioned roslaunch commands can be modifies as follows,</br>

`roslaunch ser_assignment3 simple_office.launch totalDistance:=100 initial_pose_x:=2.5 initial_pose_y:=3.5`</br>

The demo videos can be viewed:</br>
Simple-Office : https://youtu.be/rDg3b0yn8RU</br>
Multi-Office : https://youtu.be/D8Esu1peHZ8</br>