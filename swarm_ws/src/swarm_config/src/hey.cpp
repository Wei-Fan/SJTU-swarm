#include <ros/ros.h>
#include <iostream>
#include <stdio.h>

using namespace std;

void iteration(const ros::TimerEvent& e)
{
	ROS_INFO("Hey, swarm!");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hey_swarm");
	ros::NodeHandle n;

	ros::Rate rate(1);
	ros::Timer timer = n.createTimer(ros::Duration(1), iteration);
    
	ros::spin();

	return 0;
}