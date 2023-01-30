#include "ros/ros.h"
#include "std_msgs/String.h"
#include "minicar/Motors.h"
#include "minicar/BtsData.h"

#include <cstdio>
#include <cstdint>
#include <unistd.h>
#include <stdio.h>
#include <sstream>
#include <cmath>

int main(int argc, char **argv){
	ros::init(argc, argv, "accelerometer");
	ROS_INFO("Connected to roscore");
	ros::NodeHandle n;
	

	
	ros::Rate loop_rate(1/Ts);

	while(ros::ok()){
		
		ros::spinOnce();

		

		
		loop_rate.sleep();
	}
	
	return 0;
}
