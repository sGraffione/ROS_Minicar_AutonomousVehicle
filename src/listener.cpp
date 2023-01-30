#include "ros/ros.h"
#include "std_msgs/String.h"

void listenerCallback(const std_msgs:: String::ConstPtr& msg){
	ROS_INFO("State: [%s]", msg->data.c_str());
}

int main(int argc, char **argv){
	ros::init(argc,argv,"listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("state",1,listenerCallback);
	ros::spin();
	return 0;
}
