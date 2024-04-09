#include "ros/ros.h"
#include "std_msgs/String.h"
#include "minicar/Motors.h"
#include "minicar/BtsData.h"
#include "minicar/accel.h"
#include "minicar/EKFstate.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include <cstdio>
#include <cstdint>
#include <unistd.h>
#include <stdio.h>
#include <sstream>
#include <cmath>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/KroneckerProduct>

#define MAX_SPEED 0.3
#define MAX_SPEED_RATE 0.05 // tuned by hand
#define MAX_DUTY_CYCLE 4000
#define MAX_DELTA 0.523599
#define MAX_DELTA_RATE 0.1
#define GAMMA_STEER 30/30
//LQT horizon (unused)
#define T 5
#define T_LESS_1 4
#define Np 15
#define Nc 5


// Motor pin configuration
int leftMotor = 23;
int dirLeftMotor = 24;

// General parameters and state variables init
float Ts = 0.1;
float position[3];
float roll = 0, pitch = 0, yaw = 0;
//double MAX_DELTA_RATE = (M_PI/3*Ts)/0.17; // computation based on datasheet of MG996R servo motor

double L = 0.14;

// This function reads the message coming from the Kalman Filter node
void stateCallback(const minicar::EKFstate& msg){
	position[0] = msg.state[0];
	position[1] = msg.state[1];
	yaw = msg.state[2];
	//ROS_INFO("(%f %f) - %f", position[0], position[1], yaw);
}

// This function reads the odometry message during Gazebo simulation (it requires setting the correct topic in the main function)
void gazeboPositionCallback(const nav_msgs::Odometry& msg){
	position[0] = msg.pose.pose.position.x;
	position[1] = msg.pose.pose.position.y;
	position[2] = msg.pose.pose.position.z;
	double x = msg.pose.pose.orientation.x;
	double y = msg.pose.pose.orientation.y;
	double z = msg.pose.pose.orientation.z;
	double w = msg.pose.pose.orientation.w;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (w * y - x * z));
    double cosp = std::sqrt(1 - 2 * (w * y - x * z));
    pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv){
	// Initialization of Roscore Node
	ros::init(argc, argv, "controller");
	ROS_INFO("Connected to roscore");
	ros::NodeHandle n;
	
	// Define publisher to send controls to the MotorManager Node
	ros::Publisher current_pub = n.advertise<minicar::Motors>("motorsCtrl",1);

	// Define publisher to control the minicar when using Gazebo (comment out the previous message) TODO: to automate using paramenters with roslaunch
	//ros::Publisher current_pub_drive = n.advertise<geometry_msgs::Twist>("/minicar_driving_controller/cmd_vel",1);
	//ros::Publisher current_pub_steer = n.advertise<std_msgs::Float64MultiArray>("/minicar_steer_controller/command",1);

	// Define subscriber to receive improved localization with Kalman Filter
	ros::Subscriber sub = n.subscribe("EKFstate",1,stateCallback);

	// Define subscriber to receive localization when using Gazebo
	//ros::Subscriber sub = n.subscribe("/localization/state",1,gazeboPositionCallback);

	// Definition of sample time
	ros::Rate loop_rate(1/Ts);

	// Vehicle parameters (meters)
	double lr = 0.07;
	double lf = 0.07;
	double width = 0.11;

	// Reading of parameters from launch files
	double q1,q2,q3,q4,q5,r1,r2;
	n.getParam("/controller/q1", q1);
	n.getParam("/controller/q2", q2);
	n.getParam("/controller/q3", q3);
	n.getParam("/controller/q2", q4);
	n.getParam("/controller/q3", q5);
	n.getParam("/controller/r1", r1);
	n.getParam("/controller/r2", r2);

	// Vehicle initial state TODO: use parameters from launch files
	double X_init = 0.6;
	double Y_init = 0.0;
	double theta_init = M_PI_2;
	
	minicar::Motors motorsCtrl;

	// Sleep for 10 seconds before starting. It gives time for Gazebo to open.
	for (int i = 0; i < 10; i++){
		ROS_INFO("Starting in %i",10-i);
		ros::Duration(1).sleep();
	}

	// ROS MAIN LOOP
	while(ros::ok()){
		
		ros::spinOnce();

		//motorsCtrl.throttle = 0.3;
		//motorsCtrl.steering = 0;

		
		// Compute left and right steer angles according to an Ackermann steering system to have a more accurate simulation
		// TO COMMENT OUT WHEN USED ON REAL MINICAR
		//ROS_INFO("[%f %f]", Vel_opt, delta_opt);
		/*
		double R = (lr+lf)/tan(delta_opt);
		double left_delta, right_delta;
		left_delta = atan((lr+lf)/(R-width/2));
		right_delta = atan((lr+lf)/(R+width/2));
		//ROS_INFO("front steering [%f %f]",left_delta,right_delta);

		cmdS.data= {left_delta, right_delta};
		cmdV.linear.x = Vel_opt;
		cmdV.linear.y = 0;
		cmdV.linear.z = 0;
		cmdV.angular.x = 0;
		cmdV.angular.y = 0;
		cmdV.angular.z = 0;

		current_pub_steer.publish(cmdS);
		current_pub_drive.publish(cmdV);
		*/


		

		// Generation of the message to send to MotorsManager node
		//current_pub.publish(motorsCtrl);


		// Pause the execution based on the loop_rate
		loop_rate.sleep();
	}
	
	return 0;
}
