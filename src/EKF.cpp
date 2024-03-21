#include "ros/ros.h"
#include "std_msgs/String.h"
#include "minicar/BtsData.h"
#include "minicar/accel.h"
#include "minicar/EKFstate.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstring>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <vector> 
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <boost/cstdint.hpp>
 
using Eigen::MatrixXd;

float x,y,acc,vel_yaw;
float Ts = 0.1;

void positionCallback(const minicar::BtsData& msg){
	x = msg.position[0];
	y = msg.position[1];
	ROS_INFO("x: %f - y: %f",x,y);
}

void accelerometerCallback(const minicar::accel& msg){
	vel_yaw = msg.omega;
	acc = msg.xAccel;
	ROS_INFO("omega: %f - acc: %f",vel_yaw,acc);
}

//Compute the jacobian 
MatrixXd J_function(Eigen::VectorXd X,double dt){
double theta_k = X(2);
double v_k = X(3);
double a_k = X(4);
MatrixXd J(6,6);

J<<1, 0, -dt*v_k*sin(theta_k)-0.5*pow(dt,2)*a_k*sin(theta_k), dt*cos(theta_k), 0.5*pow(dt,2)*cos(theta_k), 0,
   0, 1, +dt*v_k*cos(theta_k)+0.5*pow(dt,2)*a_k*cos(theta_k), dt*sin(theta_k), 0.5*pow(dt,2)*sin(theta_k), 0,
   0, 0,                         1,                                    0,           0,                    dt,
   0, 0,                         0,                                    1,          dt,                     0,
   0, 0,                         0,                                    0,           1,                     0,
   0, 0,                         0,                                    0,           0,                     1;         

return J;
}

//Compute the Next Values
Eigen::VectorXd X_values(Eigen::VectorXd Xprev,double dt,Eigen::VectorXd u){
   Eigen::VectorXd X(6);
   //X Y yaw velLong accLong velAng
   
   X(0)=Xprev(0)+(Xprev(3)*dt*cos(Xprev(2))) + (0.5*pow(dt,2)*Xprev(4)*cos(Xprev(2)));
   X(1)=Xprev(1)+(Xprev(3)*dt*sin(Xprev(2))) + (0.5*pow(dt,2)*Xprev(4)*sin(Xprev(2)));
   X(2)=Xprev(2)+Xprev(5)*dt;
   X(3)=Xprev(3)+Xprev(4)*dt;
   X(4)=u(0);
   X(5)=u(1);
   return X;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "EKF");
	ROS_INFO("Connected to roscore");
	ros::NodeHandle n;
	
	ros::Publisher current_pub = n.advertise<minicar::EKFstate>("EKFstate",1);
	
	ros::Subscriber subPos = n.subscribe("position",1,positionCallback);
	ros::Subscriber subAccel = n.subscribe("accelerometer",1,accelerometerCallback);
	
	ros::Rate loop_rate(1/Ts);
  
    Eigen::MatrixXd B(2,6);
    Eigen::MatrixXd C(4,6);
    Eigen::MatrixXd P(6,6);
    Eigen::MatrixXd R(4,4);
    Eigen::MatrixXd Q(6,6);
    Eigen::VectorXd X_zero(6);
    Eigen::VectorXd X;
    Eigen::MatrixXd K(6,4);
    Eigen::MatrixXd J(6,6);
    Eigen::MatrixXd Id(6,6);

    Id.setIdentity();
    
    //System's matrices
    B<<0,0,0,0,1,0,
       0,0,0,0,0,1;

    C<<1,0,0,0,0,0,
       0,1,0,0,0,0,
       0,0,0,0,1,0,
       0,0,0,0,0,1;

	float yaw;
	if (n.hasParam("/accelerometer/yaw")){
		n.getParam("/accelerometer/yaw", yaw);
	}else{
		yaw = 0.0;
	}

   //Initial values
    X_zero << 0,0,yaw,0,0,0;
    X=X_zero;

    //Covariances matrices
    P.setIdentity();
    P=P*0.01;
    
    R<<0.002, 0, 0, 0,
       0, 0.002, 0, 0,
       0, 0, 0.0015, 0,
       0, 0, 0, 0.000035;

   Q<<0.0001, 0, 0, 0, 0, 0,
       0, 0.0001, 0, 0, 0, 0,
       0, 0, 0.00001,  0, 0, 0,
       0, 0, 0, 0.003, 0, 0,
       0, 0, 0,  0,  0.001,  0,
       0, 0, 0,  0,  0,  0.001;

   

 /*
   x[line]=std::stof(temp);
   y[line]=std::stof(temp);
   vel_yaw[line]=std::stod(temp)*3.14159/180;
   acc[line]=std::stod(temp)*9.81;
*/

	minicar::EKFstate estState;

	Eigen::VectorXd z(4);
	Eigen::VectorXd u(2);
	minicar::EKFstate::_state_type tempXArray;

	//EKF init
	K=P*C.transpose()*(C*P*C.transpose()+R).inverse();
	while(ros::ok()){
		ros::spinOnce();
	   
		//EKF
		z<<x,y,acc,vel_yaw;
		u<<acc,vel_yaw;
		//ROS_INFO("(%f %f) - %f", x,y,vel_yaw);
		
		X=X_values(X,Ts,u);
		J = J_function(X,Ts);
		P=J*P*J.transpose()+Q;
		K=P*C.transpose()*(C*P*C.transpose()+R).inverse();
		X=X+K*(z-C*X);
		P=(Id-K*C)*P*(Id-K*C).transpose()+K*R*K.transpose();
		ROS_INFO("(%f %f) - %f", X(0), X(1), X(2));
		double* dataPtr = X.data();
		for(int i = 0; i < X.size(); ++i){
			tempXArray[i] = static_cast<float>(dataPtr[i]);
		}
		estState.state = tempXArray;
		
		current_pub.publish(estState);
		
		loop_rate.sleep();
	}


	return 0;

}
