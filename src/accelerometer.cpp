#include "ros/ros.h"
#include "minicar/accel.h"

#include <MPU6050.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <numeric>
#include <cstdlib>
#include <chrono>

double Ts = 0.1;

int main(int argc, char **argv){
	ros::init(argc, argv, "accelerometer");
	ROS_INFO("Connected to roscore");
	ros::NodeHandle n;
	ros::Publisher current_pub = n.advertise<minicar::accel>("accelerometer",1);
	
	minicar::accel accelData;
	
	MPU6050 device(0x68);
	float ax, ay, az, gr, gp, gy; //Variables to store the accel, gyro and angle values
	float yaw = 0;
	sleep(1); //Wait for the MPU6050 to stabilize

/*
	//Calculate the offset
	std::cout << "Calculating the offsets...\n    Please keep the accelerometer level and still\n    This could take a couple of minutes...";
	device.getOffsets(&ax, &ay, &az, &gr, &gp, &gy);
	std::cout << "Gyroscope R,P,Y: " << gr << "," << gp << "," << gy << "\nAccelerometer X,Y,Z: " << ax << "," << ay << "," << az << "\n";
*/

	//Read the current yaw angle
	
	
	ros::Rate loop_rate(1/Ts);
	device.calc_yaw = true;
	/*std::ofstream logFile;
	logFile.open("/home/pi/accel.dat", std::ios::out | std::ios::binary);
	std::stringstream strDatFile(std::stringstream::out | std::stringstream::binary);
*/
	if (n.hasParam("yaw")){
		n.getParam("/accelerometer/yaw", yaw);
	}else{
		yaw = 0.0;
	}
	
	/*Gyroscope drift calibration*/
	std::vector<double> ayVect;
	ROS_INFO("Calculating gyroscope offset...");
	auto start = std::chrono::high_resolution_clock::now();
	auto time = std::chrono::duration_cast<std::chrono::seconds>(start-start);
	while(time.count() <= 5){ // 5 seconds of data
		device.getGyro(&gr, &gp, &gy);
		ayVect.push_back(gy);
		//std::cout << time.count() << std::endl;
		auto stop = std::chrono::high_resolution_clock::now();
		time = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
	}
	float offset = (std::accumulate(ayVect.begin(),ayVect.end(),0.0)/ayVect.size())*M_PI/180;
	std::cout << offset << std::endl;
	
	//float offset = 0.5*M_PI/180;
	while(ros::ok()){
		
		ros::spinOnce();
		

		//device.getAngle(0, &gr);
		//device.getAngle(1, &gp);
		//device.calc_yaw = true;
		//device.getAngle(2, &gy);
		
		
		//std::cout << "Current angle around the roll axis: " << gr << "\n";
		//std::cout << "Current angle around the pitch axis: " << gp << "\n";
		//std::cout << "Current angle around the yaw axis: " << yaw << "\n";
		
		//Get the current accelerometer values
		//device.getAccel(&ax, &ay, &az);
		//std::cout << "Accelerometer Readings: X: " << ax << ", Y: " << ay << ", Z: " << az << "\n";

		
		//logFile << gr << " " << gp << " " << gy << " " << ax << " " << ay << " " << az << " ";
		//device.calc_yaw = false;
		
		//Get the current gyroscope values
		device.getGyro(&gr, &gp, &gy);
		//std::cout << "Gyroscope Readings: X: " << gr << ", Y: " << gp << ", Z: " << gy << "\n";
		//logFile << gr << " " << gp << " " << gy << std::endl;
		gy = gy*M_PI/180-offset;
		yaw = yaw+Ts*(gy);
		accelData.yaw = yaw;
		current_pub.publish(accelData);
		
		loop_rate.sleep();
	}
	
	//logFile.write(strDatFile.str().c_str(), strDatFile.str().length());
	//logFile.close();
	
	return 0;
}
