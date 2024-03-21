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
	float ax, ay, az, gr, gp, gy, omega; //Variables to store the accel, gyro and angle values

	sleep(1); //Wait for the MPU6050 to stabilize
	
	
	ros::Rate loop_rate(1/Ts);

	/*std::ofstream logFile;
	logFile.open("/home/pi/accel.dat", std::ios::out | std::ios::binary);
	std::stringstream strDatFile(std::stringstream::out | std::stringstream::binary);

	 Read Yaw initial condition
	if (n.hasParam("/accelerometer/yaw")){
		n.getParam("/accelerometer/yaw", yaw);
	}else{
		yaw = 0.0;
	}*/
	
	/*Gyroscope drift calibration*/
	std::vector<double> gyVect, axVect, ayVect;
	ROS_INFO("Calculating gyroscope offset...");
	auto start = std::chrono::high_resolution_clock::now();
	auto time = std::chrono::duration_cast<std::chrono::seconds>(start-start);
	while(time.count() <= 5){ // 5 seconds of data
		device.getGyroRaw(&gr, &gp, &gy);
		device.getAccelRaw(&ax,&ay,&az);
		gyVect.push_back(gy);
		axVect.push_back(ax);
		ayVect.push_back(ay);

		auto stop = std::chrono::high_resolution_clock::now();
		time = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
	}
	
	float offset = (std::accumulate(gyVect.begin(),gyVect.end(),0.0)/gyVect.size())*M_PI/180;
	float offsetAx = (std::accumulate(axVect.begin(),axVect.end(),0.0)/axVect.size())*M_PI/180;
	float offsetAy = (std::accumulate(ayVect.begin(),ayVect.end(),0.0)/ayVect.size())*M_PI/180;
	ROS_INFO("Done!\n Running...");
	while(ros::ok()){
		
		ros::spinOnce();
		
		double Ts = 0;
		float omega = 0;
		float accelx, accely;
		
		device.getGyroRaw(&gr, &gp, &gy);
		device.getAccelRaw(&ax, &ay, &az);
		omega = (round((gy-offset)*10000 / GYRO_SENS)/10000)*M_PI/180;
		accelx = (round((ax-offsetAx)*10000 / ACCEL_SENS)/10000)*9.81;
		accely = (round((ay-offsetAy)*10000 / ACCEL_SENS)/10000)*9.81;
		
		accelData.omega = omega;
		accelData.xAccel = accelx;
		accelData.yAccel = accely;
		current_pub.publish(accelData);
		
		loop_rate.sleep();
	}
	
	//logFile.write(strDatFile.str().c_str(), strDatFile.str().length());
	//logFile.close();
	
	return 0;
}
