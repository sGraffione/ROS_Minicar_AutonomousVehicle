#include "ros/ros.h"
#include "minicar/BtsData.h"

#include <regex>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstring>
#include <iostream>
#include <fstream>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#define POSITION_STRING_SIZE 18
#define POSITION_STRING_OFFSET 3
#define BUFFER_SIZE 256

float Ts = 0.1;

int main(int argc, char **argv){
	ROS_INFO("Init ROS");
	ros::init(argc, argv, "localization");
	ros::NodeHandle n;
	ROS_INFO("Connected to roscore");
	ros::Publisher current_pub = n.advertise<minicar::BtsData>("position",1);
	
	ros::Rate loop_rate(1/Ts);
	
	ROS_INFO("Open device");
	int serial_port = open("/dev/ttyACM0", O_RDWR);

	//check for errors
	if(serial_port<0){
		printf("Error %i from open: %s\n", errno, strerror(errno));
	}

	struct termios tty;
	if(tcgetattr(serial_port, &tty) != 0){
		printf("Error %i from tcgetattr: %s\n",errno,strerror(errno));
	}
	// Control modes
	tty.c_cflag &= ~PARENB; // clear parity bit, disable it, most common
	tty.c_cflag &= ~CSTOPB; // cler stop filed, only one stop bit used in communication (most common)
	tty.c_cflag &= ~CSIZE; //Clear all the size bits, then set it
	tty.c_cflag |= CS8; // 8 bits per byte
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common). Flow control checks for a ready signal from hardware when some data is avaiable.
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ and ignore ctrl lines (CLOCAL = 1)
	// Local modes
	tty.c_lflag &= ~ICANON; // non-canonical input mode. It will not wait for a new line character.
	tty.c_lflag &= ~ECHO;   // Disable echo
	tty.c_lflag &= ~ECHOE;	// Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG;	// Disable interpretation of INTR, QUIT and SUSP
	//  Input modes
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software control flow
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes
	// Output modes
	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
	// Timeout modes
	tty.c_cc[VTIME] = 5;
	tty.c_cc[VMIN] = 0;
	// Baudrate
	cfsetispeed(&tty, B115200); // input baudrate
	cfsetispeed(&tty, B115200); // output baudrate
	
	// save tty settings
	if (tcsetattr(serial_port, TCSANOW, &tty) != 0){
		printf("Error %i from tcsetattr: %s\n",errno,strerror(errno));
	}
	
	
	// Wake up Tag
	unsigned char msg[] = {'\r'};
	unsigned char wu_msg[] = {'\r','\r'};
	unsigned char mode_msg[] = {'l','e','s','\r'}; // Position plus quality factor

	ROS_INFO("Wake up device");
	write(serial_port,msg,sizeof(msg));
	sleep(2);
	write(serial_port,wu_msg,sizeof(wu_msg));
	sleep(2);
	write(serial_port,mode_msg,sizeof(mode_msg));
	sleep(1);
	
	ROS_INFO("Allocate memory for buffers and stuff");
	
	// Prepare read buffer
	char read_buf[BUFFER_SIZE];
	minicar::BtsData btsData;
	size_t counter = -1;

	char part[BUFFER_SIZE];
	char *token;
	int brack1_pos = 0;
	int brack2_pos = 0;

	std::ofstream logFile;
logFile.open("/home/pi/state.dat", std::ios::out | std::ios::binary);
	std::stringstream strDatFile(std::stringstream::out | std::stringstream::binary);

	ros::Duration(2).sleep();
	ROS_INFO("Start reading from device");
	while(ros::ok()){

		int recBytes = read(serial_port, &read_buf, sizeof(read_buf));
		read_buf[recBytes] = '\0'; //end string terminator at the end of read bytes. It permits to have a clean and constant reading of the incoming datas
		if (recBytes < 0){
			printf("Error reading: %s",strerror(errno));
			break;
		}
		//ROS_INFO("Read %i bytes",recBytes);
		for (size_t i = 0; i < BUFFER_SIZE-2; i++){
			if((read_buf[i] == 'e')&(read_buf[i+1] == 's')&(read_buf[i+2] == 't')){
				//printf("%i --> %s\n",i,read_buf);
				
				// search for the brackets
				brack1_pos = i+3; // I know that after 'est' comes an open bracket '['
				for (size_t j = i+3; j < BUFFER_SIZE; j++){
					if(read_buf[j] == ']'){
						brack2_pos = j;
						break;
					}
				}
				// get what's in between
				memcpy(part,read_buf+brack1_pos+1,brack2_pos-brack1_pos-1);
				part[brack2_pos-brack1_pos-1] ='\0';
				//ROS_INFO("%s\n",part);
				
				//#############OK#############//
				
				int iCurPos = 0;
				token = strtok(part,",");
				//printf("iCurPos: %i | token: %s\n", iCurPos, token);
				while(token!=NULL){
					
					if (iCurPos == 3){
						btsData.quality = (int8_t)atoi(token);
					}else{
						btsData.position[iCurPos] = (float_t)atof(token);
					}
					iCurPos++;
					
					token = strtok(NULL,",");
					//printf("iCurPos: %i | token: %s\n", iCurPos, token);
				}

				//printf("%f %f %f with quality: %i\n",btsData.position[0],btsData.position[1],btsData.position[2],btsData.quality);
				break;
			}
		}
		ROS_INFO("Position: [%f %f]",btsData.position[0],btsData.position[1]);
		current_pub.publish(btsData);

		std::string str = std::to_string(btsData.position[0]) + " " + std::to_string(btsData.position[1]);
		strDatFile << str << std::endl;

		ros::spinOnce();
		
		loop_rate.sleep();
	}
	close(serial_port);
	logFile.write(strDatFile.str().c_str(), strDatFile.str().length());
	logFile.close();
	return 0;
}
