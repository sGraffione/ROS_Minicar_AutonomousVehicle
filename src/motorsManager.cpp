#include "ros/ros.h"
#include <signal.h>
#include "minicar/Motors.h"
#include "minicar/BtsData.h"

#include <cstdio>
#include <cstdint>
#include <unistd.h>
#include <math.h>

#include "bcm2835.h"
#include "lib-pca9685/include/pca9685servo.h"
#include <pigpio.h>

// WiringPi PinOut
#define IN1 23 // RIGHT FORWARD - board pin 13
#define IN2 24 // RIGHT BACKWARD - board pin 15
#define IN3 27 // LEFT FORWARD - board pin 18
#define IN4 22  // LEFT BACKWARD - board pin 16

#define ENA 0  // LEFT
#define ENB 1  // RIGHT

#define CENTER 63
#define MAX_DELTA 25*M_PI/180 //degree* pi/180 = radians
#define MAX_DUTY_CYCLE 4000
#define MAX_SPEED 0.3

class motorsInterface {
	public:
		int bcmRes = bcm2835_init();
		PCA9685Servo servo;
		PCA9685 pca9685;
		
	motorsInterface(){
		printf("Init obj\n");
		
		if (bcmRes != 1) {
			fprintf(stderr, "bcm2835_init() failed\n");
		}
		
		ROS_INFO("Setting PinMode");
		// Pin mode settings
		gpioSetMode(ENA, PI_OUTPUT);
		gpioSetMode(ENB, PI_OUTPUT);
		gpioSetMode(IN1, PI_OUTPUT);
		gpioSetMode(IN2, PI_OUTPUT);
		gpioSetMode(IN3, PI_OUTPUT);
		gpioSetMode(IN4, PI_OUTPUT);

		// setting direction pins to 0
		gpioWrite(IN1,PI_LOW);
		gpioWrite(IN2,PI_LOW);
		gpioWrite(IN3,PI_LOW);
		gpioWrite(IN4,PI_LOW);
			
			
		// set forward as default
		//this->setForward();
		
		this->pca9685.SetFrequency(50);

		// MG996R Servo Motor
		this->servo.SetLeftUs(600);
		this->servo.SetRightUs(2800);

	}
	
	~motorsInterface(){}
	
	void setForward(){
		gpioWrite(IN1,0);
		gpioWrite(IN2,1);
		gpioWrite(IN3,1);
		gpioWrite(IN4,0);
	}
	
	void setBackward(){
		gpioWrite(IN1,1);
		gpioWrite(IN2,0);
		gpioWrite(IN3,0);
		gpioWrite(IN4,1);
	}
	
	void listenerCallback(const minicar::Motors& msg){
		ROS_INFO("motors: [%f %f]", msg.throttle, msg.steering);
		int th = (int)(msg.throttle*MAX_DUTY_CYCLE/MAX_SPEED);
		double steer = msg.steering*180/M_PI;
		
		if(th >= 0){
			this->setForward();
		}else{
			this->setBackward();
			th = -1*th;
		}
		this->pca9685.Write(CHANNEL(ENA),VALUE(th));
		this->pca9685.Write(CHANNEL(ENB),VALUE(th));
		this->servo.SetAngle(CHANNEL(15), ANGLE(CENTER-steer));
	}
	
	void reset(){
		this->pca9685.Write(CHANNEL(ENA),VALUE(0));
		this->pca9685.Write(CHANNEL(ENB),VALUE(0));
		this->servo.SetAngle(CHANNEL(15), ANGLE(CENTER));
		gpioSetMode(IN1,PI_INPUT);
		gpioSetMode(IN2,PI_INPUT);
		gpioSetMode(IN3,PI_INPUT);
		gpioSetMode(IN4,PI_INPUT);

		//Free resources & GPIO access
   		gpioTerminate();
	}
	
};


int main(int argc, char **argv) {
	
	ros::init(argc,argv,"MotorsManager");
	ROS_INFO("Ros init");
	ros::NodeHandle n;
	if(gpioInitialise()<0){
		fprintf(stderr, "PiGPIO initialisation failed.\n\r");
		return -1;
	}

	motorsInterface motorsInt;

	ros::Subscriber sub = n.subscribe("motorsCtrl",1,&motorsInterface::listenerCallback, &motorsInt);

	if (getuid() != 0) {
		fprintf(stderr, "Program is not started as \'root\' (sudo)\n");
		return -1;
	}

	ros::spin();
	motorsInt.reset();
	return 0;
}
