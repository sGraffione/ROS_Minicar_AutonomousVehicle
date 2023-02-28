#include "ros/ros.h"
#include <signal.h>
#include "minicar/Motors.h"
#include "minicar/BtsData.h"

#include <cstdio>
#include <cstdint>
#include <unistd.h>
#include <math.h>
#include <wiringPi.h>

#include "bcm2835.h"
#include "lib-pca9685/include/pca9685servo.h"

// WiringPi PinOut
#define IN1 4 // LEFT
#define IN2 5 // LEFT
#define IN3 3 // RIGHT
#define IN4 2  // RIGHT
// BCM PinOut
#define INbcm1 19 // LEFT
#define INbcm2 13 // LEFT
#define INbcm3 6 // RIGHT
#define INbcm4 16  // RIGHT

#define ENA 0  // LEFT
#define ENB 1  // RIGHT

#define CENTER 63
#define MAX_DELTA 30*180/M_PI
#define MAX_DUTY_CYCLE 4000
#define MAX_SPEED 0.3

class motorsInterface {
	public:
		int bcmRes = bcm2835_init();
		int wirPiRes = wiringPiSetup();
		PCA9685Servo servo;
		PCA9685 pca9685;
		
	motorsInterface(){
		printf("Init obj");
		
		if (bcmRes != 1) {
			fprintf(stderr, "bcm2835_init() failed\n");
		}
		if (wirPiRes == -1){
			fprintf(stderr,"ERROR: WIRINGPI setup error");
			exit(-1);
		}
		
		ROS_INFO("Setting PinMode");
		/*
		// left
		bcm2835_gpio_write(INbcm1,1);
		bcm2835_gpio_write(INbcm2,0);
		// right
		bcm2835_gpio_write(INbcm3,1);
		bcm2835_gpio_write(INbcm4,0);
		*/
		digitalWrite(IN1,0);
		digitalWrite(IN2,1);
		digitalWrite(IN3,1);
		digitalWrite(IN4,0);
			
			
		// set forward as default
		//this->setForward();
		
		this->pca9685.SetFrequency(50);

		// MG996R Servo Motor
		this->servo.SetLeftUs(600);
		this->servo.SetRightUs(2800);
	}
	
	~motorsInterface(){}
	
	void setForward(){
		digitalWrite(IN1,0);
		digitalWrite(IN2,1);
		digitalWrite(IN3,1);
		digitalWrite(IN4,0);
	}
	
	void setBackward(){
		digitalWrite(IN1,1);
		digitalWrite(IN2,0);
		digitalWrite(IN3,0);
		digitalWrite(IN4,1);
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
		pinMode(IN1,INPUT);
		pinMode(IN2,INPUT);
		pinMode(IN3,INPUT);
		pinMode(IN4,INPUT);
		
	}
	
};


int main(int argc, char **argv) {
	
	ros::init(argc,argv,"MotorsManager");
	ROS_INFO("Ros init");
	ros::NodeHandle n;
	
	motorsInterface motorsInt;
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(ENA, PWM_OUTPUT);

	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);
	pinMode(ENB, PWM_OUTPUT);
	
	ros::Subscriber sub = n.subscribe("motorsCtrl",1,&motorsInterface::listenerCallback, &motorsInt);

	if (getuid() != 0) {
		fprintf(stderr, "Program is not started as \'root\' (sudo)\n");
		return -1;
	}

	ros::spin();
	motorsInt.reset();
	return 0;
}
