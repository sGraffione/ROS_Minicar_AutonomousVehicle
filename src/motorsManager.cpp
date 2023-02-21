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

#define IN1 24 // LEFT
#define IN2 23 // LEFT
#define IN3 22 // RIGHT
#define IN4 25  // RIGHT
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
		
		digitalWrite(6,HIGH);
		digitalWrite(19,LOW);
		digitalWrite(13,HIGH);
		digitalWrite(26,LOW);
			
			
		// set forward as default
		//this->setForward();
		
		this->pca9685.SetFrequency(50);

		// MG996R Servo Motor
		this->servo.SetLeftUs(600);
		this->servo.SetRightUs(2800);
	}
	
	~motorsInterface(){}
	/*
	void setForward(){
		// left
		bcm2835_gpio_write(13,HIGH);
		bcm2835_gpio_write(19,LOW);
		// right
		bcm2835_gpio_write(6,HIGH);
		bcm2835_gpio_write(25,LOW);
	}
	
	void setBackward(){
		// left
		bcm2835_gpio_write(13,LOW);
		bcm2835_gpio_write(19,HIGH);
		// right
		bcm2835_gpio_write(6,LOW);
		bcm2835_gpio_write(25,HIGH);
	}
	*/
	void listenerCallback(const minicar::Motors& msg){
		ROS_INFO("motors: [%f %f]", msg.throttle, msg.steering);
		
		int th = (int)(msg.throttle*MAX_DUTY_CYCLE/MAX_SPEED);
		ROS_INFO("[%i]",th);
		double steer = msg.steering*180/M_PI;
		/*
		if(th >= 0){
			this->setForward();
		}else{
			this->setBackward();
			th = -1*th;
		}*/
		ROS_INFO("[%i]",th);
		this->pca9685.Write(CHANNEL(ENA),VALUE(th));
		this->pca9685.Write(CHANNEL(ENB),VALUE(th));
		this->servo.SetAngle(CHANNEL(15), ANGLE(CENTER+steer));
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
	
	return 0;
}
