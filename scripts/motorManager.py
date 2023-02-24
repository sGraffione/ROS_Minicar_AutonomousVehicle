#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from minicar.msg import Motors
import RPi.GPIO as GPIO
import Adafruit_PCA9685
from adafruit_servokit import ServoKit
import time

IN1 = 23
IN2 = 24
IN3 = 27
IN4 = 22
ENA = 0
ENB = 1

PI = 3.141592653589793238

MAX_DELTA = 30*180/PI
MAX_DUTY_CYCLE = 4000
MAX_SPEED = 0.3

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)
kit = ServoKit(channels=16)
CENTER = 89
kit.servo[15].set_pulse_width_range(400,2100)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(IN1,GPIO.OUT)
GPIO.setup(IN2,GPIO.OUT)
GPIO.setup(IN3,GPIO.OUT)
GPIO.setup(IN4,GPIO.OUT)

def setForward():
	GPIO.output(IN1,GPIO.HIGH)
	GPIO.output(IN2,GPIO.LOW)
	GPIO.output(IN3,GPIO.HIGH)
	GPIO.output(IN4,GPIO.LOW)

def setBackward():
	GPIO.output(IN1,GPIO.LOW)
	GPIO.output(IN2,GPIO.HIGH)
	GPIO.output(IN3,GPIO.LOW)
	GPIO.output(IN4,GPIO.HIGH)

def callback(data):
	rospy.loginfo("[%s %s]\n", data.throttle,data.steering)
	th = data.throttle*MAX_DUTY_CYCLE/MAX_SPEED
	if data.throttle >= 0:
		setForward()
	else:
		setBackward()
	if th >= MAX_DUTY_CYCLE:
		th = MAX_DUTY_CYCLE
	elif data.throttle <= -MAX_DUTY_CYCLE:
		th = MAX_DUTY_CYCLE

	steer = data.steering*180/PI
	if steer > MAX_DELTA:
		steer = MAX_DELTA
	elif steer < -MAX_DELTA:
		steer = -MAX_DELTA

	pwm.set_pwm(ENA,0,int(th))
	pwm.set_pwm(ENB,0,int(th))
	kit.servo[15].angle = CENTER-steer
    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.loginfo("Connected to roscore")
    rospy.Subscriber("motorsCtrl", Motors, callback)
    
    #pwm.set_pwm(ENA,0,0)
    #pwm.set_pwm(ENB,0,0)
    #kit.servo[15].angle = CENTER

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


listener()
pwm.set_pwm(ENA,0,0)	
pwm.set_pwm(ENB,0,0)
kit.servo[15].angle = CENTER
