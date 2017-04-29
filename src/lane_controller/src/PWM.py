#!/usr/bin/python

from Adafruit_MotorHAT import Adafruit_MotorHAT
import rospy
from std_msgs.msg import Float64MultiArray
import time
import atexit
import math

# port numbers for motors
LEFT_MOTOR_PORT = 1
RIGHT_MOTOR_PORT = 2

# max/min PWM values for left/right motors
LEFT_MOTOR_MIN_PWM = 80;
LEFT_MOTOR_MAX_PWM = 210 #255;
RIGHT_MOTOR_MIN_PWM = 80;
RIGHT_MOTOR_MAX_PWM = 210 #255;

# the ideal pwm for driving straight
STEADY_PWM = 70+(LEFT_MOTOR_MAX_PWM-LEFT_MOTOR_MIN_PWM)/2;

# motors
global left_motor
global right_motor

def callback(msg):
    if(math.isnan(msg.data[0]) or math.isnan(msg.data[1])):
        return
    print(int(msg.data[0]))
    print(int(msg.data[1]))
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    # myMotor.run(Adafruit_MotorHAT.FORWARD)
    global left_motor, right_motor

    # set left motor
    if(msg.data[0] > LEFT_MOTOR_MAX_PWM):
        left_motor.setSpeed(LEFT_MOTOR_MAX_PWM)
    elif(msg.data[0] < LEFT_MOTOR_MIN_PWM):
        left_motor.setSpeed(LEFT_MOTOR_MIN_PWM)
    else:
        left_motor.setSpeed(int(msg.data[0]))

    # set right motor
    if(msg.data[1] > RIGHT_MOTOR_MAX_PWM):
        right_motor.setSpeed(RIGHT_MOTOR_MAX_PWM)
    elif(msg.data[1] < RIGHT_MOTOR_MIN_PWM):
        right_motor.setSpeed(RIGHT_MOTOR_MIN_PWM)
    else:
        right_motor.setSpeed(int(msg.data[1]))

    left_motor.run(Adafruit_MotorHAT.FORWARD)
    right_motor.run(Adafruit_MotorHAT.FORWARD)
    time.sleep(0.01)

def turnOffMotors():
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)
    
def listener():

    # motor hat initialization
    mh = Adafruit_MotorHAT(addr=0x60)
    atexit.register(turnOffMotors)

    global left_motor, right_motor
    left_motor = mh.getMotor(LEFT_MOTOR_PORT)
    right_motor = mh.getMotor(RIGHT_MOTOR_PORT)

    # set the speed to start
    left_motor.setSpeed(int(STEADY_PWM))
    left_motor.run(Adafruit_MotorHAT.FORWARD)
    right_motor.setSpeed(int(STEADY_PWM))
    right_motor.run(int(Adafruit_MotorHAT.FORWARD))

    # turn on motor
    #left_motor.run(Adafruit_MotorHAT.RELEASE)
    #right_motor.run(Adafruit_MotorHAT.RELEASE)

    # ros node initialization and callback
    rospy.init_node('pwm_driver', anonymous=True)
    rospy.Subscriber("motor_pwm", Float64MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
