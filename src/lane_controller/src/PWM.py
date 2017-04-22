#!/usr/bin/python

from Adafruit_MotorHAT import Adafruit_MotorHAT
import rospy
from std_msgs.msg import Float64MultiArray
import time
import atexit

# port numbers for motors
LEFT_MOTOR_PORT = 1
RIGHT_MOTOR_PORT = 2

# max/min PWM values for left/right motors
LEFT_MOTOR_MIN_PWM = 61;
LEFT_MOTOR_MAX_PWM = 255;
RIGHT_MOTOR_MIN_PWM = 61;
RIGHT_MOTOR_MAX_PWM = 255;

# the ideal pwm for driving straight
STEADY_PWM = (LEFT_MOTOR_MAX_PWM-LEFT_MOTOR_MIN_PWM)/2;

# motors
global left_motor
global right_motor

def callback(msg):
    print(msg.data[0])
    print(msg.data[1])
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    # myMotor.run(Adafruit_MotorHAT.FORWARD)
    global left_motor, right_motor
    left_motor.setSpeed(msg.data[0])
    right_motor.setSpeed(msg.data[1])

    # myMotor.run(Adafruit_MotorHAT.RELEASE)
    # time.sleep(1.0)

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
    left_motor.setSpeed(STEADY_PWM)
    left_motor.run(Adafruit_MotorHAT.FORWARD)
    right_motor.setSpeed(STEADY_PWM)
    right_motor.run(Adafruit_MotorHAT.FORWARD)

    # turn on motor
    left_motor.run(Adafruit_MotorHAT.RELEASE)
    right_motor.run(Adafruit_MotorHAT.RELEASE)

    # ros node initialization and callback
    rospy.init_node('pwm_driver', anonymous=True)
    rospy.Subscriber("motor_pwm", Float64MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()