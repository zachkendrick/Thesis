/*************************************************
 *  lane_controller.cpp
 *  
 *  Uses error measurements from the pose estimate
 *  generated by pose_estimate.cpp to control the steering
 *  of the vehicle using a PID controller.
 *
 *  Subscribers: pose_estimate
 *  Publishers: motor_pwm
 *
 *  Author: Zachary Kendrick
 ************************************************/

#include <ros/ros.h>
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <sys/time.h>
#include <opencv-3.1.0-dev/opencv2/plot.hpp>
#include <std_msgs/Float64MultiArray.h>

using namespace cv;
using namespace std;
using namespace ros;

// PID coefficients
const static float K_P = 200;
const static float K_I = 0;
const static float K_D = 20;

// max/min PWM values for left/right motors
const static int LEFT_MOTOR_MIN_PWM = 61;
const static int LEFT_MOTOR_MAX_PWM = 255;
const static int RIGHT_MOTOR_MIN_PWM = 61;
const static int RIGHT_MOTOR_MAX_PWM = 255;

// the ideal pwm for driving straight
const static int STEADY_PWM = -20+LEFT_MOTOR_MIN_PWM+(LEFT_MOTOR_MAX_PWM-LEFT_MOTOR_MIN_PWM)/2;

// ideal position of the car
const static float CENTER_X = 315;

// ideal position of road lanes
const static float CENTER_LINE = 130;
const static float RIGHT_LINE = 530;

// the width of the road for horizontal displacement in rviz units
const static float ROAD_WIDTH = RIGHT_LINE - CENTER_LINE;

// past time and error
float prev_error = 0.0;
long long past_time;
float err_I;

// car pose publisher
ros::Publisher pwm_pub;


/*
   Function:
      Call back function that reads a pose from
      "pose_estimate" topic and produces PWM signals
      for the robot motors from a PID loop.
   Parameters:
      const visualization_msgs::MarkerConstPtr& msg - "pose_estimate" message
   Publishes:
      pwm array data[0]-left motor, data[1]-right motor
*/

void PID(const visualization_msgs::MarkerConstPtr& msg) {

    // get the error in the angle of the pose
    tf2::Quaternion quat(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    
    // angle error normalized between -1 to 1
    float angle_err = ((M_PI/2) - quat.getAngle())/(M_PI/2);

    // displacement error normalized between -1 to 1
    float disp_err = (msg->pose.position.x - CENTER_X)/(ROAD_WIDTH/2);

    // take a weighted sum of the angle and displacement error
    float error;
    if(abs(angle_err) < 0.03) error = -disp_err;
    else error = angle_err;

    // calculate time ellapsed
    timeval time;
    gettimeofday(&time, NULL);
    long long present_time = (((long long) time.tv_sec) * 1000LL) + (((long long)time.tv_usec) / 1000LL);
    float dt = 400;//(float)(present_time - past_time);

    // calculate I and D error
    err_I += (error * dt);
    float err_D = (error - prev_error)/dt;

    // compute PID output
    float output = K_P*error + K_I*err_I + K_D*err_D;

    /*Remember some variables for next time*/
    prev_error = error;
    past_time = present_time;

    // publish the PWMs from the PID output
    std_msgs::Float64MultiArray motor_pwm;
    motor_pwm.data.push_back(STEADY_PWM+output); //left_motor_pwm;
    motor_pwm.data.push_back(STEADY_PWM-output); //right_motor_pwm;
    pwm_pub.publish(motor_pwm);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane_controller");
    ros::NodeHandle nh;
    //cv::namedWindow("histogram");
    //cv::startWindowThread();

    // intialize the past time
    timeval t;
    gettimeofday(&t, NULL);
    past_time = (((long long) t.tv_sec) * 1000LL) + (((long long)t.tv_usec) / 1000LL);

    // subscribe to the "raw_image" topic
    ros::Subscriber pose_sub = nh.subscribe("pose_estimate", 1, PID);

    // publish the pwm of the motors
    pwm_pub = nh.advertise<std_msgs::Float64MultiArray>("motor_pwm", 1);

    ros::Rate r(30);
    ros::spin();
    //cv::destroyWindow("histogram");
}
