/*************************************************
 *  pose_estimation.cpp
 *  
 *  This node subscribes to the 'visualization_marker'
 *  topic that reads in road line segments. It uses
 *  the segments to generate a 2D histogram of 
 *  lateral position vs angle. The mode of the
 *  histogram is the estimated pose of the vehicle
 *  which is published on the 'pose_estimate" topic.
 *
 *  Subscribers: visualization_marker (lane_detection_test.cpp)
 *  Publishers: pose_estimate
 *
 *  Author: Zachary Kendrick
 ************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <visualization_msgs/Marker.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <math.h>

using namespace cv;
using namespace std;
using namespace ros;
using namespace tf2;

// number of bins in 2D histogram
const static int ANGLE_BINS = 60; 
const static int DISPLACE_BINS = 60;

// the width of the road for horizontal displacement in rviz units
const static float ROAD_WIDTH = 3.0;

// car pose publisher
ros::Publisher pose_pub;

// functions
Mat histogram2D(const vector<geometry_msgs::Point_<std::allocator<void> > > points, const int id);
int lineSegmentAngle(float p1x, float p1y, float p2x, float p2y);
int lineSegmentDisplacement(float p1x, float p1y, float p2x, float p2y, const int id);
Point2f getHistogramMode(Mat histogram);
Point2f getHistogramMean(Mat histogram);



/*
   Function:
      Call back function that reads a line list from
      "visualizer_marker" topic and estimates a pose
      for the vehicle.
   Parameters:
      const visualization_msgs::MarkerConstPtr& msg - "visualizer_marker" message
   Publishes:
      Pose with id=4
*/

void estimatePose(const visualization_msgs::MarkerConstPtr& msg)
{
    // histograms of lane markings
    Mat hist;
    Mat histDisplay;

    // build a histogram of the angle/horizontal displacement
    hist = histogram2D(msg->points, msg->id);

    // find the mode (L-0 norm) of the histogram
    Point2f angle_displacement = getHistogramMode(hist);

    // find the mean (L-1 norm) of the histogram
    angle_displacement = getHistogramMean(hist);

    // publish car pose
    visualization_msgs::Marker car_pose;
    car_pose.header.frame_id = "/my_frame";
    car_pose.ns = "car_pose";
    car_pose.action = visualization_msgs::Marker::ADD;
    car_pose.id = 4;
    car_pose.type = visualization_msgs::Marker::ARROW;

    // car pose arrow in green
    car_pose.color.r = 0.0;
    car_pose.color.g = 1.0;
    car_pose.color.b = 0.0;
    car_pose.color.a = 1.0;

    // set the horizontal displacement of the car
    car_pose.pose.position.x = 0.0;
    car_pose.pose.position.y = -angle_displacement.y;
    car_pose.pose.position.z = 0.0;

    // set the angle of the car
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0,  M_PI + angle_displacement.x);
    car_pose.pose.orientation.x = quat.x();
    car_pose.pose.orientation.y = quat.y();
    car_pose.pose.orientation.z = quat.z();
    car_pose.pose.orientation.w = quat.w();

    // set the dimensions of the arrow
    car_pose.scale.x = 1.0;
    car_pose.scale.y = 0.1;
    car_pose.scale.z = 0.1;

    // publish the car pose
    pose_pub.publish(car_pose);

    if(angle_displacement.x < 60 && angle_displacement.y < 60) {
    hist.at<float>(angle_displacement.x, angle_displacement.y) = 0.5;
    }

    // display histogram
    resize(hist, histDisplay, Size(300, 300), 0, 0, cv::INTER_NEAREST);
    imshow("histogram", histDisplay);
    cv::waitKey(1);
}



/*
   Function:
      Computes a 2D histogram of the car's pose given the lane line
      points for a given frame. The histogram has angle on the veritcle
      exis and displacement on the horizontal axis. 
   Parameters:
      const visualization_msgs::MarkerConstPtr& msg - pointer containing lane markings message
   Returns:
      Mat - A 2D histogram of size ANGLE_BINSxDISPLACE_BINS
*/

Mat histogram2D(const vector<geometry_msgs::Point_<std::allocator<void> > > points, const int id) {//const vector<geometry_msgs::Point_<std::allocator<void> > > points, const vector<geometry_msgs::id_<std::allocator<void> > > id) {

    // 2D histogram of horizontal displacement and angle of car
    Mat hist = Mat(ANGLE_BINS, DISPLACE_BINS, CV_32F, double(0));

    // angle and displacement values
    float angle = 0.0;
    float displace = 0.0;

    // compute angle and displacement of line segments
    for(size_t i = 0; i < points.size(); i+=2) {
        angle = lineSegmentAngle(points[i].x, points[i].y, points[i+1].x, points[i+1].y);
        displace = lineSegmentDisplacement(points[i].x, points[i].y, points[i+1].x, points[i+1].y, id);
        hist.at<float>(angle , displace)++;
    }

    return hist;
}



/*
   Function:
      Computes the angle of a line segment in radians given
      the cartesian coordinates of the end points that
      define the line segment and returns the histogram bin.
   Parameters:
      float p1x - x value of the first point
      float p1y - y value of the first point
      float p2x - x value of the second point
      float p2y - y value of the second point
   Returns:
      float - histogram for the angle
*/

int lineSegmentAngle(float p1x, float p1y, float p2x, float p2y) {

    float angle = atan((p2y - p1y)/(p2x - p1x));
    return int(((angle/M_PI)*ANGLE_BINS) + ANGLE_BINS/2);

}



/*
   Function:
      Computes the horizontal displacement from the road center
      of a line segment given the cartesian coordinates of the 
      end points that define the line segment.
   Parameters:
      float p1x - x value of the first point
      float p1y - y value of the first point
      float p2x - x value of the second point
      float p2y - y value of the second point
      const int id - the message id of the lane lines, id=2 (white lanes) and id=3 (yellow lanes)
      const float ROAD_WIDTH - the width of a lane in a roadway in rviz units
   Returns:
      float - horizontal displacement
*/

int lineSegmentDisplacement(float p1x, float p1y, float p2x, float p2y, const int id) {

    // find the midpoint of the line segment along the horizontal displacement
    float midpoint = (p1y + p2y)/2;

    // resulting displacment from center of the road
    float displacement = 0.0;

    // white lanes
    if(id == 2 && midpoint >= 0) {
        displacement = -ROAD_WIDTH/2 + midpoint;
    }

    // yellow lanes
    else if(id == 3 && midpoint <= 0) {
        displacement = ROAD_WIDTH/2 - midpoint;
    }

    displacement = int(((displacement/ROAD_WIDTH)*DISPLACE_BINS) + DISPLACE_BINS/2);

    return displacement; 
}



/*
   Function:
      Finds the mode of a 2D histogram. Returns
      a point with the angle and displacement of the mode.
   Parameters:
      Mat histogram - 2D histogram of angle and displacement
   Returns:
      Point2f - point containing angle and
      displacement of mode.
*/

Point2f getHistogramMode(Mat histogram) {
    
    double min, max;
    Point min_loc, max_loc;
    Point2f angle_displacement;

    // find mode location in histogram (x-displacement, y-angle)
    minMaxLoc(histogram, &min, &max, &min_loc, &max_loc);

    // calculate angle and displacement at that location
    angle_displacement.x = ((max_loc.y - ANGLE_BINS/2)*M_PI)/ANGLE_BINS; 
    angle_displacement.y = ((max_loc.x - DISPLACE_BINS/2)*ROAD_WIDTH)/DISPLACE_BINS; 

    return angle_displacement;
}



/*
   Function:
      Finds the mean of a 2D histogram. Returns
      a point with the angle and displacement of the mode.
   Parameters:
      Mat histogram - 2D histogram of angle and displacement
   Returns:
      Point2f - point containing angle and
      displacement of mean.
*/

Point2f getHistogramMean(Mat histogram) {

    Point2f angle_displacement;

    // find 1st order moments
    Moments mu = moments(histogram, false);
    int mean_y = int(mu.m10/mu.m00);
    int mean_x = int(mu.m01/mu.m00);

    // calculate angle and displacement at that location
    angle_displacement.x = ((mean_x - ANGLE_BINS/2)*M_PI)/ANGLE_BINS; 
    angle_displacement.y = ((mean_y - DISPLACE_BINS/2)*ROAD_WIDTH)/DISPLACE_BINS;

    return angle_displacement;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_estimation");
    ros::NodeHandle nh;
    cv::namedWindow("histogram");
    cv::startWindowThread();

    // subscribe to the "raw_image" topic
    ros::Subscriber pose_sub = nh.subscribe("visualization_marker", 1, estimatePose);

    // publish the pose of the vehicle
    pose_pub = nh.advertise<visualization_msgs::Marker>("pose_estimate", 1);
    ros::Rate r(30);
    ros::spin();
    cv::destroyWindow("histogram");
}