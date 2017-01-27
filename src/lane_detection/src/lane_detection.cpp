/*************************************************
 *  lane_detectin.cpp
 *  
 *  This node subscribes to the raw_image topic. Using
 *  the raw image data, the node finds straight lines
 *  that make up the center dashed line that divides 
 *  the two lanes in yellow (red lines shown) and the
 *  outer white line of the lane (blue lines shown).
 *
 *  Subscribers: raw_image (video_capture.cpp)
 *  Publishers: lane_lines
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

using namespace cv;
using namespace std;

Mat cannyEdge(Mat frame);
void houghTransform(Mat edges, Mat frame, Scalar color);
Mat centerLaneMarkings(Mat edges, Mat frame); 



/*
   Function:
      Call back function that reads an image from the
      "raw_image" topic and finds lines using hough 
      transforms that make up the inner and outer
      markings of the lanes.
   Parameters:
      const sensor_msgs::ImageConstPtr& msg
   Publishes:
      TODO

*/

void imageLanePointsPublisher(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    Mat frame;
    Mat edges; 
    Mat centerMarkings;
    Mat outerMarkings;
    const int horizon = 150; // the pixel height of the horizon in the scene

    frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    frame = frame(Rect(0, horizon, frame.size().width, frame.size().height-horizon));
    edges = cannyEdge(frame);
    centerMarkings = centerLaneMarkings(edges, frame);
    bitwise_xor(centerMarkings, edges, outerMarkings);
    houghTransform(centerMarkings, frame, Scalar(0,0,255));
    houghTransform(outerMarkings, frame, Scalar(255,0,0));

    imshow("view", frame);
    imshow("center lane", centerMarkings);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}



/*
   Function:
      returns the canny edge detection of a color image
      in black and white.
   Parameters:
      Mat frame - BGR frame
   Returns:
      Mat edges - binary canny edge image 

*/

Mat cannyEdge(Mat frame) {

  Mat edges;

  // convert to grey scale, blur, canny edge
  cvtColor(frame, edges, COLOR_BGR2GRAY);
  GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
  Canny(edges, edges, 10, 30, 3);

  return edges;
}



/*
   Function: 
      Computes the hough transform of an image.
   
   Parameters:
      Mat edges - canny edge image
      Mat frame - original image
   Returns:
      void

*/

void houghTransform(Mat edges, Mat frame, Scalar color){

  vector<Vec4i> lines;
  HoughLinesP(edges, lines, 1, CV_PI/180, 50, 30, 10);
  for( size_t i = 0; i < lines.size(); i++)
  {
    Vec4i l = lines[i];
    line(frame, Point(l[0], l[1]), Point(l[2], l[3]), color, 3, CV_AA);
  }
    return;
}



/*
   Function:
      Finds the canny edges of only the yellow
      center lines by converting the image to
      HSV space.
   
   Parameters:
      Mat edges - canny edge image
      mat frame - original image
   Returns:
      Mat imCenterMarkings - the canny edges of the center markings

*/

Mat centerLaneMarkings(Mat edges, Mat frame) {

    Mat imHSV;
    Mat imLaneMarkingsMask;
    Mat imCenterMarkings;
    Mat imDilate;

    // convert to HSV, threshold, dilate, bitwise AND with original image
    cvtColor(frame, imHSV, CV_BGR2HSV);
    inRange(imHSV, Scalar(20, 230, 100), Scalar(50, 255, 150), imLaneMarkingsMask);
    dilate(imLaneMarkingsMask, imDilate, Mat(), Point(-1,-1), 3);
    bitwise_and(edges, imDilate, imCenterMarkings);

    return imCenterMarkings;

}



int main(int argc, char **argv)
{
  Mat frame;
  ros::init(argc, argv, "lane_detection");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);

  // subscribe to the "raw_image" topic
  image_transport::Subscriber sub = it.subscribe("raw_image", 1, imageLanePointsPublisher);

  // publish the "lane_lines"
  // image_transport::Publisher lane_lines_pub = it.advertise("lane_lines", 1);
  ros::spin();
  cv::destroyWindow("view");
}

