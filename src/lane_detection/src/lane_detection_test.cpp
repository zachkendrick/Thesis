/*************************************************
 *  lane_detectin.cpp
 *  
 *  This node subscribes to the raw_image topic. Using
 *  the raw image data, the node finds straight lines
 *  that make up the center dashed line that divides 
 *  the two lanes in yellow (red lines shown) and the
 *  outer white line of the lane (blue lines shown).
 *  The lines are projected from the image plane onto
 *  on to the ground plane using a homography matrix.
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
#include <visualization_msgs/Marker.h>
#include <image_geometry/pinhole_camera_model.h>

using namespace cv;
using namespace std;

Mat cannyEdge(Mat frame);
vector<Vec4i> houghTransform(Mat edges, Mat frame, Scalar color);
Mat centerLaneMarkings(Mat edges, Mat frame); 
visualization_msgs::Marker groundProjection(vector<Vec4i> lines, visualization_msgs::Marker line_list, const float color[]);

// lane lines publisher
ros::Publisher lane_lines_pub;

// colors
const static float YELLOW[4] = {1.0, 1.0, 0.0, 1.0};
const static float WHITE[4] = {1.0, 1.0, 1.0, 1.0};

// the pixel height of the horizon in the scene
const int horizon = 220; 

// homography matrix
Matx33f H(500, -279.79456, 41969.18,
	  0, 111.48711, 58276.93,
	  0, -0.87435794, 631.15369);

/*
   Function:
      Call back function that reads an image from the
      "raw_image" topic and finds lines using hough 
      transforms that make up the inner and outer
      markings of the lanes.
   Parameters:
      const sensor_msgs::ImageConstPtr& msg
   Publishes:
      Line list with id=2 (white lanes) and id=3 (yellow lanes)
*/

void imageLanePoints(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    Mat frame;
    Mat edges; 
    Mat centerMarkings;
    Mat outerMarkings;

    // find line segments that define lane markings
    frame = cv_bridge::toCvShare(msg, "bgr8")->image;

    // publisher information of line list white lanes
    visualization_msgs::Marker line_list_inner;
    line_list_inner.header.frame_id = "/my_frame";
    line_list_inner.ns = "points_and_lines";
    line_list_inner.action = visualization_msgs::Marker::ADD;
    line_list_inner.id = 2;
    line_list_inner.type = visualization_msgs::Marker::LINE_LIST;
    line_list_inner.scale.x = 5.0;//0.01;

    // publisher information of line list yellow lanes
    visualization_msgs::Marker line_list_outer;
    line_list_outer.header.frame_id = "/my_frame";
    line_list_outer.ns = "points_and_lines";
    line_list_outer.action = visualization_msgs::Marker::ADD;
    line_list_outer.id = 3;
    line_list_outer.type = visualization_msgs::Marker::LINE_LIST;
    line_list_outer.scale.x = 5.0;//0.01;

    // Uncomment to add radial undistortion
    Mat_<float> cam(3,3); cam << 309.58086449, 0.0, 332.29985864,  0.0, 309.62831779, 240.61274041, 0.0, 0.0, 1.0;
    Mat_<float> dist(1,5); dist << -3.52082519e-01, 1.59330550e-01, 4.93449598e-04, -1.77065551e-04, 0.0;
    //Mat white = 255*Mat::ones(frame.size().height, frame.size().width, CV_8U);
    //imshow("mask", white);
    
    // Mat frame;
    //Mat optimalCam;
    //optimalCam = getOptimalNewCameraMatrix(cam, dist, frame.size(), 0);
    //undistort(frame, frame_undistort, cam, dist, optimalCam);
    //undistort(white, mask_undistort, cam, dist, optimalCam);
    //frame = frame_undistort;
    
    //erode(mask_undistort, mask_undistort, Mat(), Point(-1,-1), 2);
    //imshow("test", frame);
    //imshow("mask", mask_undistort);

    // perspective transform
    // warpPerspective(frame, destination, transfo, taille, INTER_CUBIC | WARP_INVERSE_MAP);

    // find the center lane markings and outter lane markings
    frame = frame(Rect(0, horizon, frame.size().width, frame.size().height-horizon));
    //mask_undistort = mask_undistort(Rect(0, horizon, mask_undistort.size().width, mask_undistort.size().height-horizon));
    edges = cannyEdge(frame);
    //bitwise_and(edges, mask_undistort, edges);
    centerMarkings = centerLaneMarkings(edges, frame);
    bitwise_xor(centerMarkings, edges, outerMarkings);
    vector<Vec4i> lines_inner = houghTransform(centerMarkings, frame, Scalar(0,0,255));
    line_list_inner = groundProjection(lines_inner, line_list_inner, YELLOW);
    vector<Vec4i> lines_outer = houghTransform(outerMarkings, frame, Scalar(255,0,0));
    line_list_outer = groundProjection(lines_outer, line_list_outer, WHITE);

    // publish the lane markings
    lane_lines_pub.publish(line_list_inner);
    lane_lines_pub.publish(line_list_outer);

    // display window
    // imshow("view", frame);
    // imshow("center lane", centerMarkings);
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
  Canny(edges, edges, 100, 150, 3); // 10, 30

  return edges;
}



/*
   Function: 
      Computes the hough transform of an image.
   Parameters:
      Mat edges - canny edge image
      Mat frame - original image
   Returns:
      A vector containing the line segments found
      by the hough transform. Each line segment is
      a vector of length four in the order x1,y1,x2,y2
*/

vector<Vec4i> houghTransform(Mat edges, Mat frame, Scalar color){

  vector<Vec4i> lines;
  HoughLinesP(edges, lines, 1, CV_PI/180, 50, 30, 10);
  for(size_t i = 0; i < lines.size(); i++) {
    Vec4i l = lines[i];
    line(frame, Point(l[0], l[1]), Point(l[2], l[3]), color, 3, CV_AA);
  }
  return lines;
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
  // imshow("HSV", imHSV);
  inRange(imHSV, Scalar(10, 100, 100), Scalar(65, 255, 200), imLaneMarkingsMask);
  dilate(imLaneMarkingsMask, imDilate, Mat(), Point(-1,-1), 3);
  bitwise_and(edges, imDilate, imCenterMarkings);

  return imCenterMarkings;

}



/*
   Function:
      Uses homography to project points in the image
      plane into the ground plane.
   Parameters:
      vector<Vec4i> lines - a vector of line segments
      visualization_msgs::Marker line_list - the line list publisher marker
      const float color[] - the color of the line segments (r,g,b,a)
   Returns:
      The visualization_msgs with the ground projected line segments
*/

visualization_msgs::Marker groundProjection(vector<Vec4i> lines, visualization_msgs::Marker line_list, const float color[]) {
  
  // Line list is red
  line_list.color.r = color[0];
  line_list.color.g = color[1];
  line_list.color.b = color[2];
  line_list.color.a = color[3];

  for(size_t i = 0; i < lines.size(); i++)
  {
    vector<Point2f> dstPoints;
    vector<Point2f> srcPoints;
    Vec4i l = lines[i];
    srcPoints.push_back(Point2f(l[0], l[1]));
    srcPoints.push_back(Point2f(l[2], l[3]));

    perspectiveTransform(srcPoints, dstPoints,H.inv());

    geometry_msgs::Point p1;
    geometry_msgs::Point p2;

    // note the flip in axis
    p1.z = 0.0;
    p1.y = -dstPoints[0].y;
    p1.x = dstPoints[0].x;
    p2.z = 0.0;
    p2.y = -dstPoints[1].y;
    p2.x = dstPoints[1].x;
    
    // The line list needs two points for each line
    line_list.points.push_back(p1);
    line_list.points.push_back(p2);
  }

  return line_list;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "lane_detection");
  ros::NodeHandle nh;
  //cv::namedWindow("view");
  //cv::startWindowThread();
  image_transport::ImageTransport it(nh);

  // subscribe to the "raw_image" topic
  image_transport::Subscriber sub = it.subscribe("raw_image", 1, imageLanePoints);

  // publish the "lane_lines"
  lane_lines_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Rate r(30);
  ros::spin();
  //cv::destroyWindow("view");
}

