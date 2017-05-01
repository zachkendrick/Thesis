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
#include <queue> 

using namespace cv;
using namespace std;
using namespace ros;
using namespace tf2;

// number of bins in 2D histogram
const static float ANGLE_BINS = 60.0; 
const static float DISPLACE_BINS = 60.0;

// ideal position of the car
const static float CENTER_X = 315;
const static float CENTER_Y = -340;

// ideal position of the body of the car (around axels)
const static float BODY_Y = CENTER_Y - 300;

// ideal position of road lanes
const static float CENTER_LINE = 130;
const static float RIGHT_LINE = 530;

// the width of the road for horizontal displacement in rviz units
const static float ROAD_WIDTH = RIGHT_LINE - CENTER_LINE; //2*sqrt(pow((RIGHT_LINE - CENTER_LINE)/2, 2) + pow(BODY_Y - CENTER_Y, 2));;

// moving average window size
const static int AVG_WINDOW = 10;

// car pose publisher
ros::Publisher pose_pub;

// moving average queue
queue<float> displace_avg;

// previous pose estimate
Point2f prev_state;

// struct Point {
//     float x;
//     float y;
// }

struct Vector {
    float x_dir;
    float y_dir;
};

// functions
Mat histogram2D(const vector<geometry_msgs::Point_<std::allocator<void> > > points, const int id);
int lineSegmentAngle(float p1x, float p1y, float p2x, float p2y);
int lineSegmentDisplacement(float p1x, float p1y, float p2x, float p2y, const int id);
float angleWeight(float p1x, float p1y, float p2x, float p2y);
float movingAverage(float displace_bin);
Point2f getHistogramMode(Mat histogram);
Point2f getHistogramMean(Mat histogram);
float angleFromVecs(struct Vector v1, struct Vector v2);




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

void estimatePose(const visualization_msgs::MarkerConstPtr& msg) {
    
    // histograms of lane markings
    Mat hist;
    Mat histDisplay;

    // build a histogram of the angle/horizontal displacement
    hist = histogram2D(msg->points, msg->id);

    // find the mode (L-0 norm) of the histogram
    // Point2f angle_displacement = getHistogramMode(hist); //Point2f(0,0);

    // find the mean (L-1 norm) of the histogram
    Point2f angle_displacement = getHistogramMean(hist);
    // cout << angle_displacement.y << endl;
    angle_displacement.y = movingAverage(angle_displacement.y);


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
    car_pose.pose.position.x = angle_displacement.y;
    car_pose.pose.position.y = CENTER_Y; // negative
    car_pose.pose.position.z = 0.0;

    // set the angle of the car
    // cout << angle_displacement.x << endl;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, angle_displacement.x+M_PI/20.0);
    car_pose.pose.orientation.x = quat.x();
    car_pose.pose.orientation.y = quat.y();
    car_pose.pose.orientation.z = quat.z();
    car_pose.pose.orientation.w = quat.w();

    // set the dimensions of the arrow
    car_pose.scale.x = 100.0;
    car_pose.scale.y = 10.0;
    car_pose.scale.z = 10.0;

    // publish the car pose
    pose_pub.publish(car_pose);

    // display histogram
    //resize(hist, histDisplay, Size(300, 300), 0, 0, cv::INTER_NEAREST);
    //imshow("histogram", histDisplay);
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
    float angle_bin = 0.0;
    float displace_bin = 0.0;
    float weight = 0.0;

    // compute angle and displacement of line segments
    for(size_t i = 0; i < points.size(); i+=2) {
        angle_bin = lineSegmentAngle(points[i].x, points[i].y, points[i+1].x, points[i+1].y);
        displace_bin = lineSegmentDisplacement(points[i].x, points[i].y, points[i+1].x, points[i+1].y, id);
        weight = angleWeight(points[i].x, points[i].y, points[i+1].x, points[i+1].y);
        if(displace_bin == -1) {continue;}
        // if(abs(weight) > 80000) {cout << displace_bin << angle_bin << endl;}
        hist.at<float>(angle_bin, displace_bin) += weight;
    }

    return hist;
}

float movingAverage(float disp_val) {
    if(displace_avg.size() == AVG_WINDOW) {
        displace_avg.pop();
    }
    displace_avg.push(disp_val);

    float sum = 0;
    for(int i = 0; i < AVG_WINDOW; i++) {
        float tmp = displace_avg.front();
        displace_avg.pop();
        sum += tmp;
        displace_avg.push(tmp);
    }
    return sum/AVG_WINDOW;
}


float angleWeight(float p1x, float p1y, float p2x, float p2y) {

    float midPointX = (p1x + p2x)/2;
    float midPointY = (p1y + p2y)/2;
    float x = abs((CENTER_X - midPointX)/1000);
    float y = abs((CENTER_Y - midPointY)/1000);

    // euclidean distance
    // float dist = pow(x, 2) + pow(y, 2);
    float dist = x + pow(y, 2); //10

    // cost function
    float weight = exp(-dist);

    // cout << "distance: " << dist << endl;
    // cout << "weight: " << weight << endl;

    if(dist < 0.45) return weight;
    else return 0.0;              
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
    if(abs(p2x - p1x) < 1) {
    // cout << p2x - p1x << endl;
    }
    // cout << angle << endl;
    // map negative angle to turn to the left
    if(angle < 0) {angle = angle+(M_PI);}
    int angle_bin = int((angle/M_PI)*ANGLE_BINS);

    // check bounds
    if(angle_bin >= ANGLE_BINS) angle_bin = ANGLE_BINS-1;
    else if (angle_bin < 0) angle_bin = 0;

    return angle_bin;
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

float angleFromVecs(struct Vector v1, struct Vector v2) {
    float normP1 = sqrt(pow(v1.x_dir,2) + pow(v1.y_dir,2));
    float normP2 = sqrt(pow(v2.x_dir,2) + pow(v2.y_dir,2));

    // unit vectors
    v1.x_dir /= normP1;
    v1.y_dir /= normP1;
    v2.x_dir /= normP2;
    v2.y_dir /= normP2;

    float angle = acos(v1.x_dir*v2.x_dir + v1.y_dir*v2.y_dir);

    return angle;
}


int lineSegmentDisplacement(float p1x, float p1y, float p2x, float p2y, const int id) {

    // find the midpoint of the line segment along the horizontal displacement

    // float midPoint = (p1x + p2x)/2;

    // resulting displacment from center of the road
    float displacement = -1;

    // exclude lines far away from the camera
    if((p1y + p2y)/2 > 0) {return displacement;}

///////////////////////////////////////////////////////////////////////
// option 1
    
    // float px;
    // float py;

    // if(p1y <= p2y) {
    //     px = p1x;
    //     py = p1y;
    // }
    // else {
    //     px = p2x;
    //     py = p2y;
    // }

/////////////////////////////////////////////////////////////////////////
// option 2

    float px_low;
    float py_low;
    float px_high;
    float py_high;

    if(p1y <= p2y) {
        px_low = p1x; 
        py_low = p1y; 
        px_high = p2x;
        py_high = p2y;
    }
    else {
        px_low = p2x; 
        py_low = p2y; 
        px_high = p1x;
        py_high = p1y;
    }

///////////////////////////////////////////////////////////////////////////
// option 1
    // float slope = ((p1x-p2x)/(p1y-p2y));
    // midPoint += slope*(CENTER_Y - 75);
    // roadWidth = sqrt(pow(ROAD_WIDTH/2, 2) + pow(CENTER_Y-75, 2));

///////////////////////////////////////////////////////////////////////////
//option 2

    // distance to closest lane line point
    Vector v1;
    v1.x_dir = (CENTER_X-px_low);
    v1.y_dir = (BODY_Y-py_low);

    // lane line vector
    Vector v2;
    v2.x_dir = (px_low-px_high);
    v2.y_dir = (py_low-py_high);

    float angle = angleFromVecs(v1, v2);

    if(id == 3 && px_low > CENTER_X) {
        displacement = (ROAD_WIDTH/2) - sin(angle)*sqrt(pow(CENTER_X-px_low,2) + pow(BODY_Y-py_low,2));
    }
    // yellow lanes
    else if(id == 2 && px_low < CENTER_X && px_low > 0) {
        displacement = -((ROAD_WIDTH/2) - sin(angle)*sqrt(pow(CENTER_X-px_low,2) + pow(BODY_Y-py_low,2)));
    }
    // bad lane line detections should be ignored
    else {
        return displacement;
    }


///////////////////////////////////////////////////////////////////////////
// option 1
    // if(id == 3 && px > CENTER_X) {
    //     displacement = (ROAD_WIDTH/2) - sqrt(pow(CENTER_X-px,2) + pow(BODY_Y-py,2));
    // }
    // // yellow lanes
    // else if(id == 2 && px < CENTER_X && px > 0) {
    //     displacement = -((ROAD_WIDTH/2) - sqrt(pow(CENTER_X-px,2) + pow(BODY_Y-py,2)));
    // }
    // // bad lane line detections should be ignored
    // else {
    //     return displacement;
    // }


//////////////////////////////////////////////////////////////////////////////
// original

    // right white lanes
    // if(id == 3 && midPoint > CENTER_X) {
    //     displacement = (ROAD_WIDTH/2) - abs(CENTER_X - midPoint);
    // }
    // // yellow lanes
    // else if(id == 2 && midPoint < CENTER_X && midPoint > 0) {
    //     displacement = -((ROAD_WIDTH/2) - abs(CENTER_X - midPoint));
    // }
    // // bad lane line detections should be ignored
    // else {
    //     return displacement;
    // }

///////////////////////////////////////////////////////////////////////////

    // cout << ((displacement/(ROAD_WIDTH/2))+0.5) << endl;
    displacement = int(((displacement/(ROAD_WIDTH/2))+0.5)*DISPLACE_BINS);
    
    // displacement bins bound check
    if(displacement >= DISPLACE_BINS) displacement = DISPLACE_BINS-1;
    else if(displacement < 0) displacement = 0;
    // cout << displacement << endl;
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
    // cout << max_loc.y << endl;
    angle_displacement.x = (max_loc.y/ANGLE_BINS)*M_PI;
    // cout << max_loc.x << endl; 
    angle_displacement.y = CENTER_X + (((max_loc.x/DISPLACE_BINS) - 0.5)*(ROAD_WIDTH/2)); 

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

    if(mu.m00 == 0) {
        angle_displacement = prev_state;
        return angle_displacement;
    }

    int mean_y = int(mu.m10/mu.m00);
    int mean_x = int(mu.m01/mu.m00);

    // calculate angle and displacement at that location
    angle_displacement.x = (mean_x/ANGLE_BINS)*M_PI;
    angle_displacement.y = CENTER_X + (((mean_y/DISPLACE_BINS) - 0.5)*(ROAD_WIDTH/2));

    prev_state = angle_displacement;
    
    return angle_displacement;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_estimation");
    ros::NodeHandle nh;
    //cv::namedWindow("histogram");
    //cv::startWindowThread();

    // subscribe to the "raw_image" topic
    ros::Subscriber pose_sub = nh.subscribe("visualization_marker", 1, estimatePose);

    // publish the pose of the vehicle
    pose_pub = nh.advertise<visualization_msgs::Marker>("pose_estimate", 1);
    ros::Rate r(30);
    ros::spin();
    //cv::destroyWindow("histogram");
}
