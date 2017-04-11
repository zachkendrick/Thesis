/*************************************************
 *  video_capture.cpp
 *  
 *  Reads in frames from a camera and publishes them
 *  down sampled to 640x480.
 *
 *  Subscribers: N/A
 *  Publishers: raw_image
 *
 *  Author: Zachary Kendrick
 ************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    // create ros node and publisher for image
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher raw_image_pub = it.advertise("raw_image", 1);

    // change camera input
    VideoCapture cap("small_turn.mp4");
    if(!cap.isOpened()) 
        return -1;
    // Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(30);

    //namedWindow("edges",1);
    while(nh.ok())
    {
        Mat frame;
        cap >> frame;

        if(!frame.empty())
        {
        Mat frameSmall;

        //down sample image to 640x480
        resize(frame, frameSmall, Size(640,480), cv::INTER_NEAREST);

        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frameSmall).toImageMsg();
        raw_image_pub.publish(msg);
        imshow("frame", frameSmall);
        if(waitKey(1) >= 0) break;
        }

        ros::spinOnce();
        loop_rate.sleep();

    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}