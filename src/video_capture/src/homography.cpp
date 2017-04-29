// ---------------------------------------------------------------------------
/* This code illustrates the capabilities of opencv related to bird eye view
 * homography. 
 * 
 * The implementation extends the code presented by jmartel on
 * http://stackoverflow.com/questions/6606891/opencv-virtually-camera-rotating-translating-for-birds-eye-view
 * 
 * Take an additional view on
 * https://en.wikipedia.org/wiki/Homography
*/
 
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
 
#define PI 3.14

// define the cuttoff for the horizon of the image
int horizon = 180;
 
using namespace cv;
using namespace std;
 
int main( int argc, char** argv )
{
  // read the camera input
  VideoCapture cap(0); 
  if(!cap.isOpened())  
      return -1;
 
  // default values of the homography parameters
  int alpha_=90., beta_=90., gamma_=90.;
  int f_ = 500, dist_ = 500;
   
  // images
  Mat destination;
  Mat source;
  Mat big_source;
   
  /// Create Window
  namedWindow("Result", 1); 
 
  /// Add trackbars for different aspects of the 
  createTrackbar("Alpha", "Result", &alpha_, 180);
  createTrackbar("Beta", "Result", &beta_, 180);
  createTrackbar("Gamma", "Result", &gamma_, 180);
  createTrackbar("f", "Result", &f_, 2000);
  createTrackbar("Distance", "Result", &dist_, 2000);
 
  while(true) {
       
      //grab and retrieve each frames of the video sequentially 
      cap >> big_source;
      resize(big_source, source, Size(640,480), cv::INTER_NEAREST);
      source = source(Rect(0, horizon, source.size().width, source.size().height-horizon));
       
      double f, dist;
      double alpha, beta, gamma;
      alpha = ((double)alpha_ - 90.)*PI/180;
      beta = ((double)beta_ - 90.)*PI/180;
      gamma = ((double)gamma_ - 90.)*PI/180;
      f = (double) f_;
      dist = (double) dist_;
 
      Size taille = source.size();
      double w = (double)taille.width, h = (double)taille.height;
 
      // Projection 2D -> 3D matrix
      Mat A1 = (Mat_<float>(4,3) <<
          1, 0, -w/2,
          0, 1, -h/2,
          0, 0,    0,
          0, 0,    1);
 
      // Rotation matrices around the X,Y,Z axis
      Mat RX = (Mat_<float>(4, 4) <<
          1,          0,           0, 0,
          0, cos(alpha), -sin(alpha), 0,
          0, sin(alpha),  cos(alpha), 0,
          0,          0,           0, 1);
 
      Mat RY = (Mat_<float>(4, 4) <<
          cos(beta), 0, -sin(beta), 0,
                  0, 1,          0, 0,
          sin(beta), 0,  cos(beta), 0,
                  0, 0,          0, 1);
 
      Mat RZ = (Mat_<float>(4, 4) <<
          cos(gamma), -sin(gamma), 0, 0,
          sin(gamma),  cos(gamma), 0, 0,
          0,          0,           1, 0,
          0,          0,           0, 1);
 
      // Composed rotation matrix with (RX,RY,RZ)
      Mat R = RX * RY * RZ;
      // cout << R << endl;
 
      // Translation matrix on the Z axis change dist will change the height
      Mat T = (Mat_<float>(4, 4) <<           1, 0, 0, 0,           0, 1, 0, 0,           0, 0, 1, dist,           0, 0, 0, 1);       // Camera Intrisecs matrix 3D -> 2D
      Mat A2 = (Mat_<float>(3,4) <<
          f, 0, w/2, 0,
          0, f, h/2, 0,
          0, 0,   1, 0);

      // cout << A2 << endl;
 
      // Final and overall transformation matrix
      Mat transfo = A2 * (T * (R * A1));
      cout << transfo << endl;
 
      // Apply matrix transformation
      warpPerspective(source, destination, transfo, taille, INTER_CUBIC | WARP_INVERSE_MAP);
 
      // Output
      imshow("Result", destination);
       
      waitKey(30);
  }
  return 0;
}
