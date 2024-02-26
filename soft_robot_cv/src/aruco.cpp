#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace cv;
using namespace std;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  //Initialize camera and distortion parameters
  float data[9] = { 1.05326155e+03, 0.00000000e+00, 6.72839741e+02, 0.00000000e+00, 1.05471479e+03, 3.55441462e+02, 0, 0, 1.00000000e+00};
  cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32F, data);

  float data2[5] = { 0.09004139, -0.29306343, -0.00608111, 0.00926377, 0.2020092};
  cv::Mat distCoeffs = cv::Mat(1, 5, CV_32F, data2);
 
  // Pointer used for the conversion from a ROS message to 
  // an OpenCV-compatible image
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat current_frame;
   
  try
  { 
   
    // Convert the ROS message  
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
     
    // Store the values of the OpenCV-compatible image
    // into the current_frame variable
    current_frame = cv_ptr->image;
    

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  //Detect the marker
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  const cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  cv::aruco::detectMarkers(current_frame, dictionary, markerCorners, markerIds); //  cv::aruco::detectMarkers(current_frame, dictionary, markerCorners, markerIds); //
  cv::aruco::drawDetectedMarkers(current_frame, markerCorners, markerIds); 

  //Calculate pose of marker where rvecs is rotation and tvecs is translation with respect to the camera lens
  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::Mat objPoints(4, 1, CV_32FC3);
  cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.078, cameraMatrix, distCoeffs, rvecs, tvecs, objPoints); //Marker side length is 0.078 meters
  
  // cv::drawFrameAxes(current_frame, cameraMatrix, distCoeffs, rvecs, tvecs, 0.1);
  // cout << objPoints << endl;
  // std::vector<cv::Vec3d> rvecs, tvecs;

  // // Set coordinate system
  // cv::Mat objPoints(4, 1, CV_32FC3);
  // cv::aruco::drawDetectedMarkers(current_frame, markerCorners, markerIds); 
  // solvePnP(objPoints, markerCorners.at(0), cameraMatrix, distCoeffs, rvecs.at(0), tvecs.at(0));
  // // for (int i = 0; i < rvecs.size(); ++i) {
  // //   auto rvec = rvecs[i];
  // //   auto tvec = tvecs[i];
  // //   cv::drawFrameAxes(current_frame, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
  // // }

  // Display the current frame
  cv::imshow("view",current_frame); 
    
  // Display frame for 30 milliseconds
  cv::waitKey(30);
 
}


int main(int argc, char **argv)
{
    //Initialize the aruco marker
    cv::Mat markerImage;
    const cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::drawMarker(dictionary, 11, 200, markerImage, 1);
    cv::imshow("marker1", markerImage);
    cv::aruco::drawMarker(dictionary, 12, 200, markerImage, 1);
    cv::imshow("marker2", markerImage);
    cv::aruco::drawMarker(dictionary, 13, 200, markerImage, 1);
    cv::imshow("marker3", markerImage);
    cv::aruco::drawMarker(dictionary, 14, 200, markerImage, 1);
    cv::imshow("marker4", markerImage);
    
    // The name of the node
    ros::init(argc, argv, "frame_listener");
    
    // Default handler for nodes in ROS
    ros::NodeHandle nh;
    
    // Used to publish and subscribe to images
    image_transport::ImageTransport it(nh); 
    // Subscribe to the /camera topic
    image_transport::Subscriber sub = it.subscribe("camera", 1, imageCallback);

    // Make sure we keep reading new video frames by calling the imageCallback function
    ros::spin();
    
    // Close down OpenCV
    cv::destroyWindow("view");

}