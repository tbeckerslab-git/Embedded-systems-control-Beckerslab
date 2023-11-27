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
        // Start timer
    double timer = (double)getTickCount();
        


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

  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  // cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters();
  const cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  // cv::aruco::ArucoDetector detector(dictionary, detectorParams);
  cv::aruco::detectMarkers(current_frame, dictionary, markerCorners, markerIds); //  cv::aruco::detectMarkers(current_frame, dictionary, markerCorners, markerIds); //

  // cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.5, current_frame);
  cv::aruco::drawDetectedMarkers(current_frame, markerCorners, markerIds);

  // Display frame for 30 milliseconds
  cv::waitKey(1);

  // Calculate Frames per second (FPS)
  float fps = getTickFrequency() / ((double)getTickCount() - timer);
      // Display FPS on frame
  putText(current_frame, "FPS : " + to_string(int(fps)), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);

  // Display the current frame
  cv::imshow("view",current_frame); 
 
}


int main(int argc, char **argv)
{
    //Initialize the aruco marker
    cv::Mat markerImage;
    const cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::drawMarker(dictionary, 22, 200, markerImage, 1);
    cv::imshow("marker", markerImage);
    
    // The name of the node
    ros::init(argc, argv, "frame_listener");
    
    // Default handler for nodes in ROS
    ros::NodeHandle nh;
    
    // Used to publish and subscribe to images
    image_transport::ImageTransport it(nh); 
    // Subscribe to the /camera topic
    image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1, imageCallback);

    // Make sure we keep reading new video frames by calling the imageCallback function
    ros::spin();
    
    // Close down OpenCV
    cv::destroyWindow("view");

}