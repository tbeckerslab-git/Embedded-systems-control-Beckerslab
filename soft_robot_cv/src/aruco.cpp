#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class SubscribeAndPublish {
public:
  void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat current_frame;

    try {

      // Convert the ROS message
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

      // Store the values of the OpenCV-compatible image
      // into the current_frame variable
      current_frame = cv_ptr->image;

    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                msg->encoding.c_str());
    }

    // Detect the marker
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    const cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    // cv::Mat cropped_frame(
    //     current_frame, Rect(0, 0, 600, 680)); // Width 1440, Height 1080. FPS
                                                // starts to drop off at 900X900
    cv::aruco::detectMarkers(current_frame, dictionary, markerCorners,
                             markerIds);
    cv::aruco::drawDetectedMarkers(current_frame, markerCorners, markerIds);

    cv::imshow("view", current_frame);
    cv::waitKey(1);

    // // Calculate pose of marker where rvecs is rotation and tvecs is translation
    // // with respect to the camera lens
    // std::vector<cv::Vec3d> rvecs, tvecs;
    // cv::Mat objPoints(4, 1, CV_32FC3);

    // cv::aruco::estimatePoseSingleMarkers(
    //     markerCorners, 0.078, cameraMatrix, distCoeffs, rvecs, tvecs,
    //     objPoints); // Marker side length is 0.078 meters

    // // Create and publish position message
    // geometry_msgs::Point position;

    // for (auto elem : tvecs) {
    //   position.x = elem[0];
    //   position.y = elem[1];
    //   position.z = elem[2];
    // }

    // pub_.publish(position);
  }

  SubscribeAndPublish() {
    // Used to publish and subscribe to images
    image_transport::ImageTransport it(n_);

    // Topic you want to publish
    pub_ = n_.advertise<geometry_msgs::Point>("/position", 1);

    cameraMatrix = cv::Mat::eye(3, 3, CV_32F);
    distCoeffs = cv::Mat::eye(3, 3, CV_32F);


    // Topic you want to subscribe
    // FIXME: n_.subscribe should be it.subscribe. Figure out why it'ss throwing
    // an error
    sub_ = n_.subscribe("/camera/image_raw", 1,
                        &SubscribeAndPublish::imageCallback, this);

    
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  std::vector<float> data;
  cv::Mat cameraMatrix;
  std::vector<float> data2;
  cv::Mat distCoeffs;


}; // End of class SubscribeAndPublish

int main(int argc, char **argv) {
  // Initialize the aruco marker
  cv::Mat markerImage;
  const cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  cv::aruco::drawMarker(dictionary, 10, 200, markerImage, 1);
  // cv::imshow("marker", markerImage);

  // The name of the node
  ros::init(argc, argv, "aruco_handler");

  // Create an object of class SubscribeAndPublish that will take care of
  // everything
  SubscribeAndPublish SAPObject;

  // Make sure we keep reading new video frames by calling the imageCallback
  // function
  ros::spin();
}