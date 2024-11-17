#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

/**
 * This class handles all logic for reading from a camera, detecting aruco markers, and publishing
 * the position of the detected markers
 */
class SubscribeAndPublish {
public:
  /**
   * Fuynction called every time the subscriber recognizes a new image
   */
  void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    // Pointer used for the conversion from a ROS message to
    // an OpenCV-compatible image
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

    // Detect any markers
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    const cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Mat cropped_frame(
        current_frame, Rect(0, 0, 1440, 1080)); // Width 1440, Height 1080. FPS
                                                // starts to drop off at 900X900
    cv::aruco::detectMarkers(cropped_frame, dictionary, markerCorners,
                             markerIds);
    cv::aruco::drawDetectedMarkers(current_frame, markerCorners, markerIds);
    cv::imshow("view", cropped_frame);
    cv::waitKey(1);

    // Calculate pose of marker where rvecs is rotation and tvecs is translation
    // with respect to the camera lens

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

    // // Create publisher to publish to a default topic names "/postion"
    pub_ = n_.advertise<geometry_msgs::Point>("/position", 1);

    //Check the actual name of the topic / get the name of the topic after remapping
    std::string topicName = pub_.getTopic(); 
    std::string cameraIndex = topicName.substr(9, 1);

    //Load in camera config data
    // n_.getParam("/camera" + cameraIndex + "_matrix/data", data);
    // float* tmp;
    // tmp = data.data();
    // cameraMatrix = cv::Mat(3, 3, CV_32F, tmp);

    // n_.getParam("/distortion" + cameraIndex + "_coefficients/data", data2);
    // float* tmp2;
    // tmp2 = data2.data();
    // distCoeffs = cv::Mat(3, 3, CV_32F, tmp2);

    // std::cout << cameraMatrix << std::endl;
    // std::cout << distCoeffs << std::endl;


    // Topic you want to subscribe
    // FIXME: n_.subscribe should be it.subscribe. Figure out why it's throwing
    // an error
    sub_ = n_.subscribe("/camera" + cameraIndex + "/image_raw", 1,
                        &SubscribeAndPublish::imageCallback, this);

    
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_; //Publishes position data
  ros::Subscriber sub_; //Subscribes to image feed
  std::vector<float> data; //intermediary vector for loading cameraMatrix data from file
  cv::Mat cameraMatrix; //3x3 matrix
  std::vector<float> data2; //intermediary vector for loading distCoeffs data from file
  cv::Mat distCoeffs; //1X5 vector


}; // End of class SubscribeAndPublish

int main(int argc, char **argv) {
  // // Initialize the aruco marker
  cv::Mat markerImage;

  // The name of the node
  ros::init(argc, argv, "aruco_handler");

  // Create an object of class SubscribeAndPublish
  SubscribeAndPublish SAPObject;

  // Make sure we keep reading new video frames by repeatedly calling the imageCallback function
  ros::spin();
}