#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

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

    // Calculate pose of marker where rvecs is rotation and tvecs is translation
    // with respect to the camera lens
    std::vector<cv::Vec3d> rvecs, tvecs;

    cv::aruco::estimatePoseSingleMarkers(
        markerCorners, 8.0, cameraMatrix, distCoeffs, rvecs, tvecs); // Marker side length is 0.078 meters

    pcl::PointCloud<pcl::PointXYZ> positions;
    positions.width = 11; // Number of points
    positions.height = 1; // Unorganized point cloud
    positions.is_dense = false;
    positions.points.resize(positions.width * positions.height); 

    for (uint32_t i = 0; i < tvecs.size(); ++i){
        cv::Mat world_coord;
        cv::Mat camera_vec = (cv::Mat_<double>(4, 1) << tvecs[i][0], tvecs[i][1], tvecs[i][2], 1.0);
        cv::gemm(transform, camera_vec, 1.0, cv::Mat(), 0, world_coord);

        positions[markerIds[i]].x = static_cast<float>(world_coord.at<double>(0, 0));
        positions[markerIds[i]].y = static_cast<float>(world_coord.at<double>(1, 0));
        positions[markerIds[i]].z = static_cast<float>(world_coord.at<double>(2, 0));
    }

    sensor_msgs::PointCloud2 positions_msg;
    pcl::toROSMsg(positions, positions_msg);
    positions_msg.header.frame_id = "map";
    pub_.publish(positions_msg);
  }

  SubscribeAndPublish() {
    // Used to publish and subscribe to images
    image_transport::ImageTransport it(n_);

    // Topic you want to publish
    pub_ = n_.advertise<sensor_msgs::PointCloud2>("/positions", 10);

    cameraMatrix = cv::Mat::eye(3, 3, CV_32F);
    distCoeffs = cv::Mat::zeros(1, 5, CV_32F);


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
  cv::Mat transform = (cv::Mat_<double>(4, 4) << 
    1.0, 0.0, 0.0, -127.39999999999996,
    0.0, -1.0, 0.0, 79.59999999999997,
    0.0, 0.0, -1.0, 0.19999999999999993,
    0.0, 0.0, 0.0, 1.0);
  uint32_t numPoints = 1;

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