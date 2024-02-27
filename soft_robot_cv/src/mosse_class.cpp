#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

#include <opencv2/core/ocl.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

using namespace cv;
using namespace std;

class Sub_Class {
public:
  Ptr<Tracker> tracker = TrackerMOSSE::create();

  void processFrame(cv::Mat frame) {

    // Define initial bounding box
    Rect2d bbox(287, 23, 86, 320);

    // Uncomment the line below to select a different bounding box
    // bbox = selectROI(frame, false);
    // Display bounding box.
    rectangle(frame, bbox, Scalar(255, 0, 0), 2, 1);

    // Start timer
    double timer = (double)getTickCount();

    // Update the tracking result
    bool ok = tracker->update(frame, bbox);

    imshow("Tracking", frame);
    tracker->init(frame, bbox);

    if (ok) {
      // Tracking success : Draw the tracked object
      rectangle(frame, bbox, Scalar(255, 0, 0), 2, 1);
    } else {
      // Tracking failure detected.
      putText(frame, "Tracking failure detected", Point(100, 80),
              FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);
    }
    // Calculate Frames per second (FPS)
    float fps = getTickFrequency() / ((double)getTickCount() - timer);

    // Display FPS on frame
    putText(frame, "FPS : " + to_string(int(fps)), Point(100, 50),
            FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);

    // Display frame.
    imshow("Tracking", frame);

    // Exit if ESC pressed.
    // int k = waitKey(1);
    // if(k == 27)
    // {
    //     break;
    // }
  }

  void imageCallback(const sensor_msgs::ImageConstPtr &msg) {

    // Pointer used for the conversion from a ROS message to
    // an OpenCV-compatible image
    cv_bridge::CvImagePtr cv_ptr;

    try {

      // Convert the ROS message
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

      // Store the values of the OpenCV-compatible image
      // into the current_frame variable
      cv::Mat current_frame = cv_ptr->image;

      // Display the current frame
      cv::imshow("view", current_frame);

      // Display frame for 30 milliseconds
      cv::waitKey(10);

      // Process the frame
      processFrame(current_frame);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                msg->encoding.c_str());
    }
  }
};

int main(int argc, char **argv) {
  // The name of the node
  ros::init(argc, argv, "frame_listener");

  // Default handler for nodes in ROS
  ros::NodeHandle nh;

  // Used to publish and subscribe to images
  image_transport::ImageTransport it(nh);

  Sub_Class listener;
  // Initalize the tracker
  //   Ptr<Tracker> tracker = TrackerMOSSE::create(); //Shouldn't need this line

  // Define initial bounding box
  Rect2d bbox(287, 23, 86, 320);

  // Uncomment the line below to select a different bounding box
  // bbox = selectROI(frame, false);
  // Display bounding box.

  // FIXME: Define init frame as default image
  /**
  rectangle(init_frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );

  imshow("Tracking", init_frame);
  tracker->init(init_frame, bbox);
  **/

  // Subscribe to the /camera topic

  image_transport::Subscriber sub =
      it.subscribe("camera/image_raw", 1, &Sub_Class::imageCallback, &listener);

  // Make sure we keep reading new video frames by calling the imageCallback
  // function
  ros::spin();

  // Close down OpenCV
  cv::destroyWindow("view");
}
