#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()
latest_image = None

def image_callback(msg):
    global latest_image
    latest_image = bridge.imgmsg_to_cv2(msg, "bgr8")


def get_centerline_angle(points):
    """Fit a line using PCA and return angle in degrees."""
    data = np.array(points, dtype=np.float32)

    mean = np.mean(data, axis=0)
    data_centered = data - mean

    _, _, vt = np.linalg.svd(data_centered)

    direction = vt[0]

    angle = np.arctan2(direction[1], direction[0])
    return np.degrees(angle), mean, direction


if __name__ == "__main__":

    rospy.init_node("rod_centerline_detector")

    rospy.Subscriber("/camera/image_raw", Image, image_callback, queue_size=1)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        if latest_image is None:
            continue

        img = latest_image.copy()

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # --- 1. edge detection ---
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)

        # --- 2. find contours ---
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if len(contours) == 0:
            cv2.imshow("camera", img)
            cv2.waitKey(1)
            rate.sleep()
            continue

        # --- 3. pick largest contour (assumed rod) ---
        largest = max(contours, key=cv2.contourArea)

        if len(largest) < 50:
            cv2.imshow("camera", img)
            cv2.waitKey(1)
            rate.sleep()
            continue

        pts = largest.reshape(-1, 2)

        # --- 4. compute centerline ---
        angle, mean, direction = get_centerline_angle(pts)

        # --- 5. draw contour ---
        cv2.drawContours(img, [largest], -1, (0, 255, 0), 2)

        # line for visualization
        p1 = (int(mean[0] - direction[0]*200), int(mean[1] - direction[1]*200))
        p2 = (int(mean[0] + direction[0]*200), int(mean[1] + direction[1]*200))

        cv2.line(img, p1, p2, (255, 0, 0), 2)

        # --- 6. display angle ---
        cv2.putText(img,
                    f"Rod angle: {angle:.2f} deg",
                    (30, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 255),
                    2)

        rospy.loginfo_throttle(1, f"Rod angle = {angle:.2f}°")

        cv2.imshow("camera", img)
        cv2.waitKey(1)

        rate.sleep()