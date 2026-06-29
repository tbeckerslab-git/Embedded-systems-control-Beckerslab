#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from collections import deque
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

bridge = CvBridge()

# history of rod midpoints (THIS is key for plane estimation)
trajectory = deque(maxlen=200)

def compute_plane_normal(points):
    """
    Fit plane to 3D-ish trajectory embedded in image space:
    (x, y, t)
    """
    if len(points) < 20:
        return None, 0.0

    P = np.array(points)

    # normalize
    P = P - np.mean(P, axis=0)

    # PCA
    _, _, vh = np.linalg.svd(P)
    normal = vh[-1]

    # confidence = how planar motion is
    singular_values = np.linalg.svd(P, compute_uv=False)
    confidence = 1.0 - (singular_values[-1] / (singular_values[0] + 1e-6))

    return normal, confidence


def image_callback(msg):
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    _, mask = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return

    rod = max(contours, key=cv2.contourArea)
    data = rod.reshape(-1,2)

    # PCA rod direction
    mean = np.mean(data, axis=0)
    cov = np.cov(data.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    direction = eigvecs[:, np.argmax(eigvals)]

    # store trajectory (x, y, pseudo-time)
    trajectory.append([mean[0], mean[1], len(trajectory)])

    # plane estimation
    normal, conf = compute_plane_normal(trajectory)

    if normal is None:
        return

    # camera optical axis in image frame = (0,0,1)
    cam_axis = np.array([0, 0, 1])

    # alignment angle
    tilt = np.degrees(
        np.arccos(
            np.clip(np.dot(normal, cam_axis), -1.0, 1.0)
        )
    )

    # drift (how much plane changes over time)
    drift = np.std(np.array(trajectory)[-50:], axis=0).mean()

    # visualization
    p1 = mean + direction * 100
    p2 = mean - direction * 100

    cv2.line(img, tuple(p1.astype(int)), tuple(p2.astype(int)), (0,255,0), 2)

    cv2.putText(img, f"Tilt: {tilt:.2f} deg", (30,50),
                cv2.FONT_HERSHEY_SIMPLEX, 1,(0,255,0),2)

    cv2.putText(img, f"Confidence: {conf:.2f}", (30,90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8,(255,0,0),2)

    cv2.putText(img, f"Drift: {drift:.2f}", (30,130),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8,(0,0,255),2)

    cv2.imshow("Camera Alignment Monitor", img)
    cv2.waitKey(1)

    rospy.loginfo_throttle(1.0, f"Tilt={tilt:.2f} deg | conf={conf:.2f}")

if __name__ == "__main__":
    rospy.init_node("camera_perp_monitor")

    rospy.Subscriber("/camera/image_raw", Image, image_callback, queue_size=1)

    rospy.spin()