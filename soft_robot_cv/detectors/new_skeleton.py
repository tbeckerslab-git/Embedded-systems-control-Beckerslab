import numpy as np
import cv2
from scipy.spatial import cKDTree
from skimage.morphology import skeletonize
from skimage.util import img_as_ubyte

import rospy
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import threading

def get_neighbors(p, skeleton_set):
    y, x = p
    neighbors = []
    for dy in [-1, 0, 1]:
        for dx in [-1, 0, 1]:
            if dy == 0 and dx == 0:
                continue
            neighbor = (y + dy, x + dx)
            if neighbor in skeleton_set:
                neighbors.append(neighbor)
    return neighbors


def order_skeleton(skeleton_img):
    coords = list(zip(*np.where(skeleton_img > 0)))
    if len(coords) == 0:
        return []

    skeleton_set = set(coords)

    # Find endpoints (pixels with 1 neighbor)
    endpoints = [p for p in coords if len(get_neighbors(p, skeleton_set)) == 1]

    if len(endpoints) == 0:
        # fallback: pick arbitrary start
        start = coords[0]
    else:
        start = endpoints[0]

    ordered = [start]
    visited = set([start])
    current = start

    while True:
        neighbors = get_neighbors(current, skeleton_set)
        next_points = [n for n in neighbors if n not in visited]

        if not next_points:
            break

        next_p = next_points[0]
        ordered.append(next_p)
        visited.add(next_p)
        current = next_p

    return np.array(ordered)


def resample_path(points, num_points=25):
    if len(points) < 2:
        return points

    # Compute cumulative distance
    dists = np.sqrt(((np.diff(points, axis=0))**2).sum(axis=1))
    cumulative = np.insert(np.cumsum(dists), 0, 0)

    total_length = cumulative[-1]
    target_distances = np.linspace(0, total_length, num_points)

    resampled = []
    for td in target_distances:
        idx = np.searchsorted(cumulative, td)

        if idx == 0:
            resampled.append(points[0])
        elif idx >= len(points):
            resampled.append(points[-1])
        else:
            t = ((td - cumulative[idx-1]) /
                 (cumulative[idx] - cumulative[idx-1] + 1e-8))
            interp = (1 - t) * points[idx-1] + t * points[idx]
            resampled.append(interp)

    return np.array(resampled)

class CVNode(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)

        # Publishers
        self.pub = rospy.Publisher('/skeleton', Float32MultiArray, queue_size=10)

        # Subscribers
        rospy.Subscriber("/camera/image_color", Image, self.callback)

        # Define the lower and upper bounds for the red color in BGR format
        self.lower_red = np.array([150, 150, 150])  # Lower bound for red
        self.upper_red = np.array([255, 255, 255])  # Upper bound for red

        # Initialize reference points
        self.reference_points = None
        # Store downsampled and aligned center points coordinates for each frame
        self.aligned_center_points = []
        self.distortion_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.camera_matrix = np.array([[1.0,  0.0, 0.0],
                                        [ 0.0, 1.0, 0.0],
                                        [ 0.0,  0.0,  1.0]])
        self.rotation = np.array([[1.0, 0.0, 0.0],
                                    [0.0, -1.0, 0.0],
                                    [0.0, 0.0, -1.0]])

        self.translation = np.array([0.0, 0.0, 0.0])
        self.inverse_K = np.linalg.inv(self.camera_matrix)
        self.inverse_rotation = np.linalg.inv(self.rotation)
        self.world = []
        
        # Flag to indicate whether to update the image
        self.update_image = False
        # self.image_thread = threading.Thread(target=self.display_image)
        # self.image_thread.start()

    def callback(self, msg):
        self.image = self.br.imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

        # Threshold
        _, mask = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)

        # Morphology
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, 2)
        mask = cv2.erode(mask, kernel, 2)

        cv2.imshow("Grayscale Mask", mask)

        # Largest component
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        largest = max(contours, key=cv2.contourArea)
        mask_clean = np.zeros_like(mask)
        cv2.drawContours(mask_clean, [largest], -1, 255, -1)

        # TODO: Any cv changes would be here probably
        # Skeletonize
        skeleton = skeletonize(mask_clean > 0)
        skeleton = img_as_ubyte(skeleton)

        # 👉 ORDERED PATH
        ordered_path = order_skeleton(skeleton)

        if len(ordered_path) < 5:
            return

        # 👉 RESAMPLE TO 25 POINTS
        sampled = resample_path(ordered_path, 25)

        self.reference_points = sampled

        # Convert to homogeneous coords
        homogeneous = np.hstack((sampled, np.ones((sampled.shape[0], 1))))
        self.aligned_center_points = homogeneous.tolist()
        xy_points = [[pt[0], pt[1]] for pt in self.aligned_center_points]
        flat_points = np.array(xy_points, dtype=np.float32).flatten()
        msg = Float32MultiArray()
        msg.data = flat_points.tolist()

        # Publish the message
        self.pub.publish(msg)

        # Visualization
        vis = self.image.copy()
        for pt in sampled:
            cv2.circle(vis, (int(pt[1]), int(pt[0])), 3, (0, 255, 0), -1)

        cv2.imshow("Skeleton Overlay", vis)
        cv2.waitKey(1)

    def display_image(self):
        while not rospy.is_shutdown():
            if self.update_image and self.image is not None:
                # Overlay skeleton points on the original image
                for coord in self.reference_points:
                    # Draw a small circle for each skeletonized point
                    vis = self.image.copy()

                    for coord in self.reference_points:
                        cv2.circle(
                            vis,
                            (int(coord[1]), int(coord[0])),
                            2,
                            (0, 255, 0),
                            -1
                        )

                    cv2.imshow("Skeleton Overlay", vis)
                    cv2.waitKey(1)


                # Display the image with overlayed skeletonized points
                cv2.imshow("Skeleton Overlay", self.image)
                cv2.waitKey(1)  # Necessary to update the display
                self.update_image = False  # Reset flag to avoid continuous display
            else:
                rospy.sleep(0.1)  # Sleep briefly to reduce CPU usage
                print("no image")
    

if __name__ == '__main__':
    rospy.init_node("skeleton", anonymous=True)
    my_node = CVNode()
    rospy.spin()

    cv2.destroyAllWindows()