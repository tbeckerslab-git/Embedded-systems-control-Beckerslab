#!/usr/bin/env python

import cv2
import numpy as np
import torch
from filterpy.kalman import KalmanFilter
from skimage.morphology import skeletonize
from skimage.util import img_as_ubyte
from scipy.spatial import cKDTree
from scipy.interpolate import UnivariateSpline, CubicSpline
import matplotlib.pyplot as plt
from scipy.io import savemat
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Int16
from cv_bridge import CvBridge
from skimage.morphology import skeletonize
import threading
from std_msgs.msg import Header

def load_predictions(file_path='predicted_test_heights.mat'):
    """
    Loads predicted test heights from a .mat file.
    Args:
    file_path (str): Path to the .mat file containing the predictions.
    Returns:
    np.ndarray: The loaded predicted test heights.
    """

    mat_contents = sio.loadmat(file_path)
    predicted_test_heights = mat_contents['predicted_test_heights']
    print(f"Loaded predicted test heights from '{file_path}'.")
    return predicted_test_heights

def apply_kalman_filter(noisy_data):

    noisy_data = np.squeeze(noisy_data).T
    print(noisy_data.shape[0])
    num_dimensions = noisy_data.shape[0]

    # Create a Kalman Filter instance
    kf = KalmanFilter(dim_x=2 * num_dimensions, dim_z=num_dimensions)

    # Configuration of the Kalman filter (F, H, R, Q, P as previously discussed)
    # Initial state

    kf.x = np.zeros(2 * num_dimensions)
    kf.F = np.eye(2 * num_dimensions)
    for i in range(num_dimensions):
        kf.F[i, i + num_dimensions] = 1  # assuming dt=1 for simplicity

    kf.H = np.zeros((num_dimensions, 2 * num_dimensions))
    kf.H[:, :num_dimensions] = np.eye(num_dimensions)
    kf.R = 0.1 * np.eye(num_dimensions)
    kf.Q = 0.1 * np.eye(2 * num_dimensions)
    kf.P *= 1000

    # Apply the Kalman filter
    filtered_states = np.zeros_like(noisy_data)
    print(noisy_data.shape)
    for i in range(noisy_data.shape[-1]):
        for j in range(noisy_data.shape[1]):
            kf.predict()
            # Proper reshaping of measurement vector
            kf.update(noisy_data[:, j, i].reshape(-1, 1))

            # print(kf.x[:num_dimensions])
            filtered_states[:, j, i] = kf.x[:num_dimensions]

    return filtered_states

def rotate_to_horizontal_3d(points):
    """
    Rotate a set of 2D points such that the line through the first and last points becomes horizontal.
    This version handles 3D arrays where multiple sets of points are stored in a single array.
    Args:
    points (numpy array): A 3D array of shape (n_sets, n_points, 2), where n_sets is the number of point sets,
                          n_points is the number of points in each set.

    Returns:
    rotated_points (numpy array): The rotated 3D points with the line through the first and last points in each set aligned horizontally.
    """
    rotated_points = np.zeros_like(points)  # Create an array to store the rotated points

    for i in range(points.shape[0]):  # Loop over each set of points
        # Extract the set of points
        current_points = points[i]

        # Define the two end points (the first and last points in the set)
        point1 = current_points[0]
        point2 = current_points[-1]

        # Compute the angle of the line relative to the horizontal axis
        dx = point2[0] - point1[0]
        dy = point2[1] - point1[1]
        angle = np.arctan2(dy, dx)

        # Create a rotation matrix to rotate by the negative of the angle
        rotation_matrix = np.array([[np.cos(-angle), -np.sin(-angle)],
                                    [np.sin(-angle), np.cos(-angle)]])

        # Rotate all the points around point1
        rotated_points[i] = (current_points - point1) @ rotation_matrix.T + point1

    return rotated_points

def extract_longest_path(skeleton):
    """
    Extract the longest path (main centerline) from a skeleton image.
    This removes short branches/spurs and keeps only the main line.


    Args:
        skeleton: Binary skeleton image (numpy array)


    Returns:
        pruned_skeleton: Binary image with only the longest path
    """
    from scipy import ndimage
    from collections import deque


    # Find all skeleton pixels
    skel_points = np.argwhere(skeleton > 0)
    if len(skel_points) < 2:
        return skeleton


    # Build adjacency using 8-connectivity
    skel_set = set(map(tuple, skel_points))


    # Find endpoints (pixels with only 1 neighbor)
    endpoints = []
    for pt in skel_points:
        r, c = pt
        neighbors = 0
        for dr in [-1, 0, 1]:
            for dc in [-1, 0, 1]:
                if dr == 0 and dc == 0:
                    continue
                if (r + dr, c + dc) in skel_set:
                    neighbors += 1
        if neighbors == 1:
            endpoints.append(tuple(pt))


    # If no endpoints found (closed loop), just return original
    if len(endpoints) < 2:
        return skeleton


    # BFS to find longest path between any two endpoints
    def bfs_longest_path(start):
        visited = {start}
        queue = deque([(start, [start])])
        longest_path = [start]


        while queue:
            current, path = queue.popleft()
            if len(path) > len(longest_path):
                longest_path = path


            r, c = current
            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    if dr == 0 and dc == 0:
                        continue
                    neighbor = (r + dr, c + dc)
                    if neighbor in skel_set and neighbor not in visited:
                        visited.add(neighbor)
                        queue.append((neighbor, path + [neighbor]))


        return longest_path


    # Find the longest path starting from each endpoint
    best_path = []
    for endpoint in endpoints:
        path = bfs_longest_path(endpoint)
        if len(path) > len(best_path):
            best_path = path


    # Create new skeleton with only the longest path
    pruned_skeleton = np.zeros_like(skeleton)
    for r, c in best_path:
        pruned_skeleton[r, c] = 255


    return pruned_skeleton

def smooth_skeleton_path(skeleton, smoothing_factor=0.1, trim_start=0.05, trim_end=0.05):
    """
    Smooth the skeleton path using spline fitting to remove kinks.


    Args:
        skeleton: Binary skeleton image (numpy array)
        smoothing_factor: Controls smoothness (0 = interpolate exactly, higher = smoother)
        trim_start: Percentage of points to trim from the start/base (0.05 = 5%)
        trim_end: Percentage of points to trim from the end/tip (0.05 = 5%)


    Returns:
        smoothed_skeleton: Binary image with smoothed path
    """
    from scipy.interpolate import splprep, splev


    # Find all skeleton pixels
    skel_points = np.argwhere(skeleton > 0)
    if len(skel_points) < 4:  # Need at least 4 points for spline
        return skeleton


    # Sort points along the path (assuming path is roughly ordered)
    # Use the order from extract_longest_path if available
    # Otherwise, sort by row (vertical position)
    sorted_indices = np.argsort(skel_points[:, 0])
    sorted_points = skel_points[sorted_indices]


    # Trim points from start and end separately to remove curls/artifacts
    num_to_trim_start = int(len(sorted_points) * trim_start)
    num_to_trim_end = int(len(sorted_points) * trim_end)
    total_trim = num_to_trim_start + num_to_trim_end
    if total_trim > 0 and len(sorted_points) > total_trim + 4:
        if num_to_trim_end > 0:
            sorted_points = sorted_points[num_to_trim_start:-num_to_trim_end]
        else:
            sorted_points = sorted_points[num_to_trim_start:]


    # Extract x (column) and y (row) coordinates
    rows = sorted_points[:, 0].astype(float)
    cols = sorted_points[:, 1].astype(float)


    # Fit a parametric spline
    try:
        # Use smoothing spline - s controls smoothness
        # Higher s = smoother curve
        num_points = len(rows)
        s_value = num_points * smoothing_factor  # Scale smoothing with number of points


        tck, u = splprep([cols, rows], s=s_value, k=3)


        # Evaluate spline at same number of points as original
        u_new = np.linspace(0, 1, num_points)
        smooth_cols, smooth_rows = splev(u_new, tck)


        # Round to integer pixel coordinates
        smooth_rows = np.round(smooth_rows).astype(int)
        smooth_cols = np.round(smooth_cols).astype(int)


        # Clip to image bounds
        smooth_rows = np.clip(smooth_rows, 0, skeleton.shape[0] - 1)
        smooth_cols = np.clip(smooth_cols, 0, skeleton.shape[1] - 1)


        # Create new skeleton with smoothed path
        smoothed_skeleton = np.zeros_like(skeleton)
        for r, c in zip(smooth_rows, smooth_cols):
            smoothed_skeleton[r, c] = 255


        # Fill gaps by drawing lines between consecutive points
        for i in range(len(smooth_rows) - 1):
            cv2.line(smoothed_skeleton,
                    (smooth_cols[i], smooth_rows[i]),
                    (smooth_cols[i+1], smooth_rows[i+1]),
                    255, 1)


        return smoothed_skeleton


    except Exception as e:
        # If spline fitting fails, return original
        print(f"Spline smoothing failed: {e}")
        return skeleton


class CVNode(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)

        # Publishers
        self.pub = rospy.Publisher('/skeleton', PointCloud2, queue_size=10)

        # Subscribers
        rospy.Subscriber("/camera/image_color", Image, self.callback)

        # Define the lower and upper bounds for the red color in BGR format
        self.lower_red = np.array([30, 30, 30])  # Lower bound for red
        self.upper_red = np.array([80, 80, 80])  # Upper bound for red

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
        self.image_thread = threading.Thread(target=self.display_image)
        self.image_thread.start()

    def callback(self, msg):
        # Convert the ROS message into an OpenCV matrix
        self.image = self.br.imgmsg_to_cv2(msg)
        gray_frame = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

            # Create a mask by thresholding - use percentile-based threshold with minimum floor
        # Detect pixels brighter than 99th percentile (top 1% brightest pixels)
        # Also require pixels to be above a minimum brightness to ignore dim background spots
        min_brightness = 10  # Minimum pixel intensity to consider (adjust if needed)
        threshold_val = max(np.percentile(gray_frame, 90), min_brightness)
        _, mask = cv2.threshold(gray_frame, threshold_val, 255, cv2.THRESH_BINARY)


        # Perform morphological operations to connect scattered regions
        kernel = np.ones((5, 5), np.uint8)
        mask_dilated = cv2.dilate(mask, kernel, iterations=2)
        mask_eroded = cv2.erode(mask_dilated, kernel, iterations=2)


        # Keep only the largest connected component (the main object, ignore noise/dust)
        contours, _ = cv2.findContours(mask_eroded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            # Find the largest contour by area
            largest_contour = max(contours, key=cv2.contourArea)
            # Create a new mask with only the largest contour
            mask_eroded = np.zeros_like(mask_eroded)
            cv2.drawContours(mask_eroded, [largest_contour], -1, 255, -1)

        # Apply skeletonization
        skeleton = skeletonize(mask_eroded > 0)
        print("skeleton done")
        skeleton = img_as_ubyte(skeleton)  # Convert to uint8 format for saving


        # Extract only the longest path (main centerline), removing short branches
        skeleton = extract_longest_path(skeleton)
        print("longest path done   ")


        # Smooth the skeleton path to remove kinks (adjust smoothing_factor as needed)
        # Higher value = smoother line, lower value = follows original path more closely
        # trim_start/trim_end remove points from each end separately to eliminate curls/artifacts
        # Adjust these values independently based on where curls appear (base vs tip)
        skeleton = smooth_skeleton_path(skeleton, smoothing_factor=50.0, trim_start=0.05, trim_end=0.15)


        # Find the coordinates of the centerline (skeleton) pixels
        skeleton_coords = np.column_stack(np.where(skeleton > 0))

        cv2.imshow('Skeleton Frame', skeleton)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            return

        cv2.destroyAllWindows()
        print("displ")


        # Skip frame if no skeleton points detected
        if len(skeleton_coords) == 0:
            self.aligned_center_points.append([])
            return


        # Downsample or match points
        if self.reference_points is None:
            # Initialize reference points from the first frame
            if len(skeleton_coords) > 30:
                indices = np.linspace(0, len(skeleton_coords) - 1, 100).astype(int)
                self.reference_points = skeleton_coords[indices]
            else:
                self.reference_points = skeleton_coords
        else:
            # Match current frame points to the reference points using nearest neighbor
            tree = cKDTree(skeleton_coords)
            _, indices = tree.query(self.reference_points)
            self.reference_points = skeleton_coords[indices]

        # Store aligned center points for the current frame
        ref_points = np.array(self.reference_points)
        homogeneous_points = np.hstack((ref_points, np.ones((ref_points.shape[0], 1), dtype=ref_points.dtype)))
        self.aligned_center_points = homogeneous_points.tolist()

        for pixel_coord in self.aligned_center_points:
            cam_coords = self.inverse_K @ pixel_coord
            s = 1  # Scale factor (flat surface assumption)
            world_coords = self.inverse_rotation @ (s * cam_coords - self.translation)
            self.world.append(world_coords)

        # Convert the pixel coordinates into world coordinates
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'  # Replace 'map' with your desired frame ID
        
        pc2_msg = pc2.create_cloud(header, fields, self.world)
        self.pub.publish(pc2_msg)

        self.world = []

        # Set flag to update the image
        self.update_image = True
        print("done callback")

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
