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


class CVNode(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)

        # Publishers
        self.pub = rospy.Publisher('/positions', PointCloud2, queue_size=10)

        # Subscribers
        rospy.Subscriber("/camera/image_color", Image, self.callback)

        # Define the lower and upper bounds for the red color in BGR format
        # Note: These bounds may need adjustment depending on the specific red shades in your video
        self.lower_red = np.array([0, 0, 60])  # Lower bound for red
        self.upper_red = np.array([80, 80, 255])  # Upper bound for red

        self.frame_width = 1440
        self.frame_height = 1080

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

    def callback(self, msg):
        # Convert the ROS message into an OpenCV matrix
        self.image = self.br.imgmsg_to_cv2(msg)

        # Create a mask for the red color
        mask = cv2.inRange(self.image, self.lower_red, self.upper_red)

        # Perform morphological operations to connect scattered regions
        kernel = np.ones((5, 5), np.uint8)
        mask_dilated = cv2.dilate(mask, kernel, iterations=2)
        mask_eroded = cv2.erode(mask_dilated, kernel, iterations=2)

        # Apply skeletonization
        skeleton = skeletonize(mask_eroded > 0)

        # Find the coordinates of the centerline (skeleton) pixels
        skeleton_coords = np.column_stack(np.where(skeleton > 0))

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

        # # Store aligned center points for the current frame
        # self.aligned_center_points.append(self.reference_points.tolist()) #Refactor to get 3d coords
        ref_points = np.array(self.reference_points)
        homogeneous_points = np.hstack((ref_points, np.ones((ref_points.shape[0], 1), dtype=ref_points.dtype)))
        # self.aligned_center_points.append(homogeneous_points.tolist())
        self.aligned_center_points  = homogeneous_points.tolist()


        # print(len(self.aligned_center_points))
        for pixel_coord in self.aligned_center_points:
            cam_coords = self.inverse_K @ pixel_coord
            # Assuming world Z = 0 (flat surface assumption)
            s = 1 #(0 - self.translation[2]) / (self.rotation[2] @ cam_coords)  # Scale factor

            world_coords = self.inverse_rotation @ (s * cam_coords - self.translation)
            self.world.append(world_coords)

        # Convert the pixel coordinates into world coordinates

        fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
    ]
        
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map' # Replace 'map' with your desired frame ID
        
        pc2_msg = pc2.create_cloud(header, fields, self.world)
        self.pub.publish(pc2_msg)

        self.world = []
        


if __name__ == '__main__':
    rospy.init_node("skeleton", anonymous=True)
    my_node = CVNode()
    rospy.spin()

    cv2.destroyAllWindows()
