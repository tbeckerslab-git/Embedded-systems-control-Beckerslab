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


class Nodo(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)

        # Publishers
        self.pub = rospy.Publisher('/positions', Int16, queue_size=10)

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

    def callback(self, msg):
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

        # Store aligned center points for the current frame
        self.aligned_center_points.append(self.reference_points.tolist())

        # Draw the consistent downsampled centerline points on the original frame
        downsampled_frame = self.image.copy()
        for x, y in self.reference_points:
            cv2.circle(downsampled_frame, (y, x), 3, (0, 255, 0), -1)  # Draw the points as small green circles

        # Display the frames (optional, can slow down processing)
        # skeleton = skeleton.astype(np.uint8) * 255
        # cv2.imshow('Skeleton Frame', skeleton)
        # cv2.imshow('Downsampled Centerline Points Frame', downsampled_frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     cv2.destroyAllWindows()


    #     fields = [
    #     PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    #     PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    #     PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
    # ]
        
    #     header = rospy.Header()
    #     header.stamp = rospy.Time.now()
    #     header.frame_id = 'map' # Replace 'map' with your desired frame ID
        
    #     pc2_msg = pc2.create_cloud(header, fields, points)
        print(self.aligned_center_points)


if __name__ == '__main__':
    rospy.init_node("skeleton", anonymous=True)
    my_node = Nodo()
    rospy.spin()

    cv2.destroyAllWindows()

        # #Sort by list
        # for frame_index, points in enumerate(aligned_center_points):
        #     temp_lst = []
        #     for point in points:
        #         temp = point[0]
        #         point[0] = point[1]
        #         point[1] = temp
        #         temp_lst.append(point)
        #     temp_lst = np.array(temp_lst)



        #     # Sort the array by the first coordinate (x-coordinate after swapping)
        #     point_sort = temp_lst[temp_lst[:, 0].argsort()]

        #     # If you need to store or further use sorted points for each frame, you can do so here
        #     aligned_center_points[frame_index] = point_sort.tolist()



    # pts_movements = []
    # with open('aligned_center_points.txt', 'w') as file:
    #     for frame_index, points in enumerate(aligned_center_points):
    #         file.write(f"Frame {frame_index + 1}:\n")
    #         pts_frame = []
    #         for point in points:
    #             # temp = point[0]

    #             # point[0] = point[1]

    #             # point[1] = temp

    #             file.write(f"{point}\n")

    #             # point = point.tolist()

    #             point.insert(2, frame_index)  # x coordinate

    #             pts_frame.append(point)

    #         file.write("\n")

    #         pts_movements.append(pts_frame)


    #     points_data = np.array(pts_movements)

    #     x_to_mat = []
    #     y_to_mat = []
    #     z_to_mat = []

    #     for i in range(points_data.shape[0]):
    #         string = points_data[i][:, :]
    #         # string_sort = string[string[:, 0].argsort()]
    #         x_to_mat.append(string[:, 0])
    #         y_to_mat.append(string[:, 1])
    #         z_to_mat.append(string[:, 2])
    #         # horizontal = string_sort[:, 0]
    #         # vertical = string_sort[:, 1]

    #     # Convert lists to numpy arrays
    #     x = np.array(x_to_mat).flatten()
    #     y = np.array(y_to_mat).flatten()
    #     z = np.array(z_to_mat).flatten()



    #     # Save data to a .mat file
    #     savemat('aligned_center_points.mat', {'x': x, 'y': y, 'z': z})
    #     print('mat saved')

    #     # Select the frame you want to plot
    #     frame_index = 0  # Change this to plot a different frame
    #     # Create a 3D scatter plot
    #     x_all, y_all, z_all = [], [], []


    #     fig = plt.figure()
    #     ax = fig.add_subplot(111, projection='3d')



    #     for frame_index in range(0, points_data.shape[0], 5):  #

    #         frame_points = points_data[frame_index]



    #         # Extract x, y, z coordinates

    #         x = frame_points[:, 0]  # width --horizontal

    #         y = frame_points[:, 1]  # height --vertical

    #         z = frame_points[:, 2]  # time



    #         # Append to the lists

    #         x_all.extend(x)

    #         y_all.extend(y)

    #         z_all.extend(z)



    #         # Scatter plot for each frame

    #         ax.scatter(x, y, z)



    #     # Convert lists to numpy arrays

    #     x_all = np.array(x_all)

    #     y_all = np.array(y_all)

    #     z_all = np.array(z_all)



    #     # Set labels for the scatter plot

    #     ax.set_xlabel('Horizontal axis')

    #     ax.set_ylabel('Vertical axis')

    #     ax.set_zlabel('Time axis')



    #     plt.show()  # Show the scatter plot


