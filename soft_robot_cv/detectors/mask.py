import cv2
import numpy as np
import torch
from scipy.io import savemat
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Int16
from cv_bridge import CvBridge
from scipy.interpolate import splprep, splev

def smooth_midline(points, smoothing_factor=0.1, trim_start=0.05, trim_end=0.05):
    """
    Smooth centerline points using a parametric spline.

    Args:
        points (Nx2 array): [row, col] coordinates
        smoothing_factor: higher = smoother
        trim_start: percent trimmed from base
        trim_end: percent trimmed from tip

    Returns:
        Nx2 smoothed centerline points
    """

    if len(points) < 4:
        return points

    points = np.array(points)

    rows = points[:,0].astype(float)
    cols = points[:,1].astype(float)

    # trim artifacts at ends
    n = len(rows)
    trim_s = int(n * trim_start)
    trim_e = int(n * trim_end)

    if trim_s + trim_e < n - 4:
        if trim_e > 0:
            rows = rows[trim_s:-trim_e]
            cols = cols[trim_s:-trim_e]
        else:
            rows = rows[trim_s:]
            cols = cols[trim_s:]

    try:
        num_points = len(rows)

        s_value = num_points * smoothing_factor

        tck, u = splprep([cols, rows], s=s_value, k=3)

        u_new = np.linspace(0, 1, num_points)

        smooth_cols, smooth_rows = splev(u_new, tck)

        smoothed = np.vstack([smooth_rows, smooth_cols]).T

        return smoothed

    except Exception as e:
        rospy.logwarn(f"Spline smoothing failed: {e}")
        return points

def reconstruct_skeleton_image(self, points, shape):
    """
    Convert smoothed centerline points into a binary skeleton image.

    Args:
        points: Nx2 array [row, col]
        shape: (height, width)

    Returns:
        skeleton_img: binary image
    """

    skeleton_img = np.zeros(shape, dtype=np.uint8)

    if len(points) < 2:
        return skeleton_img

    rows = np.clip(points[:,0].astype(int), 0, shape[0]-1)
    cols = np.clip(points[:,1].astype(int), 0, shape[1]-1)

    # draw curve
    for i in range(len(rows)-1):
        cv2.line(
            skeleton_img,
            (cols[i], rows[i]),
            (cols[i+1], rows[i+1]),
            255,
            1
        )

    return skeleton_img

def extract_centerline(self, mask):
    """
    Compute midpoints of mask rows to get centerline.
    Assumes soft body anchored at top and swinging like pendulum.
    """

    rows, cols = mask.shape
    centerline = []

    for r in range(rows):

        xs = np.where(mask[r] > 0)[0]

        if len(xs) > 0:
            midpoint = int((xs[0] + xs[-1]) / 2)
            centerline.append([r, midpoint])

    return np.array(centerline)

class CVNode(object):
    def __init__(self):
        self.image = None
        self.br = CvBridge()

        self.pub = rospy.Publisher('/positions', PointCloud2, queue_size=10)
        rospy.Subscriber("/camera/image_color", Image, self.callback)

        self.lower_red = np.array([0, 0, 60])
        self.upper_red = np.array([80, 80, 255])

        self.reference_points = None
        self.aligned_center_points = []

        self.distortion_coeffs = np.array([0,0,0,0,0])

        self.camera_matrix = np.array([
            [1.0,0.0,0.0],
            [0.0,1.0,0.0],
            [0.0,0.0,1.0]
        ])

        self.rotation = np.array([
            [1,0,0],
            [0,-1,0],
            [0,0,-1]
        ])

        self.translation = np.array([0,0,0])

        self.inverse_K = np.linalg.inv(self.camera_matrix)
        self.inverse_rotation = np.linalg.inv(self.rotation)

        self.world = []

        self.skeleton_pub = rospy.Publisher('/skeleton', Image, queue_size=10)

    # def extract_centerline(self, mask):
        """
        Extract centerline by computing midpoints of mask rows.
        """
        centers = []

        rows = np.where(mask > 0)[0]
        if len(rows) == 0:
            return np.array([])

        min_row = rows.min()
        max_row = rows.max()

        for y in range(min_row, max_row):

            xs = np.where(mask[y] > 0)[0]

            if len(xs) < 2:
                continue

            left = xs.min()
            right = xs.max()

            center_x = (left + right) / 2

            centers.append([y, center_x])

        return np.array(centers)

    def callback(self, msg):

        self.image = self.br.imgmsg_to_cv2(msg)

        # Red mask
        mask = cv2.inRange(self.image, self.lower_red, self.upper_red)

        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Extract centerline
        centerline = extract_centerline(mask)

        if len(centerline) == 0:
            return

        centerline = smooth_midline(centerline)

        # Downsample to 100 points
        if len(centerline) > 100:
            idx = np.linspace(0, len(centerline)-1, 100).astype(int)
            centerline = centerline[idx]

        self.reference_points = centerline

        ref_points = np.array(self.reference_points)

        homogeneous_points = np.hstack((
            ref_points,
            np.ones((ref_points.shape[0],1))
        ))

        self.aligned_center_points = homogeneous_points.tolist()

        for pixel_coord in self.aligned_center_points:

            cam_coords = self.inverse_K @ pixel_coord

            s = 1

            world_coords = self.inverse_rotation @ (s * cam_coords - self.translation)

            self.world.append(world_coords)

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'

        pc2_msg = pc2.create_cloud(header, fields, self.world)

        self.pub.publish(pc2_msg)

        self.world = []

        skeleton_img = self.reconstruct_skeleton_image(
            centerline,
            mask.shape
        )
        skeleton_msg = self.br.cv2_to_imgmsg(skeleton_img, encoding="mono8")
        self.skeleton_pub.publish(skeleton_msg)