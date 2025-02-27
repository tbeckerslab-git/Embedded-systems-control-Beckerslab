#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int16
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2


class PythonControllerNode(object):
    def __init__(self):
        self.pub = rospy.Publisher('/set_torque', Int16, queue_size=10)
        rospy.Subscriber("/positions", PointCloud2, self.callback)
        self.demo_counter = 0

    def callback(self, msg):
        points_list = []
    
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([point[0], point[1], point[2]])

        # Convert to NumPy array
        positions = np.array(points_list, dtype=np.float32)

        # TODO: use perform calculations here

        # Output your torque here
        if self.demo_counter < 100:
            self.demo_counter += 1
            self.pub.publish(self.demo_counter)
        elif self.demo_counter < 200:
            self.demo_counter += 1
            self.pub.publish(200 - self.demo_counter)
        else:
            self.demo_counter = 0


if __name__ == '__main__':
    rospy.init_node("python_controller", anonymous=True)
    my_node = PythonControllerNode()
    rospy.spin()