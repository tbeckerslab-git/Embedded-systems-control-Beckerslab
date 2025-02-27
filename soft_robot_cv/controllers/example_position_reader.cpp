#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <iostream>

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    // Check if the cloud has any points
    if (cloud_msg->width == 0 || cloud_msg->height == 0) {
        ROS_WARN("Empty PointCloud received!");
        return;
    }

    // Verify that the cloud contains x, y, z fields
    int offset_x = -1, offset_y = -1, offset_z = -1;
    for (const auto& field : cloud_msg->fields) {
        if (field.name == "x") offset_x = field.offset;
        if (field.name == "y") offset_y = field.offset;
        if (field.name == "z") offset_z = field.offset;
    }

    if (offset_x == -1 || offset_y == -1 || offset_z == -1) {
        ROS_ERROR("PointCloud2 message does not have x, y, z fields!");
        return;
    }

    // Decode the points
    for (size_t i = 0; i < cloud_msg->width; ++i) { // Iterate through points
        // Calculate the starting position of this point
        size_t point_offset = i * cloud_msg->point_step;

        // Extract x, y, z
        float x, y, z;
        memcpy(&x, &cloud_msg->data[point_offset + offset_x], sizeof(float));
        memcpy(&y, &cloud_msg->data[point_offset + offset_y], sizeof(float));
        memcpy(&z, &cloud_msg->data[point_offset + offset_z], sizeof(float));

        // Print the point
        ROS_INFO("Point[%zu]: x=%.6f, y=%.4f, z=%.8f", i, x, y, z);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_listener");
    ros::NodeHandle nh;

    // Subscribe to the PointCloud2 topic
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/positions", 10, pointCloudCallback);

    ros::spin();
    return 0;
}
