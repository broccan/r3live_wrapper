#!/usr/bin/env python
"""
check_snippet_bag_data.py

This script inspects a ROS bag file (the snippet bag) and prints information about the data and datatypes for:
  1. Camera Images (expected topic: /dji_osdk_ros/main_camera_images, expected type: sensor_msgs/Image)
  2. LiDAR Point Clouds (expected topic: /velodyne_points, expected type: sensor_msgs/PointCloud2)
  3. IMU Data (expected topic: /dji_osdk_ros/imu, expected type: sensor_msgs/Imu)

It prints the actual internal type (_type) of each message so you can verify whether they match what you expect.
"""

import sys
import rosbag
import rospy
from sensor_msgs.msg import Image, Imu, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

def inspect_camera_images(bag, topic, num_msgs=3):
    print("==== Inspecting Camera Images on topic: {} ====".format(topic))
    bridge = CvBridge()
    count = 0
    for _, msg, t in bag.read_messages(topics=[topic]):
        if count >= num_msgs:
            break
        print("\nMessage Timestamp: {:.3f}".format(t.to_sec()))
        # Print the internal type string:
        print("  _type: {}".format(getattr(msg, '_type', 'N/A')))
        # Check if msg is an instance of sensor_msgs/Image.
        if not isinstance(msg, Image):
            print("ERROR: Message on {} is not of type sensor_msgs/Image (using isinstance check).".format(topic))
        else:
            print("OK: Message is a sensor_msgs/Image instance.")
        print("  Encoding: {}".format(msg.encoding))
        print("  Dimensions: {} x {}".format(msg.width, msg.height))
        try:
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)
            print("  CV Image Shape: {}".format(cv_img.shape))
            if len(cv_img.shape) == 3:
                mean_vals = np.mean(cv_img, axis=(0,1))
                print("  Mean Pixel Value (per channel): {}".format(mean_vals))
            else:
                print("  Mean Pixel Value: {:.3f}".format(np.mean(cv_img)))
        except CvBridgeError as e:
            print("CvBridge Error: {}".format(e))
        count += 1

def inspect_pointcloud(bag, topic, num_msgs=3):
    print("==== Inspecting LiDAR PointCloud on topic: {} ====".format(topic))
    count = 0
    for _, msg, t in bag.read_messages(topics=[topic]):
        if count >= num_msgs:
            break
        print("\nMessage Timestamp: {:.3f}".format(t.to_sec()))
        print("  _type: {}".format(getattr(msg, '_type', 'N/A')))
        if not isinstance(msg, PointCloud2):
            print("ERROR: Message on {} is not of type sensor_msgs/PointCloud2.".format(topic))
        else:
            print("OK: Message is a sensor_msgs/PointCloud2 instance.")
        fields = [f.name for f in msg.fields]
        print("  Point Fields: {}".format(fields))
        points = list(pc2.read_points(msg, field_names=fields, skip_nans=True))
        num_sample = min(5, len(points))
        print("  Sample Points (first {}):".format(num_sample))
        for pt in points[:num_sample]:
            print("    {}".format(pt))
        count += 1

def inspect_imu(bag, topic, num_msgs=3):
    print("==== Inspecting IMU Data on topic: {} ====".format(topic))
    count = 0
    for _, msg, t in bag.read_messages(topics=[topic]):
        if count >= num_msgs:
            break
        print("\nMessage Timestamp: {:.3f}".format(t.to_sec()))
        print("  _type: {}".format(getattr(msg, '_type', 'N/A')))
        if not isinstance(msg, Imu):
            print("ERROR: Message on {} is not of type sensor_msgs/Imu.".format(topic))
        else:
            print("OK: Message is a sensor_msgs/Imu instance.")
        print("  Header:")
        print("    frame_id: {}".format(msg.header.frame_id))
        print("    seq: {}".format(msg.header.seq))
        print("    stamp: {:.3f}".format(msg.header.stamp.to_sec()))
        print("  Orientation: x={:.3f}, y={:.3f}, z={:.3f}, w={:.3f}".format(
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))
        print("  Angular Velocity: x={:.3f}, y={:.3f}, z={:.3f}".format(
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))
        print("  Linear Acceleration: x={:.3f}, y={:.3f}, z={:.3f}".format(
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))
        count += 1

def main(bag_file):
    try:
        bag = rosbag.Bag(bag_file, 'r')
    except Exception as e:
        print("Failed to open bag file {}: {}".format(bag_file, e))
        sys.exit(1)

    print("=== Checking Bag File: {} ===".format(bag_file))
    inspect_camera_images(bag, "/dji_osdk_ros/main_camera_images")
    inspect_pointcloud(bag, "/velodyne_points")
    inspect_imu(bag, "/dji_osdk_ros/imu")
    bag.close()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python check_snippet_bag_data.py <bag_file>")
        sys.exit(1)
    bag_file = sys.argv[1]
    rospy.init_node("check_snippet_bag_data", anonymous=True)
    main(bag_file)

