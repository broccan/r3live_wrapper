#!/usr/bin/env python
"""
master_convert.py

This master script converts an input ROS bag file into a new bag file formatted
for direct use with R3LIVE. It performs the following transformations:

1. Camera Images (/dji_osdk_ros/main_camera_images):
   - Convert from RGB (rgb8) to BGR (bgr8)
   - Apply undistortion using the provided camera calibration parameters
   - Generate a corresponding CameraInfo message
   - Output topics:
       /camera/image_color         (processed image)
       /camera/image_color_frame_info (CameraInfo)

2. LiDAR Point Clouds (/velodyne_points):
   - Remove extra fields (e.g., ring, time) so that only x, y, z, intensity remain
   - Output topic: /livox/lidar

3. IMU Data (/dji_osdk_ros/imu):
   - Retain header, angular_velocity, and linear_acceleration
   - Replace orientation with an identity quaternion and mark it invalid (covariance = -1)
   - Output topic: /livox/imu

All other topics are copied without modification.

Calibration Parameters:
  - Image Size: 4056 x 3040
  - Intrinsics:
      fx = 2914.5948836376165, fy = 2914.871933719483
      cx = 2040.550222088686,   cy = 1544.4466952001208
  - Distortion Coefficients:
      k1 = 0.32840253378312634, k2 = -1.2727385464137828,
      p1 = 0.000923390610538214, p2 = 0.00012362990419048574, k3 = 1.40321072388729

Usage:
    python3 master_convert.py <input_bag> <output_bag>
"""

import sys
import rosbag
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, Imu, CameraInfo, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

# ------------------ Camera Image Transformation ------------------ #
def undistort_image(cv_img, camera_matrix, dist_coeffs):
    """Apply undistortion to the input BGR image."""
    undistorted_img = cv2.undistort(cv_img, camera_matrix, dist_coeffs)
    return undistorted_img

def create_camera_info_msg(header):
    """
    Creates a CameraInfo message using the provided header and hard-coded calibration parameters.
    Note: The intrinsic parameters below assume an image size of 4056x3040.
    """
    cam_info = CameraInfo()
    cam_info.header = header
    cam_info.width = 4056
    cam_info.height = 3040

    # Intrinsic camera matrix (K)
    fx = 2914.5948836376165
    fy = 2914.871933719483
    cx = 2040.550222088686
    cy = 1544.4466952001208
    cam_info.K = [fx, 0, cx,
                  0, fy, cy,
                  0,  0,  1]

    # Distortion coefficients: [k1, k2, p1, p2, k3]
    cam_info.D = [0.32840253378312634,
                  -1.2727385464137828,
                  0.000923390610538214,
                  0.00012362990419048574,
                  1.40321072388729]

    cam_info.distortion_model = "plumb_bob"

    # Rectification matrix (R) and Projection matrix (P)
    cam_info.R = [1, 0, 0,
                  0, 1, 0,
                  0, 0, 1]
    cam_info.P = [fx, 0, cx, 0,
                  0, fy, cy, 0,
                  0, 0, 1, 0]
    return cam_info

def convert_and_undistort_image(msg, bridge, camera_matrix, dist_coeffs):
    """
    Converts a ROS Image message (assumed to be encoded as "rgb8") to a BGR image,
    applies undistortion, and returns a new Image message (encoded as "bgr8").
    Also returns a CameraInfo message with calibration details.
    """
    try:
        # Convert the ROS image to an OpenCV image (in RGB).
        rgb_image = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge error (RGB->CV): %s", e)
        return None, None

    # Convert from RGB to BGR.
    bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

    # Apply undistortion.
    undistorted_image = undistort_image(bgr_image, camera_matrix, dist_coeffs)

    try:
        # Convert the undistorted image back to a ROS Image message (BGR).
        new_img_msg = bridge.cv2_to_imgmsg(undistorted_image, encoding="bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge error (CV->ROS): %s", e)
        return None, None

    new_img_msg.header = msg.header
    # Create a CameraInfo message for this image.
    cam_info_msg = create_camera_info_msg(msg.header)
    return new_img_msg, cam_info_msg

# ------------------ LiDAR Point Cloud Transformation ------------------ #
def convert_pointcloud(msg):
    """
    Converts a sensor_msgs/PointCloud2 message by extracting only x, y, z, and intensity.
    This removes extra fields such as 'ring' and 'time'.
    """
    # Extract the desired fields.
    points = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
    if not points:
        rospy.logwarn("No points found in LiDAR message at time %.3f", msg.header.stamp.to_sec())
        return None

    # Define new point fields.
    new_fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1),
    ]
    new_cloud = pc2.create_cloud(msg.header, new_fields, points)
    return new_cloud

# ------------------ IMU Data Transformation ------------------ #
def convert_imu(msg):
    """
    Converts an IMU message by retaining header, angular_velocity, and linear_acceleration.
    Orientation is replaced with an identity quaternion and its covariance is set to -1.
    """
    new_imu = Imu()
    new_imu.header = msg.header
    new_imu.angular_velocity = msg.angular_velocity
    new_imu.linear_acceleration = msg.linear_acceleration

    # Set orientation to identity.
    new_imu.orientation.x = 0.0
    new_imu.orientation.y = 0.0
    new_imu.orientation.z = 0.0
    new_imu.orientation.w = 1.0

    # Mark orientation as invalid.
    new_imu.orientation_covariance = [-1.0] * 9

    # Optionally, preserve covariance for angular velocity and linear acceleration.
    new_imu.angular_velocity_covariance = msg.angular_velocity_covariance
    new_imu.linear_acceleration_covariance = msg.linear_acceleration_covariance

    return new_imu

# ------------------ Main Bag Processing ------------------ #
def process_bag(input_bag_file, output_bag_file):
    bridge = CvBridge()

    # Define camera calibration parameters.
    # (These parameters are as provided. Note: Ensure that the calibration resolution matches your data.)
    fx = 2914.5948836376165
    fy = 2914.871933719483
    cx = 2040.550222088686
    cy = 1544.4466952001208

    camera_matrix = np.array([[fx, 0, cx],
                              [0, fy, cy],
                              [0,  0,  1]], dtype=np.float32)

    dist_coeffs = np.array([0.32840253378312634,
                           -1.2727385464137828,
                            0.000923390610538214,
                            0.00012362990419048574,
                            1.40321072388729], dtype=np.float32)

    rospy.loginfo("Processing bag file %s", input_bag_file)
    with rosbag.Bag(output_bag_file, 'w') as outbag:
        with rosbag.Bag(input_bag_file, 'r') as inbag:
            for topic, msg, t in inbag.read_messages():
                # Process camera images.
                if topic == "/dji_osdk_ros/main_camera_images":
                    new_img_msg, cam_info_msg = convert_and_undistort_image(msg, bridge, camera_matrix, dist_coeffs)
                    if new_img_msg is not None and cam_info_msg is not None:
                        outbag.write("/camera/image_color", new_img_msg, t)
                        outbag.write("/camera/image_color_frame_info", cam_info_msg, t)
                        rospy.loginfo("Processed camera image at t=%.3f", t.to_sec())
                    else:
                        rospy.logerr("Failed to process camera image at t=%.3f", t.to_sec())
                # Process LiDAR point clouds.
                elif topic == "/velodyne_points":
                    new_cloud = convert_pointcloud(msg)
                    if new_cloud is not None:
                        outbag.write("/livox/lidar", new_cloud, t)
                        rospy.loginfo("Processed LiDAR cloud at t=%.3f", t.to_sec())
                    else:
                        rospy.logerr("Failed to process LiDAR cloud at t=%.3f", t.to_sec())
                # Process IMU data.
                elif topic == "/dji_osdk_ros/imu":
                    new_imu = convert_imu(msg)
                    if new_imu is not None:
                        outbag.write("/livox/imu", new_imu, t)
                        rospy.loginfo("Processed IMU data at t=%.3f", t.to_sec())
                    else:
                        rospy.logerr("Failed to process IMU data at t=%.3f", t.to_sec())
                else:
                    # Copy any other topics unchanged.
                    outbag.write(topic, msg, t)
    rospy.loginfo("Conversion complete. Output bag saved as %s", output_bag_file)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 master_convert.py <input_bag> <output_bag>")
        sys.exit(1)
    rospy.init_node("master_convert", anonymous=True)
    input_bag = sys.argv[1]
    output_bag = sys.argv[2]
    process_bag(input_bag, output_bag)

