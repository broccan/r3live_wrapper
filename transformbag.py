import rosbag
from sensor_msgs.msg import PointCloud2, PointField, Imu, Image
from cv_bridge import CvBridge
import struct
import numpy as np
import cv2

# Paths
input_bag_path = 'snippet1.bag'
output_bag_path = 'transformed_snippet1.bag'

# Topics
camera_topic_in = '/dji_osdk_ros/main_camera_images'
camera_topic_out = '/camera/image_color'

lidar_topic_in = '/velodyne_points'
lidar_topic_out = '/livox/lidar'

imu_topic_in = '/dji_osdk_ros/imu'
imu_topic_out = '/livox/imu'

# PointCloud2 fields for R3LIVE
fields = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
]

# Initialize CV Bridge
bridge = CvBridge()

def transform_image(msg):
    """
    Convert RGB images to BGR format.
    """
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")  # Read RGB
    bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)  # Convert to BGR
    bgr_msg = bridge.cv2_to_imgmsg(bgr_image, encoding="bgr8")
    bgr_msg.header = msg.header
    return bgr_msg

def transform_pointcloud(msg):
    """
    Strip 'ring' and 'time' fields, keeping only x, y, z, and intensity.
    """
    num_points = msg.width
    points = []

    for i in range(num_points):
        offset = i * msg.point_step
        x, y, z, intensity = struct.unpack_from('ffff', msg.data, offset)
        points.append([x, y, z, intensity])

    # Create new PointCloud2 message
    new_data = np.array(points, dtype=np.float32).tobytes()
    new_msg = PointCloud2(
        header=msg.header,
        height=1,
        width=len(points),
        is_dense=True,
        is_bigendian=False,
        fields=fields,
        point_step=16,
        row_step=16 * len(points),
        data=new_data
    )
    return new_msg

def transform_imu(msg):
    """
    Filter IMU data to retain only header, angular_velocity, and linear_acceleration.
    """
    imu_msg = Imu()
    imu_msg.header = msg.header
    imu_msg.angular_velocity = msg.angular_velocity
    imu_msg.linear_acceleration = msg.linear_acceleration
    return imu_msg

# Process the bag file
with rosbag.Bag(output_bag_path, 'w') as outbag:
    print("Processing bag file...")

    for topic, msg, t in rosbag.Bag(input_bag_path).read_messages():
        if topic == camera_topic_in:
            # Transform and write images
            transformed_image = transform_image(msg)
            outbag.write(camera_topic_out, transformed_image, t)

        elif topic == lidar_topic_in:
            # Transform and write PointCloud2
            transformed_lidar = transform_pointcloud(msg)
            outbag.write(lidar_topic_out, transformed_lidar, t)

        elif topic == imu_topic_in:
            # Transform and write IMU
            transformed_imu = transform_imu(msg)
            outbag.write(imu_topic_out, transformed_imu, t)

        else:
            # Copy other topics as is
            outbag.write(topic, msg, t)

    print(f"Transformation complete. Saved to {output_bag_path}")
