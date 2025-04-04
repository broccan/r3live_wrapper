#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from livox_ros_driver.msg import CustomMsg, CustomPoint
import sensor_msgs.point_cloud2 as pc2

class PointCloudToLivoxConverterNode:
    def __init__(self):
        # Create a publisher for /livox/lidar
        self.pub = rospy.Publisher('/livox/lidar', CustomMsg, queue_size=10)

        # Subscribe to /velodyne_points
        self.sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.pointcloud_callback)

    def pointcloud2_to_custommsg(self, cloud_msg):
        """
        Convert a sensor_msgs/PointCloud2 message to a livox_ros_driver/CustomMsg message.
        """
        custom_msg = CustomMsg()
        custom_msg.header = cloud_msg.header  # Copy the header
        custom_msg.timebase = 0  # Set timebase (adjust as needed)
        custom_msg.point_num = 0  # Initialize point count

        # Iterate through the points in the PointCloud2 message
        for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            custom_point = CustomPoint()
            custom_point.x = point[0]
            custom_point.y = point[1]
            custom_point.z = point[2]
            custom_point.reflectivity = int(point[3])  # Use intensity as reflectivity
            custom_point.tag = 0  # Set tag 
            custom_point.line = 0  # Set line

            # Add the point to the CustomMsg
            custom_msg.points.append(custom_point)
            custom_msg.point_num += 1

        return custom_msg

    def pointcloud_callback(self, cloud_msg):
        """
        Callback function for the /velodyne_points topic.
        """
        try:
            # Convert PointCloud2 to CustomMsg
            custom_msg = self.pointcloud2_to_custommsg(cloud_msg)

            # Publish the CustomMsg to /livox/lidar
            self.pub.publish(custom_msg)
        except Exception as e:
            rospy.logerr(f"Error converting PointCloud2 to CustomMsg: {e}")

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('pointcloud_transfer_node')

    # Create an instance of the PointCloudToLivoxConverterNode class
    converter = PointCloudToLivoxConverterNode()

    # Keep the node running
    rospy.spin()