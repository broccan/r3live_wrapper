#!/usr/bin/env python3

import rospy
from livox_ros_driver.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField
import struct

def livox_callback(msg):
    # Create a PointCloud2 message
    cloud = PointCloud2()
    cloud.header = msg.header
    cloud.height = 1
    cloud.width = len(msg.points)
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = cloud.point_step * cloud.width
    cloud.is_dense = True

    # Convert CustomMsg points to PointCloud2 data
    cloud.data = []
    for point in msg.points:
        cloud.data += struct.pack('fff', point.x, point.y, point.z)

    # Publish the PointCloud2 message
    pub.publish(cloud)

if __name__ == "__main__":
    rospy.init_node("livox_to_pointcloud")
    pub = rospy.Publisher("/livox/pointcloud", PointCloud2, queue_size=10)
    rospy.Subscriber("/livox/lidar", CustomMsg, livox_callback)
    rospy.spin()