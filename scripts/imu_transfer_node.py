#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

class ImuTransferNode:
    def __init__(self):
        # Create a publisher for /livox/imu
        self.pub = rospy.Publisher('/livox/imu', Imu, queue_size=10)

        # Subscribe to /dji_osdk_ros/imu
        self.sub = rospy.Subscriber('/dji_osdk_ros/imu', Imu, self.imu_callback)

    def imu_callback(self, imu_msg):
        """
        Callback function for the /dji_osdk_ros/imu topic.
        """
        try:
            # Publish the IMU message to /livox/imu
            self.pub.publish(imu_msg)
        except Exception as e:
            rospy.logerr(f"Error republishing IMU message: {e}")

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('imu_transfer_node')

    # Create an instance of the ImuTransferNode class
    imu_transfer = ImuTransferNode()

    # Keep the node running
    rospy.spin()