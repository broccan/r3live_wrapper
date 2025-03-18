#!/usr/bin/env python3

import rospy
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

def publish_static_transform():
    broadcaster = StaticTransformBroadcaster()
    static_transform = TransformStamped()

    static_transform.header.stamp = rospy.Time.now()
    static_transform.header.frame_id = "map"
    static_transform.child_frame_id = "velodyne"
    static_transform.transform.translation.x = 0.0
    static_transform.transform.translation.y = 0.0
    static_transform.transform.translation.z = 0.0
    static_transform.transform.rotation.x = 0.0
    static_transform.transform.rotation.y = 0.0
    static_transform.transform.rotation.z = 0.0
    static_transform.transform.rotation.w = 1.0

    broadcaster.sendTransform(static_transform)
    rospy.loginfo("Published static transform between map and velodyne")

if __name__ == "__main__":
    rospy.init_node("static_transform_publisher", anonymous=True)
    try:
        publish_static_transform()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Static Transform Publisher terminated.")
