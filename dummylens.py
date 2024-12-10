#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class LensDistortionCorrector:
    def __init__(self, intrinsic_matrix, distortion_coeffs):
        self.bridge = CvBridge()
        self.intrinsic_matrix = np.array(intrinsic_matrix, dtype=np.float32)
        self.distortion_coeffs = np.array(distortion_coeffs, dtype=np.float32)

        # Compute undistort map
        self.new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
            self.intrinsic_matrix, self.distortion_coeffs, (640, 480), 1, (640, 480)
        )
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.intrinsic_matrix,
            self.distortion_coeffs,
            None,
            self.new_camera_matrix,
            (640, 480),
            cv2.CV_16SC2,
        )

        self.image_sub = rospy.Subscriber(
            "/dji_osdk_ros/main_camera_images_bgr8", Image, self.image_callback
        )
        self.image_pub = rospy.Publisher(
            "/dji_osdk_ros/main_camera_images_undistorted", Image, queue_size=10
        )

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Undistort the image
            undistorted_image = cv2.remap(cv_image, self.map1, self.map2, cv2.INTER_LINEAR)

            # Convert back to ROS Image message
            undistorted_msg = self.bridge.cv2_to_imgmsg(undistorted_image, encoding="bgr8")
            undistorted_msg.header = msg.header  # Preserve the original header
            self.image_pub.publish(undistorted_msg)
            rospy.loginfo("Published undistorted image")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
        except Exception as e:
            rospy.logerr(f"Unexpected error: {e}")

if __name__ == "__main__":
    rospy.init_node("lens_distortion_corrector", anonymous=True)

    # Example Intrinsics and Distortion Coefficients
    intrinsic_matrix = [
       
    ]
    distortion_coeffs = 
    try:
        LensDistortionCorrector(intrinsic_matrix, distortion_coeffs)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Lens Distortion Corrector Node terminated.")
