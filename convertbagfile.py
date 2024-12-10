#!/usr/bin/env python3

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
import cv2

def process_bag(input_bag, output_bag):
    bridge = CvBridge()
    try:
        with rosbag.Bag(output_bag, 'w') as out_bag:
            with rosbag.Bag(input_bag, 'r') as in_bag:
                for topic, msg, t in in_bag.read_messages():
                    if topic == "/dji_osdk_ros/main_camera_images":
                        try:
                            # Convert to OpenCV and back to BGR8
                            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                            bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
                            bgr_msg = bridge.cv2_to_imgmsg(bgr_image, encoding="bgr8")
                            bgr_msg.header = msg.header
                            out_bag.write("/dji_osdk_ros/main_camera_images_bgr8", bgr_msg, t)
                        except CvBridgeError as e:
                            rospy.logerr(f"Image conversion failed: {e}")
                        except Exception as e:
                            rospy.logerr(f"Unexpected error: {e}")
                    else:
                        # Copy other topics as is
                        out_bag.write(topic, msg, t)
            rospy.loginfo(f"New bag file created at {output_bag}")
    except rosbag.ROSBagException as e:
        rospy.logerr(f"Error processing bag file: {e}")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")

if __name__ == "__main__":
    rospy.init_node("bag_processor", anonymous=True)
    input_bag_path = rospy.get_param("~input_bag", "snippet1.bag")
    output_bag_path = rospy.get_param("~output_bag", "snippet_with_bgr8.bag")
    process_bag(input_bag_path, output_bag_path)
