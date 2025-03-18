import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import signal
import sys

bridge = CvBridge()
last_message_time = None

def signal_handler(sig, frame):
    rospy.loginfo("Shutting down node")
    sys.exit(0)

def convert_and_publish(msg):
    global last_message_time
    last_message_time = rospy.Time.now()
    rospy.loginfo("Received an image message")
    try:
        # Convert rgb8 to bgr8
        bgr_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        bgr_msg = bridge.cv2_to_imgmsg(bgr_image, encoding="bgr8")
        bgr_msg.header = msg.header
        pub.publish(bgr_msg)
        rospy.loginfo("Published converted image")
    except CvBridgeError as e:
        rospy.logerr(f"Failed to convert image: {e}")

rospy.init_node("convert_rgb_to_bgr")
rospy.loginfo("Converting rgb8 images to bgr8")
pub = rospy.Publisher("/dji_osdk_ros/main_camera_images_bgr8", Image, queue_size=10)
rospy.Subscriber("/dji_osdk_ros/main_camera_images", Image, convert_and_publish)

# Handle Ctrl+C
signal.signal(signal.SIGINT, signal_handler)

# Timeout after 5 seconds of inactivity
while not rospy.is_shutdown():
    if last_message_time and (rospy.Time.now() - last_message_time > rospy.Duration(5)):
        rospy.loginfo("No new messages received for 5 seconds. Exiting.")
        break
    rospy.sleep(0.1)