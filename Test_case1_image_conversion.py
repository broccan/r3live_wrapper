import pytest
from unittest.mock import MagicMock
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from convert_rgb_to_bgr import convert_and_publish, pub

def test_convert_and_publish():
    bridge = CvBridge()
    
    pub.publish = MagicMock()
    
    #Create a dummy RGB8 image input
    rgb_image = bridge.cv2_to_imgmsg([[255, 0, 0], [0, 255, 0]], encoding="rgb8")
    rgb_image.header.stamp = rospy.Time.now()
    
    convert_and_publish(rgb_image)
    
    assert pub.publish.called, "The publish method was not called!"
    
    published_msg = pub.publish.call_args[0][0]
    
    assert published_msg.encoding == "bgr8", "The image format is not BGR8!"
    
    assert published_msg.header == rgb_image.header, "The headers do not match!"
