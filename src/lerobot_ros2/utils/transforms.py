"""
Transform utility functions
"""

import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# CV Bridge instance (lazy initialization)
_cv_bridge = None


def _get_cv_bridge():
    """Get CV Bridge instance (lazy initialization)"""
    global _cv_bridge
    if _cv_bridge is None:
        _cv_bridge = CvBridge()
    return _cv_bridge


def cv_to_ros_image(cv_image, encoding="rgb8"):
    """
    Convert OpenCV image to ROS Image message

    Args:
        cv_image: OpenCV image (numpy array)
        encoding: Encoding (Default: rgb8)

    Returns:
        sensor_msgs/Image
    """
    bridge = _get_cv_bridge()
    return bridge.cv2_to_imgmsg(cv_image, encoding=encoding)


def ros_to_cv_image(ros_image, encoding="rgb8"):
    """
    Convert ROS Image message to OpenCV image

    Args:
        ros_image: sensor_msgs/Image
        encoding: Encoding (Default: rgb8)

    Returns:
        numpy array
    """
    bridge = _get_cv_bridge()
    return bridge.imgmsg_to_cv2(ros_image, desired_encoding=encoding)
