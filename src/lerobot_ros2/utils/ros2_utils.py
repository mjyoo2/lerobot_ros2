"""
ROS2 utility functions
"""

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import rclpy
import time


def create_qos_profile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
):
    """
    Create QoS profile

    Args:
        reliability: Reliability policy
        durability: Durability policy
        history: History policy
        depth: Queue depth

    Returns:
        QoSProfile
    """
    return QoSProfile(
        reliability=reliability,
        durability=durability,
        history=history,
        depth=depth,
    )


def wait_for_topic(node, topic_name, topic_type, timeout_sec=5.0):
    """
    Wait until topic is published

    Args:
        node: ROS2 node
        topic_name: Topic name
        topic_type: Topic type
        timeout_sec: Timeout (seconds)

    Returns:
        bool: True if topic is found, False if timeout
    """
    start_time = time.time()

    while time.time() - start_time < timeout_sec:
        topic_names_and_types = node.get_topic_names_and_types()

        for name, types in topic_names_and_types:
            if name == topic_name:
                return True

        time.sleep(0.1)

    return False
