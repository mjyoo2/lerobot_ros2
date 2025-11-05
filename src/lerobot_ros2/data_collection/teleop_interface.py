"""
TeleopInterface: Interface for teleoperation during data collection.

This module provides a bridge between LeRobot teleoperators and ROS2,
allowing leader arms, keyboards, or gamepads to control robots.
"""

import threading
from typing import Optional, Dict

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray

try:
    from lerobot.teleoperators.teleoperator import Teleoperator
    from lerobot.teleoperators.utils import make_teleoperator_from_config
except ImportError:
    raise ImportError(
        "LeRobot not found. Please install: pip install lerobot>=2.0.0"
    )


class TeleopInterface(Node):
    """
    ROS2 node that reads from LeRobot teleoperator and publishes actions.

    This node:
    - Reads actions from LeRobot teleoperator (leader arm, keyboard, gamepad)
    - Publishes actions to /joint_commands topic
    - Optionally reads current follower state for leader-follower synchronization

    Args:
        teleop_config: LeRobot teleoperator configuration
        node_name: Name of the ROS2 node (default: 'teleop_interface')
        publish_rate: Publishing frequency in Hz (default: 30)
        namespace: ROS2 namespace (default: empty string)
    """

    def __init__(
        self,
        teleop_config,
        node_name: str = "teleop_interface",
        publish_rate: float = 30.0,
        namespace: str = "",
    ):
        super().__init__(node_name, namespace=namespace)

        self.teleop_config = teleop_config
        self.publish_rate = publish_rate

        # LeRobot teleoperator instance
        self.teleoperator: Optional[Teleoperator] = None

        # State tracking
        self.current_action: Optional[np.ndarray] = None
        self.is_connected = False

        # Thread safety
        self.lock = threading.Lock()

        # QoS profile for real-time control
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self._setup_publishers()

        # Timer for reading teleoperator and publishing actions
        self.timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(self.timer_period, self.publish_callback)

        self.get_logger().info(f"TeleopInterface initialized (rate: {publish_rate} Hz)")

    def _setup_publishers(self):
        """Setup ROS2 publishers for action commands."""
        self.action_pub = self.create_publisher(
            Float64MultiArray,
            "/joint_commands",
            self.qos_profile,
        )

        self.get_logger().info("Action publisher configured")

    def connect(self):
        """
        Connect to LeRobot teleoperator.

        Initializes the teleoperator hardware (leader arm, keyboard, etc.).
        """
        try:
            self.get_logger().info("Connecting to LeRobot teleoperator...")

            # Create teleoperator from config
            self.teleoperator = make_teleoperator_from_config(self.teleop_config)

            # Connect to teleoperator hardware
            self.teleoperator.connect()

            self.is_connected = True
            self.get_logger().info("✅ Successfully connected to teleoperator")

            return True

        except Exception as e:
            self.get_logger().error(f"❌ Failed to connect to teleoperator: {e}")
            self.is_connected = False
            return False

    def disconnect(self):
        """Disconnect from LeRobot teleoperator."""
        if self.teleoperator and self.is_connected:
            try:
                self.get_logger().info("Disconnecting from teleoperator...")
                self.teleoperator.disconnect()
                self.is_connected = False
                self.get_logger().info("✅ Disconnected successfully")
            except Exception as e:
                self.get_logger().error(f"Error during disconnect: {e}")

    def publish_callback(self):
        """
        Timer callback to read from teleoperator and publish to ROS2.

        This runs at the specified publish_rate.
        """
        if not self.is_connected or self.teleoperator is None:
            return

        try:
            # Read action from teleoperator
            # Note: For leader-follower, you might need to pass current follower state
            # action = self.teleoperator.get_action(follower_state)
            # For now, we use simple get_action
            with self.lock:
                action = self.teleoperator.get_action()
                self.current_action = action

            # Publish action to ROS2
            if action is not None:
                action_msg = Float64MultiArray()
                action_msg.data = action.tolist()
                self.action_pub.publish(action_msg)

        except Exception as e:
            self.get_logger().error(f"Error in publish callback: {e}")

    def get_current_action(self) -> Optional[np.ndarray]:
        """
        Get the most recent action from teleoperator.

        Returns:
            Action array or None if not available
        """
        with self.lock:
            return self.current_action.copy() if self.current_action is not None else None

    def __del__(self):
        """Cleanup on destruction."""
        self.disconnect()


class LeaderFollowerInterface:
    """
    Simplified interface for leader-follower teleoperation.

    This class manages both leader and follower robots without ROS2,
    useful for direct hardware control during data collection.

    Args:
        leader_config: Configuration for leader robot
        follower_config: Configuration for follower robot
        control_rate: Control loop frequency in Hz (default: 30)
    """

    def __init__(
        self,
        leader_config,
        follower_config,
        control_rate: float = 30.0,
    ):
        from lerobot.robots.utils import make_robot_from_config

        self.leader_config = leader_config
        self.follower_config = follower_config
        self.control_rate = control_rate

        self.leader = None
        self.follower = None
        self.is_connected = False

        # For threaded operation
        self.control_thread = None
        self.running = False

    def connect(self):
        """Connect to both leader and follower robots."""
        try:
            from lerobot.robots.utils import make_robot_from_config

            print("Connecting to leader robot...")
            self.leader = make_robot_from_config(self.leader_config)
            self.leader.connect()

            print("Connecting to follower robot...")
            self.follower = make_robot_from_config(self.follower_config)
            self.follower.connect()

            self.is_connected = True
            print("✅ Successfully connected to leader and follower")
            return True

        except Exception as e:
            print(f"❌ Failed to connect: {e}")
            self.is_connected = False
            return False

    def disconnect(self):
        """Disconnect from both robots."""
        if self.leader:
            try:
                self.leader.disconnect()
            except Exception as e:
                print(f"Error disconnecting leader: {e}")

        if self.follower:
            try:
                self.follower.disconnect()
            except Exception as e:
                print(f"Error disconnecting follower: {e}")

        self.is_connected = False
        print("✅ Disconnected from leader and follower")

    def get_observations(self) -> Dict[str, np.ndarray]:
        """
        Get observations from both leader and follower.

        Returns:
            Dictionary with leader and follower observations
        """
        if not self.is_connected:
            raise RuntimeError("Not connected. Call connect() first.")

        leader_obs = self.leader.get_observation()
        follower_obs = self.follower.get_observation()

        return {
            "leader": leader_obs,
            "follower": follower_obs,
        }

    def send_leader_to_follower(self):
        """
        Read leader position and send to follower (one step).

        This is the core of leader-follower teleoperation.
        """
        if not self.is_connected:
            raise RuntimeError("Not connected. Call connect() first.")

        # Read leader position
        leader_obs = self.leader.get_observation()
        leader_state = leader_obs["observation.state"]

        # Send as action to follower
        action = {"action": leader_state}
        self.follower.send_action(action)

    def start_control_loop(self):
        """
        Start continuous leader-follower control loop in background thread.
        """
        if not self.is_connected:
            print("❌ Not connected. Call connect() first.")
            return False

        if self.running:
            print("❌ Control loop already running.")
            return False

        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.start()
        print("✅ Started leader-follower control loop")
        return True

    def stop_control_loop(self):
        """Stop the control loop."""
        if not self.running:
            print("❌ Control loop not running.")
            return False

        self.running = False
        if self.control_thread:
            self.control_thread.join()
        print("⏹️  Stopped control loop")
        return True

    def _control_loop(self):
        """Internal control loop thread."""
        import time

        loop_period = 1.0 / self.control_rate

        while self.running:
            start_time = time.time()

            try:
                self.send_leader_to_follower()
            except Exception as e:
                print(f"Error in control loop: {e}")

            # Sleep to maintain rate
            elapsed = time.time() - start_time
            sleep_time = loop_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def __del__(self):
        """Cleanup on destruction."""
        if self.running:
            self.stop_control_loop()
        self.disconnect()


def main():
    """Example usage of TeleopInterface."""
    import argparse
    from lerobot_ros2.configs import load_config

    parser = argparse.ArgumentParser(description="Run TeleopInterface ROS2 node")
    parser.add_argument(
        "--config",
        type=str,
        required=True,
        help="Path to teleoperator configuration YAML file",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=30.0,
        help="Publishing rate in Hz (default: 30)",
    )
    args = parser.parse_args()

    # Load teleoperator configuration
    teleop_config = load_config(args.config)

    # Initialize ROS2
    rclpy.init()

    # Create teleop interface node
    teleop_interface = TeleopInterface(teleop_config, publish_rate=args.rate)

    # Connect to teleoperator
    if not teleop_interface.connect():
        teleop_interface.get_logger().error("Failed to connect to teleoperator. Exiting.")
        rclpy.shutdown()
        return

    # Spin
    try:
        rclpy.spin(teleop_interface)
    except KeyboardInterrupt:
        teleop_interface.get_logger().info("Keyboard interrupt received")
    finally:
        teleop_interface.disconnect()
        teleop_interface.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
