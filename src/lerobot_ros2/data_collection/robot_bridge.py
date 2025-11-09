"""
RobotBridge: Native hardware control and ROS2 connection

Bridges ROS2 with DynamixelController and CameraController.
Operates independently without LeRobot dependencies.
"""

import threading
from typing import Dict, Optional
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from lerobot_ros2.hardware import DynamixelController, FeetechController, CameraController
from lerobot_ros2.hardware.calibration import CalibrationManager
from lerobot_ros2.utils.safety import SafetyChecker
from pathlib import Path

# URDF joint limits (from original SO-101 URDF)
URDF_LIMITS = {
    "shoulder_pan": 1.91986,
    "shoulder_lift": 1.74533,
    "elbow_flex": 1.69,
    "wrist_flex": 1.65806,
    "wrist_roll": 2.74385,
    "gripper": 1.155855,  # Corrected from URDF (was incorrectly set to shoulder_lift value)
}


class RobotBridge(Node):
    """
    ROS2 node: Bridges native hardware control to ROS2 topics

    Data flow:
    1. Hardware → ROS2 (publish sensor data)
       - DynamixelController.read_positions() → /joint_states
       - CameraController.read_images() → /camera/{name}/image_raw

    2. ROS2 → Hardware (receive control commands)
       - /joint_commands → DynamixelController.write_positions()
       - /joint_trajectory (MoveIt) → DynamixelController.write_positions()

    Args:
        robot_config: Robot configuration (loaded from YAML)
        node_name: ROS2 node name
        publish_rate: Publishing frequency (Hz)
        namespace: ROS2 namespace
    """

    def __init__(
        self,
        robot_config: Dict,
        node_name: str = "robot_bridge",
        publish_rate: float = 30.0,
        namespace: str = "",
        enable_torque: bool = True,
    ):
        super().__init__(node_name, namespace=namespace)

        self.robot_config = robot_config
        self.publish_rate = publish_rate
        self.bridge = None  # Lazy initialization
        self.enable_torque_on_connect = enable_torque

        # Joint names (URDF compatible)
        self.joint_names = self._get_joint_names()

        # Hardware controllers
        # Type is DynamixelController or FeetechController
        self.motor_controller = None
        self.camera_controller: Optional[CameraController] = None

        # State tracking
        self.current_observation: Optional[Dict[str, np.ndarray]] = None
        self.last_action: Optional[Dict[str, np.ndarray]] = None
        self.is_connected = False

        # Thread safety
        self.lock = threading.Lock()

        # Safety checker (initialized in connect())
        self.safety_checker: Optional[SafetyChecker] = None

        # QoS profile
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self._setup_publishers()

        # Subscribers
        self._setup_subscribers()

        # Timer for publishing sensor data
        self.timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(self.timer_period, self.publish_callback)

        self.get_logger().info(f"RobotBridge Initialization complete (rate: {publish_rate} Hz)")

    def _setup_publishers(self):
        """ROS2 Publisher configuration"""
        # Joint states
        self.joint_state_pub = self.create_publisher(
            JointState,
            "/joint_states",
            self.qos_profile,
        )

        # Camera images (created dynamically after camera connection)
        self.image_pubs: Dict[str, any] = {}

        self.get_logger().info("Publishers configured")

    def _setup_subscribers(self):
        """ROS2 Subscriber configuration"""
        # Simple position commands
        self.action_sub = self.create_subscription(
            Float64MultiArray,
            "/joint_commands",
            self.action_callback,
            self.qos_profile,
        )

        # MoveIt trajectory commands
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            self.trajectory_callback,
            self.qos_profile,
        )

        self.get_logger().info("Subscribers configured")

    def connect(self) -> bool:
        """Connect to hardware"""
        try:
            self.get_logger().info("Connecting to hardware...")

            # Load calibration
            calibration_by_id = {}
            calibration_dir = Path(self.robot_config.get("calibration_dir", "./calibration"))
            if calibration_dir.exists():
                try:
                    manager = CalibrationManager(calibration_dir)
                    calibrations = manager.load()

                    # Map motor_name -> motor_id
                    for motor_name, calib in calibrations.items():
                        calibration_by_id[calib.motor_id] = calib

                    self.get_logger().info(f"✓ Calibration loaded: {len(calibrations)} motors")
                except Exception as e:
                    self.get_logger().warning(f"⚠️  Calibration load failed: {e}")
            else:
                self.get_logger().warning(f"⚠️  No calibration directory: {calibration_dir}")

            # Initialize motor controller
            motor_config = self._parse_motor_config(self.robot_config)
            motor_type = self.robot_config.get("motor_type", "dynamixel").lower()

            if motor_type == "feetech":
                self.get_logger().info("Using Feetech controller")
                self.motor_controller = FeetechController(
                    port=motor_config["port"],
                    baudrate=motor_config["baudrate"],
                    motor_ids=motor_config["motor_ids"],
                    calibration=calibration_by_id,
                )
            else:  # dynamixel
                self.get_logger().info("Using Dynamixel controller")
                self.motor_controller = DynamixelController(
                    port=motor_config["port"],
                    baudrate=motor_config["baudrate"],
                    motor_ids=motor_config["motor_ids"],
                    calibration=calibration_by_id,
                )

            try:
                if not self.motor_controller.connect():
                    raise Exception("motor connection failed")
            except Exception as conn_err:
                import traceback
                self.get_logger().error(f"Motor connect() failed: {conn_err}")
                self.get_logger().error(f"Traceback:\n{traceback.format_exc()}")
                raise

            # Log mapping verification
            self.get_logger().info("=" * 80)
            self.get_logger().info("🔍 Joint-Motor mapping verification:")
            for i, (joint_name, motor_id) in enumerate(zip(self.joint_names, motor_config["motor_ids"])):
                self.get_logger().info(f"  [{i}] joint_names[{i}] = '{joint_name}' → motor_ids[{i}] = {motor_id}")
            self.get_logger().info("=" * 80)

            # Enable torque (optional)
            if self.enable_torque_on_connect:
                self.get_logger().info("Enabling motor torque...")
                self.motor_controller.enable_torque()
                self.get_logger().info("✓ Torque enabled")
            else:
                self.get_logger().info("⚠️  Torque disabled - motors can be moved by hand")
                self.get_logger().info("   (Commands will be ignored)")

            # Initialize camera controller (if cameras configured)
            if "cameras" in self.robot_config:
                camera_configs = self.robot_config["cameras"]
                self.camera_controller = CameraController(camera_configs)

                if not self.camera_controller.connect():
                    self.get_logger().warning("Camera connection failed (continuing)")
                    self.camera_controller = None
                else:
                    # Create publishers for each camera
                    for camera_name in camera_configs.keys():
                        topic_name = f"/camera/{camera_name}/image_raw"
                        self.image_pubs[camera_name] = self.create_publisher(
                            Image,
                            topic_name,
                            self.qos_profile,
                        )
                        self.get_logger().info(f"Camera topic created: {topic_name}")

            # Initialize safety checker
            position_limits = {}
            if calibration_by_id:
                # Extract position limits from calibration data
                for motor_id, calib in calibration_by_id.items():
                    # Convert Present_position range to normalized range
                    center = (calib.range_min + calib.range_max) / 2
                    range_half = (calib.range_max - calib.range_min) / 2
                    # Normalized range: -100 ~ 100
                    position_limits[motor_id] = (-100.0, 100.0)

            self.safety_checker = SafetyChecker(
                position_limits=position_limits,
                max_position_delta=50.0,  # Normalized units per command
            )
            self.get_logger().info(f"✓ Safety checker initialized: {len(position_limits)} motors")

            self.is_connected = True
            self.get_logger().info("✓ Hardware connected successfully")
            return True

        except Exception as e:
            self.get_logger().error(f"✗ Hardware connection failed: {e}")
            self.is_connected = False
            return False

    def _get_joint_names(self) -> list:
        """Extract joint names from configuration (URDF compatible)"""
        # Method 1: Use joint_names list if available
        if "joint_names" in self.robot_config:
            return self.robot_config["joint_names"]

        # Method 2: Auto-detect SO-101 robot (6 motors)
        if "motors" in self.robot_config and len(self.robot_config["motors"]) == 6:
            # Default SO-101 joint names (matches URDF)
            self.get_logger().info("🤖 Auto-detected SO-101 robot, using standard joint names")
            return [
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "wrist_flex",
                "wrist_roll",
                "gripper"
            ]

        # Method 3: Extract joint_name from motors
        if "motors" in self.robot_config:
            joint_names = []
            for motor_name in sorted(self.robot_config["motors"].keys()):
                motor_info = self.robot_config["motors"][motor_name]
                if "joint_name" in motor_info:
                    joint_names.append(motor_info["joint_name"])
                else:
                    # Fallback: motor_1 → joint_1
                    joint_names.append(motor_name.replace("motor_", "joint_"))
            return joint_names

        # Method 4: Fallback - generic joint names
        return [f"joint_{i+1}" for i in range(6)]

    def _parse_motor_config(self, config: Dict) -> Dict:
        """Extract motor info from configuration"""
        port = config.get("port", "/dev/ttyUSB0")
        baudrate = config.get("baudrate", 1000000)

        # Extract motor ID list
        motor_ids = []
        if "motors" in config:
            # Check if motors have joint_name field (for custom mapping)
            has_joint_name_field = any(
                "joint_name" in motor_info
                for motor_info in config["motors"].values()
            )

            if has_joint_name_field:
                # Use joint_name field to map motors
                for joint_name in self.joint_names:
                    for motor_name, motor_info in config["motors"].items():
                        if motor_info.get("joint_name") == joint_name:
                            motor_ids.append(motor_info["id"])
                            break
            else:
                # Default: use sorted motor name order (motor_1, motor_2, ...)
                # This matches the auto-detected joint_names order
                for motor_name in sorted(config["motors"].keys()):
                    motor_info = config["motors"][motor_name]
                    motor_ids.append(motor_info["id"])

        return {
            "port": port,
            "baudrate": baudrate,
            "motor_ids": motor_ids,
        }

    def disconnect(self):
        """Disconnect from hardware"""
        if not self.is_connected:
            return

        try:
            self.get_logger().info("Disconnecting from hardware...")

            if self.motor_controller:
                self.motor_controller.disconnect()

            if self.camera_controller:
                self.camera_controller.disconnect()

            self.is_connected = False
            self.get_logger().info("✓ Hardware disconnected")

        except Exception as e:
            self.get_logger().error(f"Error during disconnect: {e}")

    def publish_callback(self):
        """
        Timer callback: Read data from hardware and publish to ROS2
        """
        if not self.is_connected:
            return

        try:
            with self.lock:
                # Read motor positions
                obs = {}

                if self.motor_controller:
                    motor_obs = self.motor_controller.get_observation()
                    obs.update(motor_obs)

                # Read camera images
                if self.camera_controller:
                    camera_obs = self.camera_controller.get_observation()
                    obs.update(camera_obs)

                self.current_observation = obs

            # Timestamp
            timestamp = self.get_clock().now().to_msg()

            # Publish joint states
            self._publish_joint_states(obs, timestamp)

            # Publish camera images
            self._publish_images(obs, timestamp)

        except Exception as e:
            self.get_logger().error(f"Publish callback error: {e}")

    def _publish_joint_states(self, obs: Dict[str, np.ndarray], timestamp):
        """Publish joint states (ROS2 uses radians)"""
        if "observation.state" not in obs:
            return

        joint_msg = JointState()
        joint_msg.header.stamp = timestamp
        joint_msg.header.frame_id = "base_link"

        # observation.state is normalized (-100~100)
        # ROS2 joint_states requires radians
        normalized_positions = obs["observation.state"]
        radians_positions = self._normalized_to_radians(normalized_positions)

        # Debug: Log every 30th message (1Hz at 30Hz rate)
        if not hasattr(self, '_publish_counter'):
            self._publish_counter = 0
        self._publish_counter += 1
        if self._publish_counter % 30 == 0:
            self.get_logger().info(f"📤 Publishing /joint_states (ALL 6):")
            for i, (joint_name, norm, rad) in enumerate(zip(self.joint_names, normalized_positions, radians_positions)):
                self.get_logger().info(f"  [{i}] {joint_name}: norm={norm:.2f}, rad={rad:.4f}")

        joint_msg.position = radians_positions.tolist()

        # Joint names (from config file, URDF compatible)
        joint_msg.name = self.joint_names[:len(radians_positions)]

        self.joint_state_pub.publish(joint_msg)

    def _publish_images(self, obs: Dict[str, np.ndarray], timestamp):
        """Publish camera images"""
        for key, value in obs.items():
            if "observation.images" in key:
                # Extract camera name (e.g., "observation.images.top" -> "top")
                camera_name = key.split(".")[-1]

                if camera_name in self.image_pubs:
                    try:
                        # Lazy initialization of CvBridge
                        if self.bridge is None:
                            self.bridge = CvBridge()

                        # numpy array -> ROS Image
                        img_msg = self.bridge.cv2_to_imgmsg(value, encoding="rgb8")
                        img_msg.header.stamp = timestamp
                        img_msg.header.frame_id = f"{camera_name}_camera"

                        self.image_pubs[camera_name].publish(img_msg)

                    except Exception as e:
                        self.get_logger().error(f"Image publish error ({camera_name}): {e}")

    def action_callback(self, msg: Float64MultiArray):
        """
        Receive control commands from ROS2

        Args:
            msg: Target joint positions (radians or normalized -100~100)

        Note:
            Automatically detects radians vs normalized format.
            - If all values in [-10, 10] range, treats as radians and converts
            - Otherwise treats as normalized and uses directly
            - Large position changes are automatically split into multiple steps for smooth motion
        """
        if not self.is_connected or not self.motor_controller:
            return

        try:
            # ROS message -> action dict
            positions = np.array(msg.data, dtype=np.float32)

            self.get_logger().info("=" * 80)
            self.get_logger().info("📥 /joint_commands received:")
            self.get_logger().info(f"   Raw data: {[f'{p:.4f}' for p in positions]}")

            # Auto-detect format: radians vs normalized
            # Radians are typically in range [-π, π] ≈ [-3.14, 3.14]
            # Normalized are in range [-100, 100]
            is_radians = np.all(np.abs(positions) <= 10.0)
            self.get_logger().info(f"   Format detected: {'RADIANS' if is_radians else 'NORMALIZED'}")

            if is_radians:
                # Convert radians to normalized (same as trajectory_callback)
                self.get_logger().info(f"   Converting radians → normalized...")
                for i, (joint_name, rad_val) in enumerate(zip(self.joint_names[:len(positions)], positions)):
                    self.get_logger().info(f"     [{i}] {joint_name}: {rad_val:.4f} rad")
                positions = self._radians_to_normalized(positions)
                self.get_logger().info(f"   After conversion (normalized): {[f'{p:.2f}' for p in positions]}")

            # Clamp to position limits first
            if self.safety_checker:
                positions = self.safety_checker.clamp_to_limits(positions)

            # Check if we need to interpolate (split large movements into multiple steps)
            current_pos = self.current_observation.get("observation.state") if self.current_observation else None
            if current_pos is not None and self.safety_checker:
                # Calculate max delta
                deltas = np.abs(positions - current_pos[:len(positions)])
                max_delta = np.max(deltas)
                max_allowed = self.safety_checker.max_position_delta

                if max_delta > max_allowed:
                    # Automatically interpolate
                    num_steps = int(np.ceil(max_delta / max_allowed)) + 1
                    self.get_logger().info(f"🔄 Large movement detected (delta={max_delta:.1f})")
                    self.get_logger().info(f"   Splitting into {num_steps} steps for smooth motion...")

                    # Interpolate: current_pos → positions in num_steps
                    for step in range(1, num_steps + 1):
                        alpha = step / num_steps
                        intermediate_pos = current_pos[:len(positions)] * (1 - alpha) + positions * alpha

                        self.get_logger().info(f"   Step {step}/{num_steps}: alpha={alpha:.2f}")

                        action = {"action": intermediate_pos}
                        with self.lock:
                            self.motor_controller.send_action(action)
                            self.last_action = action

                        # Small delay between steps
                        import time
                        time.sleep(0.1)

                    self.get_logger().info("✅ Interpolated movement complete")
                    self.get_logger().info("=" * 80)
                    return

            # Normal case: single step movement
            self.get_logger().info(f"📤 Sending to motors (normalized):")
            for i, (joint_name, norm_val) in enumerate(zip(self.joint_names[:len(positions)], positions)):
                motor_id = self.motor_controller.motor_ids[i] if i < len(self.motor_controller.motor_ids) else "?"
                self.get_logger().info(f"     [{i}] {joint_name} (motor {motor_id}): {norm_val:.2f}")
            self.get_logger().info("=" * 80)

            action = {
                "action": positions
            }

            with self.lock:
                # Send command to motors
                self.motor_controller.send_action(action)
                self.last_action = action

        except Exception as e:
            self.get_logger().error(f"Action callback error: {e}")

    def trajectory_callback(self, msg: JointTrajectory):
        """
        Receive MoveIt trajectory commands

        Args:
            msg: JointTrajectory (trajectory generated by MoveIt)

        Note:
            Currently only uses the last point of the trajectory as target (simple implementation).
            Following the entire trajectory requires trajectory execution.
        """
        if not self.is_connected or not self.motor_controller:
            return

        try:
            if not msg.points:
                self.get_logger().warning("Empty trajectory received")
                return

            # Use last point as target (final destination)
            last_point = msg.points[-1]

            # Map positions according to joint name order
            # MoveIt's joint_names order may differ from ours, so we need mapping
            positions = np.zeros(len(self.joint_names), dtype=np.float32)

            self.get_logger().info("📥 Trajectory received:")
            self.get_logger().info(f"   MoveIt joint order: {msg.joint_names}")
            self.get_logger().info(f"   MoveIt positions: {[f'{p:.3f}' for p in last_point.positions]}")

            for i, joint_name in enumerate(msg.joint_names):
                if joint_name in self.joint_names:
                    # Find index in our joint order
                    our_index = self.joint_names.index(joint_name)
                    positions[our_index] = last_point.positions[i]
                    self.get_logger().info(f"   Mapping: {joint_name} (MoveIt[{i}]={last_point.positions[i]:.3f}) → our[{our_index}]")
                else:
                    self.get_logger().warning(f"Unknown joint: {joint_name}")

            self.get_logger().info(f"   Our joint order: {self.joint_names}")
            self.get_logger().info(f"   Our positions (radians): {[f'{p:.3f}' for p in positions]}")

            # Convert radians to Present_Position to Normalized
            #
            # When using calibration:
            #   1. radians → present_position: position * (4096 / 2π)
            #   2. present_position → normalized: motor_controller handles automatically
            #
            # Without calibration (fallback):
            #   Use URDF default limits (-1.75 ~ 1.75 radians)
            #   normalized = (radians / 1.75) * 100

            positions_normalized = self._radians_to_normalized(positions)
            self.get_logger().info(f"   Normalized positions: {[f'{p:.2f}' for p in positions_normalized]}")

            # Safety check
            if self.safety_checker:
                is_safe, error_msg = self.safety_checker.validate_command(positions_normalized)
                if not is_safe:
                    self.get_logger().warning(f"⚠️  Trajectory command rejected: {error_msg}")
                    self.get_logger().warning(f"   Radians: {positions.tolist()}")
                    self.get_logger().warning(f"   Normalized: {positions_normalized.tolist()}")
                    return

                # Clamp to safe limits
                positions_normalized = self.safety_checker.clamp_to_limits(positions_normalized)

            action = {
                "action": positions_normalized
            }

            self.get_logger().info("📤 Sending to motors:")
            for i, (joint_name, norm_pos) in enumerate(zip(self.joint_names, positions_normalized)):
                motor_id = self.motor_controller.motor_ids[i]
                self.get_logger().info(f"   [{i}] {joint_name} → motor_id={motor_id}, normalized={norm_pos:.2f}")

            with self.lock:
                self.motor_controller.send_action(action)
                self.last_action = action

            self.get_logger().info("✅ Trajectory executed")

        except Exception as e:
            self.get_logger().error(f"Trajectory callback error: {e}")

    def _normalized_to_radians(self, normalized: np.ndarray) -> np.ndarray:
        """
        Convert normalized coordinates to radians (based on URDF limits)

        Simple linear transformation: normalized (-100~100) → radians (URDF limits)

        Args:
            normalized: Normalized positions (-100~100)

        Returns:
            Joint positions in radians
        """
        radians = np.zeros_like(normalized, dtype=np.float32)

        for idx, joint_name in enumerate(self.joint_names[:len(normalized)]):
            urdf_limit = URDF_LIMITS.get(joint_name, 1.75)  # Fallback
            radians[idx] = normalized[idx] / 100.0 * urdf_limit

        return radians

    def _radians_to_normalized(self, radians: np.ndarray) -> np.ndarray:
        """
        Convert radians to normalized coordinates (based on URDF limits)

        Simple linear transformation: radians (URDF limits) → normalized (-100~100)

        Args:
            radians: Joint positions in radians

        Returns:
            Normalized positions (-100~100)
        """
        normalized = np.zeros_like(radians, dtype=np.float32)

        for idx, joint_name in enumerate(self.joint_names[:len(radians)]):
            urdf_limit = URDF_LIMITS.get(joint_name, 1.75)  # Fallback
            normalized[idx] = radians[idx] / urdf_limit * 100.0

        return normalized

    def get_current_observation(self) -> Optional[Dict[str, np.ndarray]]:
        """Return current observation"""
        with self.lock:
            return self.current_observation.copy() if self.current_observation else None

    def check_hardware(self) -> bool:
        """Check hardware status"""
        if not self.is_connected or not self.motor_controller:
            return False

        try:
            obs = self.motor_controller.get_observation()
            return obs is not None
        except Exception as e:
            self.get_logger().error(f"Hardware check failed: {e}")
            return False

    def __del__(self):
        """Destructor"""
        self.disconnect()


def main():
    """Test main function"""
    import argparse
    from lerobot_ros2.configs import load_config

    parser = argparse.ArgumentParser(description="Run RobotBridge V2")
    parser.add_argument(
        "--config",
        type=str,
        required=True,
        help="Robot config file path",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=30.0,
        help="Publishing frequency (Hz)",
    )
    parser.add_argument(
        "--no-torque",
        action="store_true",
        help="Disable motor torque (motors can be moved by hand, teleop mode)",
    )
    args = parser.parse_args()

    # Load configuration
    robot_config = load_config(args.config)

    # ROS2 initialization
    rclpy.init()

    # Create Bridge node
    bridge = RobotBridge(
        robot_config,
        publish_rate=args.rate,
        enable_torque=not args.no_torque  # Invert flag
    )

    # Connect to hardware
    if not bridge.connect():
        bridge.get_logger().error("Hardware connection failed. Terminating.")
        rclpy.shutdown()
        return

    # Spin
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        bridge.get_logger().info("User interrupted")
    finally:
        bridge.disconnect()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
