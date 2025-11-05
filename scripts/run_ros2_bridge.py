#!/usr/bin/env python3
"""
ROS2 Motor Bridge Execution Script

Enables motor control via ROS2 topics.

Usage:
    python scripts/run_ros2_bridge.py --config configs/robot/my_robot.yaml
    python scripts/run_ros2_bridge.py --config configs/robot/my_robot.yaml --rate 50

ROS2 Topics:
    Published:
        /joint_states (sensor_msgs/JointState) - Current motor positions

    Subscribed:
        /joint_commands (std_msgs/Float64MultiArray) - Target motor positions (normalized -100~100)

Example:
    # Terminal 1: Run bridge
    python scripts/run_ros2_bridge.py --config configs/robot/my_robot.yaml

    # Terminal 2: Monitor current positions
    ros2 topic echo /joint_states

    # Terminal 3: Control motors (set all motors to 0)
    ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

    # Terminal 4: Move specific motors
    ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray "data: [50.0, 0.0, -30.0, 0.0, 0.0, 0.0]"
"""

import argparse
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import rclpy
from lerobot_ros2.configs import load_config
from lerobot_ros2.data_collection.robot_bridge import RobotBridge


def main():
    parser = argparse.ArgumentParser(
        description="ROS2 Motor Bridge - Control motors via ROS2",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
  # Run bridge
  python scripts/run_ros2_bridge.py --config configs/robot/my_robot.yaml

  # In another terminal:
  # View current positions
  ros2 topic echo /joint_states

  # Move all motors to neutral position (0)
  ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

  # Move specific motors
  ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray "data: [50.0, -20.0, 0.0, 0.0, 0.0, 0.0]"

Coordinate System:
  - Positions use normalized values: -100 ~ +100
  - 0 = calibration neutral position
  - ±100 = calibration min/max range
        """
    )
    parser.add_argument(
        "--config",
        type=str,
        required=True,
        help="Robot config file path (e.g., configs/robot/my_robot.yaml)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=30.0,
        help="Publishing frequency Hz (Default: 30.0)",
    )

    args = parser.parse_args()

    # Check config file
    config_path = Path(args.config)
    if not config_path.exists():
        print(f"❌ Config file not found: {args.config}")
        return 1

    # Load configuration
    print(f"📋 Loading config: {args.config}")
    try:
        robot_config = load_config(args.config)
    except Exception as e:
        print(f"❌ Failed to load config: {e}")
        return 1

    # Check calibration
    calibration_dir = Path(robot_config.get("calibration_dir", "./calibration"))
    if not calibration_dir.exists():
        print(f"\n⚠️  WARNING: No calibration directory found: {calibration_dir}")
        print("   Run calibration first:")
        print(f"   python scripts/calibrate_motors.py --config {args.config}")
        print(f"   python scripts/fix_goal_position.py --config {args.config}")
        print()
        response = input("Continue without calibration? (y/N): ")
        if response.lower() != 'y':
            return 1

    print("\n" + "="*80)
    print("🚀 ROS2 Motor Bridge")
    print("="*80)
    print(f"Config: {args.config}")
    print(f"Rate: {args.rate} Hz")
    print(f"Calibration: {calibration_dir}")
    print()
    print("ROS2 Topics:")
    print("  📤 Publishing:")
    print("     /joint_states (sensor_msgs/JointState)")
    print("  📥 Subscribing:")
    print("     /joint_commands (std_msgs/Float64MultiArray)")
    print()
    print("💡 Use Ctrl+C to stop")
    print("="*80)
    print()

    # ROS2 initialization
    rclpy.init()

    # Create bridge node
    bridge = RobotBridge(robot_config, publish_rate=args.rate)

    # Connect to hardware
    print("🔌 Connecting to hardware...")
    if not bridge.connect():
        bridge.get_logger().error("❌ Hardware connection failed")
        rclpy.shutdown()
        return 1

    print()
    print("="*80)
    print("✅ ROS2 Bridge is running!")
    print("="*80)
    print()
    print("Try these commands in another terminal:")
    print()
    print("  # Monitor current position")
    print("  ros2 topic echo /joint_states")
    print()
    print("  # Move all motors to center (0)")
    num_motors = len(robot_config.get("motors", {}))
    zero_command = ", ".join(["0.0"] * num_motors)
    print(f"  ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray \"data: [{zero_command}]\"")
    print()
    print("  # Move specific motors")
    print(f"  ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray \"data: [50.0, {', '.join(['0.0'] * (num_motors - 1))}]\"")
    print()
    print("="*80)
    print()

    # Spin
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        print("\n")
        print("="*80)
        print("🛑 Shutting down...")
        print("="*80)
    finally:
        bridge.disconnect()
        bridge.destroy_node()
        rclpy.shutdown()
        print("✓ ROS2 Bridge stopped")

    return 0


if __name__ == "__main__":
    sys.exit(main())
