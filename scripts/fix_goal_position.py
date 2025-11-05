#!/usr/bin/env python3
"""
Goal Position Emergency Fix Script

Fixes the issue where motors move suddenly due to unsynchronized Goal Position after calibration.

Usage:
    python scripts/fix_goal_position.py --config configs/robot/your_config.yaml
"""

import argparse
import sys
import time
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from lerobot_ros2.configs import load_config
from lerobot_ros2.hardware import DynamixelController, FeetechController
from lerobot_ros2.hardware.calibration import CalibrationManager


def fix_goal_positions(config_path):
    """Synchronize Goal Position with Present Position"""

    # Load configuration
    print(f"📋 Loading config: {config_path}")
    robot_config = load_config(config_path)

    # Load calibration
    calibration_dir = Path(robot_config.get("calibration_dir", "./calibration"))
    manager = CalibrationManager(calibration_dir)

    try:
        calibrations = manager.load()
        print(f"✓ Calibration loaded: {len(calibrations)} motors\n")
    except FileNotFoundError:
        print("❌ No calibration file found!")
        return

    # Create controller
    port = robot_config.get("port", "/dev/ttyUSB0")
    baudrate = robot_config.get("baudrate", 1000000)
    motor_type = robot_config.get("motor_type", "dynamixel").lower()
    motor_ids = [motor["id"] for motor in robot_config["motors"].values()]

    # Convert calibration
    calibration_by_id = {}
    for motor_name, calib in calibrations.items():
        calibration_by_id[calib.motor_id] = calib

    if motor_type == "feetech":
        controller = FeetechController(port=port, baudrate=baudrate, motor_ids=motor_ids, calibration=calibration_by_id)
    else:
        controller = DynamixelController(port=port, baudrate=baudrate, motor_ids=motor_ids, calibration=calibration_by_id)

    # Connect
    print(f"🔌 Connecting to {port}...")
    if not controller.connect():
        print("❌ Connection failed")
        return

    print("\n" + "="*80)
    print("🛠️  FIXING GOAL POSITIONS")
    print("="*80)
    print("\n⚠️  This will set Goal Position = Present Position for all motors")
    print("   This prevents motors from jumping when torque is enabled.\n")

    input("Press Enter to continue (or Ctrl+C to cancel)...")

    # Disable torque (for safety)
    controller.disable_torque()
    print("\n✓ Torque disabled for safety")

    # Synchronize Goal Position
    print("\n📍 Syncing Goal Position with Present Position...")
    time.sleep(0.1)

    for motor_name, motor_info in robot_config["motors"].items():
        motor_id = motor_info["id"]

        try:
            # Read current Present Position
            if hasattr(controller, '_read_4byte'):
                present_pos = controller._read_4byte(motor_id, controller.ADDR_PRESENT_POSITION)
                goal_pos_old = controller._read_4byte(motor_id, controller.ADDR_GOAL_POSITION)
            else:
                present_pos = controller._read_2byte(motor_id, controller.ADDR_PRESENT_POSITION)
                goal_pos_old = controller._read_2byte(motor_id, controller.ADDR_GOAL_POSITION)

            # Show before state
            diff = abs(goal_pos_old - present_pos)
            if diff > 50:
                print(f"\n  ⚠️  Motor {motor_id} ({motor_name}):")
                print(f"      Present Position: {present_pos}")
                print(f"      Goal Position (old): {goal_pos_old}")
                print(f"      Difference: {diff} steps ← THIS IS THE PROBLEM!")
            else:
                print(f"\n  ✓ Motor {motor_id} ({motor_name}):")
                print(f"      Present: {present_pos}, Goal: {goal_pos_old} (diff: {diff})")

            # Set Goal Position to Present Position
            if hasattr(controller, '_write_4byte'):
                controller._write_4byte(motor_id, controller.ADDR_GOAL_POSITION, present_pos)
            else:
                controller._write_2byte(motor_id, controller.ADDR_GOAL_POSITION, present_pos)

            time.sleep(0.05)

            # Verify
            if hasattr(controller, '_read_4byte'):
                goal_pos_new = controller._read_4byte(motor_id, controller.ADDR_GOAL_POSITION)
            else:
                goal_pos_new = controller._read_2byte(motor_id, controller.ADDR_GOAL_POSITION)

            if abs(goal_pos_new - present_pos) < 5:
                print(f"      ✅ Goal Position updated: {goal_pos_new}")
            else:
                print(f"      ❌ Goal Position mismatch! Expected {present_pos}, got {goal_pos_new}")

        except Exception as e:
            print(f"  ✗ Motor {motor_id} ({motor_name}) failed: {e}")

    print("\n" + "="*80)
    print("✅ Goal Position sync complete!")
    print("="*80)
    print("\n💡 You can now use the GUI safely:")
    print("   python scripts/control_motors_gui.py --config", config_path)
    print("\n   When you enable torque, motors will NOT jump!")
    print()

    # Disconnect
    controller.disconnect()


def main():
    parser = argparse.ArgumentParser(description="Goal Position emergency fix")
    parser.add_argument(
        "--config",
        type=str,
        required=True,
        help="Robot config file path",
    )

    args = parser.parse_args()

    try:
        fix_goal_positions(args.config)
    except KeyboardInterrupt:
        print("\n\n❌ Cancelled")
    except Exception as e:
        print(f"\n\n❌ Error: {e}")
        raise


if __name__ == "__main__":
    main()
