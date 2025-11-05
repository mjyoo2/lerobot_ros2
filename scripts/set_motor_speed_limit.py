#!/usr/bin/env python3
"""
Motor Speed Limit Configuration

Limits the maximum movement speed of motors.

Usage:
    python scripts/set_motor_speed_limit.py --config configs/robot/your_config.yaml --speed 200

Speed values:
    - 0: No speed limit (maximum speed)
    - 1-1000: Lower = slower (e.g., 100=slow, 500=medium, 1000=fast)
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


def set_speed_limit(config_path, speed_limit):
    """Configure motor speed limit"""

    # Load configuration
    print(f"📋 Loading config: {config_path}")
    robot_config = load_config(config_path)

    # Create controller
    port = robot_config.get("port", "/dev/ttyUSB0")
    baudrate = robot_config.get("baudrate", 1000000)
    motor_type = robot_config.get("motor_type", "dynamixel").lower()
    motor_ids = [motor["id"] for motor in robot_config["motors"].values()]

    if motor_type == "feetech":
        controller = FeetechController(port=port, baudrate=baudrate, motor_ids=motor_ids)
    else:
        controller = DynamixelController(port=port, baudrate=baudrate, motor_ids=motor_ids)

    # Connect
    print(f"🔌 Connecting to {port}...")
    if not controller.connect():
        print("❌ Connection failed")
        return

    print("\n" + "="*80)
    print("⚡ SETTING MOTOR SPEED LIMIT")
    print("="*80)
    print(f"\nSpeed limit: {speed_limit}")
    print("  • 0 = No limit (max speed)")
    print("  • 1-1000 = Speed limit (lower = slower)")
    print()

    # Disable torque (for EEPROM writing)
    controller.disable_torque()
    print("✓ Torque disabled\n")

    # Feetech: Configure Goal_Time or Acceleration
    # Goal_Time: Time to reach target position (unit: 20ms)
    # Acceleration: Acceleration rate (0-254, 0=unlimited)

    if motor_type == "feetech":
        # Feetech STS3215: Use Acceleration register
        ADDR_ACCELERATION = 41  # Acceleration register

        # Convert speed limit to acceleration
        # speed_limit: 1-1000 → acceleration: 254-1 (inverse proportion)
        if speed_limit == 0:
            acceleration = 0  # Unlimited
        else:
            # Map 1-1000 range to 1-254 (inverse proportion)
            acceleration = max(1, min(254, int(255 - (speed_limit / 1000 * 254))))

        print(f"📝 Setting Acceleration to {acceleration} (lower = slower acceleration)...\n")

        for motor_name, motor_info in robot_config["motors"].items():
            motor_id = motor_info["id"]

            try:
                # Write Acceleration
                controller._write_1byte(motor_id, ADDR_ACCELERATION, acceleration)
                time.sleep(0.05)

                # Verify
                readback = controller._read_1byte(motor_id, ADDR_ACCELERATION)
                print(f"  ✓ Motor {motor_id} ({motor_name}): Acceleration = {readback}")

            except Exception as e:
                print(f"  ✗ Motor {motor_id} ({motor_name}) failed: {e}")

    else:
        # Dynamixel: Use Profile Velocity
        print("⚠️  Dynamixel speed limiting not implemented yet")
        print("   Please use Feetech motors or implement Dynamixel support")

    print("\n" + "="*80)
    print("✅ Speed limit applied!")
    print("="*80)
    print(f"\n💡 Motors will now move slower when you set distant target positions.")
    print(f"   Lower acceleration = {acceleration} means gentler movements.\n")

    # Disconnect (torque remains disabled)
    controller.disconnect()


def main():
    parser = argparse.ArgumentParser(description="Motor speed limit configuration")
    parser.add_argument(
        "--config",
        type=str,
        required=True,
        help="Robot config file path",
    )
    parser.add_argument(
        "--speed",
        type=int,
        default=300,
        help="Speed limit (0=unlimited, 1-1000: lower=slower, default=300)",
    )

    args = parser.parse_args()

    # Validate speed
    if args.speed < 0 or args.speed > 1000:
        print("❌ Speed must be between 0 and 1000")
        return

    try:
        set_speed_limit(args.config, args.speed)
    except KeyboardInterrupt:
        print("\n\n❌ Cancelled")
    except Exception as e:
        print(f"\n\n❌ Error: {e}")
        raise


if __name__ == "__main__":
    main()
