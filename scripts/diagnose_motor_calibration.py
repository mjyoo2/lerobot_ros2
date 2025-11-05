#!/usr/bin/env python3
"""
Motor Calibration Diagnostic Script

Diagnoses causes of sudden movements when torque is enabled.

Usage:
    python scripts/diagnose_motor_calibration.py --config configs/robot/your_config.yaml
"""

import argparse
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from lerobot_ros2.configs import load_config
from lerobot_ros2.hardware import DynamixelController, FeetechController
from lerobot_ros2.hardware.calibration import CalibrationManager


def diagnose(config_path):
    """Diagnose calibration status"""

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
        print("   Run: python scripts/calibrate_motors.py --config <config.yaml>")
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

    print("="*80)
    print("📊 MOTOR CALIBRATION DIAGNOSTIC REPORT")
    print("="*80)

    # Diagnose each motor
    for motor_name, motor_info in robot_config["motors"].items():
        motor_id = motor_info["id"]

        print(f"\n{'─'*80}")
        print(f"Motor: {motor_name} (ID: {motor_id})")
        print(f"{'─'*80}")

        # 1. Calibration info
        if motor_name in calibrations:
            calib = calibrations[motor_name]
            print(f"📋 Calibration Data:")
            print(f"   Homing Offset: {calib.homing_offset}")
            print(f"   Range: [{calib.range_min}, {calib.range_max}]")
            print(f"   Span: {calib.range_max - calib.range_min}")
            print(f"   Drive Mode: {calib.drive_mode}")
        else:
            print(f"❌ No calibration data!")
            continue

        # 2. Values read from motor
        try:
            # Present Position (raw, after homing offset applied)
            present_raw = controller._read_4byte(motor_id, controller.ADDR_PRESENT_POSITION) if hasattr(controller, '_read_4byte') else controller._read_2byte(motor_id, controller.ADDR_PRESENT_POSITION)

            # Goal Position (raw)
            goal_raw = controller._read_4byte(motor_id, controller.ADDR_GOAL_POSITION) if hasattr(controller, '_read_4byte') else controller._read_2byte(motor_id, controller.ADDR_GOAL_POSITION)

            # Homing Offset (from motor)
            homing_offset_motor = controller.get_homing_offset(motor_id)

            print(f"\n🔍 Motor Registers (Raw Values):")
            print(f"   Present Position: {present_raw}")
            print(f"   Goal Position: {goal_raw}")
            print(f"   Homing Offset (motor): {homing_offset_motor}")
            print(f"   Homing Offset (file): {calib.homing_offset}")

            # Position Limits (if available)
            try:
                if hasattr(controller, '_read_2byte'):
                    min_limit = controller._read_2byte(motor_id, controller.ADDR_MIN_POSITION_LIMIT)
                    max_limit = controller._read_2byte(motor_id, controller.ADDR_MAX_POSITION_LIMIT)
                    print(f"   Position Limits (motor): [{min_limit}, {max_limit}]")
                    print(f"   Position Limits (file): [{calib.range_min}, {calib.range_max}]")
            except:
                pass

            # 3. Check for mismatches
            print(f"\n⚠️  Issues:")
            issues_found = False

            # Homing offset mismatch
            if abs(homing_offset_motor - calib.homing_offset) > 5:
                print(f"   ❌ Homing Offset mismatch! Motor={homing_offset_motor}, File={calib.homing_offset}")
                print(f"      → Motor firmware offset doesn't match calibration file!")
                issues_found = True

            # Goal Position and Present Position difference
            position_diff = abs(goal_raw - present_raw)
            if position_diff > 100:
                print(f"   ❌ Large Goal/Present difference: {position_diff} steps")
                print(f"      → When torque is enabled, motor will jump to Goal Position!")
                print(f"      → Present: {present_raw}, Goal: {goal_raw}")
                issues_found = True

            # Present Position outside calibration range
            if present_raw < calib.range_min or present_raw > calib.range_max:
                print(f"   ⚠️  Present Position ({present_raw}) is outside calibrated range!")
                print(f"      → This may cause unexpected behavior")
                issues_found = True

            # Goal Position outside calibration range
            if goal_raw < calib.range_min or goal_raw > calib.range_max:
                print(f"   ❌ Goal Position ({goal_raw}) is outside calibrated range!")
                print(f"      → Motor will try to move outside safe range!")
                issues_found = True

            if not issues_found:
                print(f"   ✓ No issues detected")

        except Exception as e:
            print(f"   ❌ Error reading motor registers: {e}")

    print(f"\n{'='*80}")
    print("💡 RECOMMENDATIONS:")
    print("="*80)
    print("""
If you see large Goal/Present differences:
  1. Recalibrate: python scripts/calibrate_motors.py --config <config.yaml>
  2. This will sync Goal Position with current Present Position

If Homing Offset doesn't match:
  1. The calibration was not written to motor firmware correctly
  2. Recalibrate to fix this

If Goal Position is outside range:
  1. This is dangerous! Motor will jump to that position
  2. Recalibrate immediately
""")

    # Disconnect
    controller.disconnect()
    print("✓ Diagnostic complete\n")


def main():
    parser = argparse.ArgumentParser(description="Motor calibration diagnostic")
    parser.add_argument(
        "--config",
        type=str,
        required=True,
        help="Robot config file path",
    )

    args = parser.parse_args()

    try:
        diagnose(args.config)
    except KeyboardInterrupt:
        print("\n\n❌ Cancelled")
    except Exception as e:
        print(f"\n\n❌ Error: {e}")
        raise


if __name__ == "__main__":
    main()
