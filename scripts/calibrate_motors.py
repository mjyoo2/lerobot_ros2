#!/usr/bin/env python3
"""
LeRobot-style Motor Calibration (CLI)

Calibrate motors by manually moving them by hand.
Sequential step-by-step process without GUI.

Usage:
    python scripts/calibrate_motors.py --config configs/robot/so100_config.yaml
"""

import argparse
import sys
import time
import threading
from pathlib import Path

import numpy as np

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from lerobot_ros2.configs import load_config
from lerobot_ros2.hardware import DynamixelController, FeetechController
from lerobot_ros2.hardware.calibration import MotorCalibration, CalibrationManager


def input_with_timeout(prompt, timeout=None):
    """
    Wait for Enter key press (no timeout - waits until user presses Enter)

    Args:
        prompt: Message to display
        timeout: Not used (kept for compatibility)
    """
    return input(prompt)


class LeRobotCalibration:
    """LeRobot-style calibration"""

    def __init__(self, robot_config):
        self.robot_config = robot_config
        self.controller = None
        self.motor_names = list(robot_config["motors"].keys())
        self.motor_ids = [motor["id"] for motor in robot_config["motors"].values()]

        # Store results
        self.homing_offsets = {}
        self.range_mins = {}
        self.range_maxes = {}

    def _init_controller(self):
        """Initialize motor controller"""
        port = self.robot_config.get("port", "/dev/ttyUSB0")
        baudrate = self.robot_config.get("baudrate", 1000000)
        motor_type = self.robot_config.get("motor_type", "dynamixel").lower()

        if motor_type == "feetech":
            self.controller = FeetechController(port=port, baudrate=baudrate, motor_ids=self.motor_ids)
        else:
            self.controller = DynamixelController(port=port, baudrate=baudrate, motor_ids=self.motor_ids)

        self.controller.connect()
        print(f"✓ Connected {len(self.motor_ids)} motors\n")

    def run(self):
        """Run calibration"""
        print("\n" + "="*70)
        print("LeRobot-style Motor Calibration")
        print("="*70)
        print("\nThis process consists of the following steps:")
        print("  1. Disable torque (so you can move motors by hand)")
        print("  2. Set neutral pose (record Homing Offset)")
        print("  3. Record range of motion (move through full range by hand)")
        print("  4. Save calibration")
        print("\n⚠️  Prepare to move motors by hand!\n")

        try:
            # Initialize controller
            self._init_controller()

            # Step 1: Disable torque
            self._step1_disable_torque()

            # Step 2: Set center position
            self._step2_set_center()

            # Step 3: Record range
            self._step3_record_range()

            # Step 4: Save
            self._step4_save()

            print("\n" + "="*70)
            print("✅ Calibration Complete!")
            print("="*70)

        except KeyboardInterrupt:
            print("\n\n❌ Calibration cancelled")
        except Exception as e:
            print(f"\n\n❌ Error occurred: {e}")
            raise
        finally:
            if self.controller:
                self.controller.disconnect()

    def _step1_disable_torque(self):
        """Step 1: Disable torque"""
        print("="*70)
        print("Step 1: Disable Torque")
        print("="*70)

        self.controller.disable_torque()
        print("✓ All motor torques disabled.")
        print("  You can now move motors freely by hand.\n")

    def _step2_set_center(self):
        """Step 2: Set neutral pose (Homing Offset)"""
        print("="*70)
        print("Step 2: Set Neutral Pose (Homing Offset)")
        print("="*70)
        print("\n⚠️  Manually move all motors to neutral pose (center position).")
        print("   - Robot arm should be in a natural middle position.")
        print("   - This pose should allow equal range of motion in both directions.")

        input("\nPress Enter when ready...")

        # Set current position as center (2047)
        print("\n⚙️  Setting Homing Offset...")
        self.homing_offsets = self.controller.set_center_position()

        print("\n✅ Neutral pose has been set!")
        print("   This position is now the logical center (2047).\n")

    def _step3_record_range(self):
        """Step 3: Record range of motion"""
        print("="*70)
        print("Step 3: Record Range of Motion")
        print("="*70)
        print("\n⚠️  Move all joints through their full range of motion.")
        print("   - Slowly move each motor from minimum to maximum position")
        print("   - Move until you reach physical limits (don't force it)")
        print("   - Press Enter when done")

        print("\n📊 Starting real-time position tracking...\n")

        # Initial positions (raw values)
        current_positions = self.controller.read_positions_raw()

        # Initialize min/max for each motor
        for i, motor_name in enumerate(self.motor_names):
            self.range_mins[motor_name] = int(current_positions[i])
            self.range_maxes[motor_name] = int(current_positions[i])

        # Calculate max motor name length (for alignment)
        max_name_len = max(len(name) for name in self.motor_names)

        # Initial screen display
        print("📊 Recording range... (Press Enter to finish)")
        for motor_name in self.motor_names:
            print(f"  {motor_name:<{max_name_len}}  │  "
                  f"Current: ----  │  "
                  f"Range: [----, ----]  │  "
                  f"Span: ----")

        # Background position tracking
        stop_recording = threading.Event()

        def record_loop():
            """Real-time position tracking loop"""
            while not stop_recording.is_set():
                positions = self.controller.read_positions_raw()

                # Update min/max for each motor
                for i, motor_name in enumerate(self.motor_names):
                    pos = int(positions[i])
                    self.range_mins[motor_name] = min(self.range_mins[motor_name], pos)
                    self.range_maxes[motor_name] = max(self.range_maxes[motor_name], pos)

                # Clear screen (remove previous output)
                num_lines = len(self.motor_names) + 1
                for _ in range(num_lines):
                    print("\033[A\033[K", end="")  # Move up + clear line

                # Header
                print("📊 Recording range... (Press Enter to finish)")

                # Display each motor status
                for i, motor_name in enumerate(self.motor_names):
                    pos = int(positions[i])
                    min_val = self.range_mins[motor_name]
                    max_val = self.range_maxes[motor_name]
                    span = max_val - min_val

                    print(f"  {motor_name:<{max_name_len}}  │  "
                          f"Current: {pos:4d}  │  "
                          f"Range: [{min_val:4d}, {max_val:4d}]  │  "
                          f"Span: {span:4d}")

                time.sleep(0.05)  # 20Hz

        # Start background thread
        record_thread = threading.Thread(target=record_loop, daemon=True)
        record_thread.start()

        # Wait for Enter
        try:
            input()  # Wait until Enter is pressed
        finally:
            stop_recording.set()
            record_thread.join(timeout=1.0)

        print("\n\n✅ Range recording complete!")
        print("\nRecorded ranges:")
        print("─" * 70)
        for motor_name in self.motor_names:
            motor_id = self.robot_config["motors"][motor_name]["id"]
            min_val = self.range_mins[motor_name]
            max_val = self.range_maxes[motor_name]
            range_span = max_val - min_val

            print(f"  {motor_name:<{max_name_len}} (ID:{motor_id:2d})  │  "
                  f"Range: [{min_val:4d}, {max_val:4d}]  │  "
                  f"Span: {range_span:4d}")
        print("─" * 70)
        print()

    def _step4_save(self):
        """Step 4: Save calibration"""
        print("="*70)
        print("Step 4: Save Calibration")
        print("="*70)

        calibration_dir = Path(self.robot_config.get("calibration_dir", "./calibration"))

        # Create MotorCalibration objects
        calibrations = {}
        for motor_name in self.motor_names:
            motor_info = self.robot_config["motors"][motor_name]
            motor_id = motor_info["id"]

            calibrations[motor_name] = MotorCalibration(
                motor_id=motor_id,
                model=motor_info.get("model", "unknown"),
                drive_mode=0,  # 0=normal, 1=inverted
                homing_offset=self.homing_offsets.get(motor_id, 0),
                range_min=self.range_mins[motor_name],
                range_max=self.range_maxes[motor_name],
            )

        # Save to JSON file
        manager = CalibrationManager(calibration_dir)
        manager.save(calibrations)

        # Write position limits to motor EEPROM (LeRobot approach)
        print(f"\n📝 Writing Position Limits to motor EEPROM...")
        for motor_name, calib in calibrations.items():
            motor_info = self.robot_config["motors"][motor_name]
            motor_id = motor_info["id"]

            try:
                # Write Min/Max Position Limits
                self.controller._write_2byte(motor_id, self.controller.ADDR_MIN_POSITION_LIMIT, calib.range_min)
                self.controller._write_2byte(motor_id, self.controller.ADDR_MAX_POSITION_LIMIT, calib.range_max)
                print(f"  ✓ Motor {motor_id} ({motor_name}): limits=[{calib.range_min}, {calib.range_max}]")
            except Exception as e:
                print(f"  ✗ Motor {motor_id} ({motor_name}): Failed to write limits - {e}")

        # CRITICAL: Update Goal Position to current Present Position
        # This prevents motors from jumping when torque is enabled
        import time
        print(f"\n📍 Syncing Goal Position with Present Position...")
        time.sleep(0.1)  # Wait for EEPROM writes to complete

        for motor_name, calib in calibrations.items():
            motor_info = self.robot_config["motors"][motor_name]
            motor_id = motor_info["id"]

            try:
                # Read current Present Position (after homing offset applied)
                if hasattr(self.controller, '_read_4byte'):
                    present_pos = self.controller._read_4byte(motor_id, self.controller.ADDR_PRESENT_POSITION)
                else:
                    present_pos = self.controller._read_2byte(motor_id, self.controller.ADDR_PRESENT_POSITION)

                # Set Goal Position to current Present Position
                if hasattr(self.controller, '_write_4byte'):
                    self.controller._write_4byte(motor_id, self.controller.ADDR_GOAL_POSITION, present_pos)
                else:
                    self.controller._write_2byte(motor_id, self.controller.ADDR_GOAL_POSITION, present_pos)

                print(f"  ✓ Motor {motor_id} ({motor_name}): Goal={present_pos}")
            except Exception as e:
                print(f"  ✗ Motor {motor_id} ({motor_name}): Failed to sync Goal - {e}")

        print(f"\n✅ Calibration saved: {calibration_dir}")
        print(f"   • Homing Offsets: permanently stored in motor firmware")
        print(f"   • Position Limits: written to motor EEPROM")
        print(f"   • Calibration data: saved to {calibration_dir}/calibration.json")


def main():
    parser = argparse.ArgumentParser(
        description="LeRobot-style calibration (move motors by hand)"
    )
    parser.add_argument(
        "--config",
        type=str,
        required=True,
        help="Robot config file path (e.g., configs/robot/so100_config.yaml)",
    )

    args = parser.parse_args()

    # Load config
    print(f"Loading config: {args.config}")
    robot_config = load_config(args.config)

    # Run calibration
    calibration = LeRobotCalibration(robot_config)
    calibration.run()


if __name__ == "__main__":
    main()
