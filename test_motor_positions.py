#!/usr/bin/env python3
"""Quick test to check current motor positions"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent / "src"))

from lerobot_ros2.hardware import FeetechController
from lerobot_ros2.hardware.calibration import CalibrationManager

# Load config
port = "/dev/ttyACM0"
motor_ids = [1, 2, 3, 4, 5, 6]

# Load calibration
calib_dir = Path("calibration/custom_robot")
manager = CalibrationManager(calib_dir)
calibrations = manager.load()

# Map to motor_id
calib_by_id = {calib.motor_id: calib for calib in calibrations.values()}

print(f"\n{'='*60}")
print("Motor Position Test")
print(f"{'='*60}\n")

# Connect
controller = FeetechController(
    port=port,
    baudrate=1000000,
    motor_ids=motor_ids,
    calibration=calib_by_id
)

if not controller.connect():
    print("Connection failed!")
    sys.exit(1)

# The debug output will show during connect()
print("\nTest complete!\n")

controller.disconnect()
