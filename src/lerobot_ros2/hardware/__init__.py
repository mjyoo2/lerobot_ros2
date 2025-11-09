"""
Hardware control modules (Direct implementation, LeRobot dependency removed)
"""

from .find_port import find_dynamixel_port, get_available_ports, get_port_info, test_port_connection
from .calibration import MotorCalibration, CalibrationManager, load_calibration, save_calibration

# Optional imports
try:
    from .dynamixel_controller import DynamixelController
except ImportError:
    DynamixelController = None

try:
    from .feetech_controller import FeetechController
except ImportError:
    FeetechController = None

try:
    from .camera_controller import CameraController
except ImportError:
    CameraController = None

__all__ = [
    "DynamixelController",
    "FeetechController",
    "CameraController",
    "find_dynamixel_port",
    "get_available_ports",
    "get_port_info",
    "test_port_connection",
    "MotorCalibration",
    "CalibrationManager",
    "load_calibration",
    "save_calibration",
]
