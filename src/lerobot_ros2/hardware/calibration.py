"""
Motor Calibration Data Structures

Store and load calibration data (LeRobot-inspired, custom implementation)
"""

import json
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, Optional


@dataclass
class MotorCalibration:
    """
    Motor calibration data (LeRobot-compatible)

    Attributes:
        motor_id: Motor ID (same as 'id' in LeRobot)
        model: Motor model name (e.g., "xl330-m288")
        drive_mode: Drive mode (0=normal, 1=inverted)
        homing_offset: Homing offset written to motor EEPROM
        range_min: Minimum position (Present_Position with homing_offset already applied)
        range_max: Maximum position (Present_Position with homing_offset already applied)
    """
    motor_id: int
    model: str
    drive_mode: int = 0
    homing_offset: int = 0
    range_min: int = 0
    range_max: int = 4095

    def to_dict(self) -> dict:
        """Convert to dictionary"""
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> 'MotorCalibration':
        """Create from dictionary"""
        return cls(**data)

    def clamp_position(self, position: int) -> int:
        """
        Clamp position within calibration range

        Args:
            position: Target position (Present_Position)

        Returns:
            Clamped position
        """
        return max(self.range_min, min(position, self.range_max))

    def is_within_range(self, position: int) -> bool:
        """
        Check if position is within calibration range

        Args:
            position: Position (Present_Position)

        Returns:
            True if within range
        """
        return self.range_min <= position <= self.range_max


class CalibrationManager:
    """Calibration file manager"""

    def __init__(self, calibration_dir: Optional[Path] = None):
        """
        Args:
            calibration_dir: Calibration storage directory
        """
        self.calibration_dir = Path(calibration_dir) if calibration_dir else None

    def save(self, calibrations: Dict[str, MotorCalibration], filename: str = "calibration.json"):
        """
        Save calibration to JSON file

        Args:
            calibrations: Motor name → MotorCalibration dictionary
            filename: Filename to save
        """
        if not self.calibration_dir:
            raise ValueError("calibration_dir not set")

        self.calibration_dir.mkdir(parents=True, exist_ok=True)

        # Convert to dictionary
        calib_dict = {
            name: calib.to_dict()
            for name, calib in calibrations.items()
        }

        # Save as JSON
        filepath = self.calibration_dir / filename
        with open(filepath, "w") as f:
            json.dump(calib_dict, f, indent=2)

        print(f"✓ Calibration saved: {filepath}")

    def load(self, filename: str = "calibration.json") -> Dict[str, MotorCalibration]:
        """
        Load calibration from JSON file

        Args:
            filename: Filename to load

        Returns:
            Motor name → MotorCalibration dictionary
        """
        if not self.calibration_dir:
            raise ValueError("calibration_dir not set")

        filepath = self.calibration_dir / filename

        if not filepath.exists():
            raise FileNotFoundError(f"Calibration file not found: {filepath}")

        # Load from JSON
        with open(filepath, "r") as f:
            calib_dict = json.load(f)

        # Convert to MotorCalibration objects
        calibrations = {
            name: MotorCalibration.from_dict(data)
            for name, data in calib_dict.items()
        }

        print(f"✓ Calibration loaded: {filepath} ({len(calibrations)} motors)")
        return calibrations

    def exists(self, filename: str = "calibration.json") -> bool:
        """
        Check if calibration file exists

        Args:
            filename: Filename to check

        Returns:
            True if exists
        """
        if not self.calibration_dir:
            return False

        filepath = self.calibration_dir / filename
        return filepath.exists()


def create_default_calibration(
    motor_id: int,
    model: str,
    drive_mode: int = 0,
    homing_offset: int = 0,
    range_min: int = 0,
    range_max: int = 4095
) -> MotorCalibration:
    """
    Create default calibration

    Args:
        motor_id: Motor ID
        model: Motor model name
        drive_mode: Drive mode (0=normal, 1=inverted)
        homing_offset: Homing offset
        range_min: Minimum position
        range_max: Maximum position

    Returns:
        MotorCalibration object
    """
    return MotorCalibration(
        motor_id=motor_id,
        model=model,
        drive_mode=drive_mode,
        homing_offset=homing_offset,
        range_min=range_min,
        range_max=range_max,
    )


# Convenience functions
def load_calibration(calibration_dir: Path, filename: str = "calibration.json") -> Dict[str, MotorCalibration]:
    """
    Load calibration (convenience function)

    Args:
        calibration_dir: Calibration directory
        filename: Filename

    Returns:
        Motor name → MotorCalibration dictionary
    """
    manager = CalibrationManager(calibration_dir)
    return manager.load(filename)


def save_calibration(
    calibrations: Dict[str, MotorCalibration],
    calibration_dir: Path,
    filename: str = "calibration.json"
):
    """
    Save calibration (convenience function)

    Args:
        calibrations: Motor name → MotorCalibration dictionary
        calibration_dir: Calibration directory
        filename: Filename
    """
    manager = CalibrationManager(calibration_dir)
    manager.save(calibrations, filename)
