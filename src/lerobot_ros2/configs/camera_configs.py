"""
Camera configuration schemas.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class CameraConfig:
    """
    Camera configuration.

    Supports both OpenCV (USB webcams) and RealSense cameras.
    """

    # Camera type
    camera_type: str  # "opencv" or "realsense"

    # Camera name/identifier
    name: str = "camera"

    # OpenCV camera parameters
    index: int = 0  # Camera index or device path

    # Common parameters
    fps: int = 30
    width: int = 640
    height: int = 480
    color_mode: str = "rgb"  # "rgb" or "bgr"

    # RealSense specific
    enable_depth: bool = False
    serial_number: Optional[str] = None

    # Preprocessing
    rotation: int = 0  # 0, 90, 180, 270

    def to_lerobot_config(self):
        """Convert to LeRobot camera configuration."""
        try:
            if self.camera_type == "opencv":
                from lerobot.cameras.opencv import OpenCVCameraConfig

                return OpenCVCameraConfig(
                    index_or_path=self.index,
                    fps=self.fps,
                    width=self.width,
                    height=self.height,
                    color_mode=self.color_mode,
                    rotation=self.rotation,
                )

            elif self.camera_type == "realsense":
                from lerobot.cameras.realsense import RealSenseCameraConfig

                return RealSenseCameraConfig(
                    serial_number=self.serial_number,
                    fps=self.fps,
                    width=self.width,
                    height=self.height,
                    enable_depth=self.enable_depth,
                    color_mode=self.color_mode,
                )

            else:
                raise ValueError(f"Unsupported camera type: {self.camera_type}")

        except ImportError as e:
            raise ImportError(
                f"Failed to import LeRobot camera for {self.camera_type}: {e}"
            )
