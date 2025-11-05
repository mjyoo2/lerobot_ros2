"""
Robot configuration schemas.

Defines configuration structures for robots compatible with LeRobot.
"""

from dataclasses import dataclass, field
from typing import Dict, Any, Optional


@dataclass
class RobotConfig:
    """
    Base robot configuration.

    This is a simplified configuration that can be converted to
    LeRobot-specific robot configs (e.g., SO100FollowerConfig).
    """

    # Robot identification
    robot_type: str  # e.g., "so100_follower", "koch_v1.1", etc.
    robot_id: str = "robot_001"

    # Hardware connection
    port: str = "/dev/ttyUSB0"
    baudrate: int = 1_000_000

    # Camera configuration
    cameras: Dict[str, Any] = field(default_factory=dict)

    # Calibration
    calibration_dir: Optional[str] = None

    # Control parameters
    max_relative_target: Optional[float] = None  # Safety: max movement per step

    # Additional robot-specific parameters
    extra_params: Dict[str, Any] = field(default_factory=dict)

    def to_lerobot_config(self):
        """
        Convert to LeRobot robot configuration.

        Returns the configuration in a format compatible with LeRobot's
        make_robot_from_config() function.
        """
        # Import here to avoid circular dependency
        try:
            if self.robot_type == "so100_follower":
                from lerobot.robots.so100_follower import SO100FollowerConfig
                from lerobot.cameras.opencv import OpenCVCameraConfig
                from lerobot.cameras.realsense import RealSenseCameraConfig

                # Convert camera configs
                camera_configs = {}
                for cam_name, cam_cfg in self.cameras.items():
                    cam_type = cam_cfg.get("type", "opencv")

                    if cam_type == "opencv":
                        camera_configs[cam_name] = OpenCVCameraConfig(
                            index_or_path=cam_cfg.get("index", 0),
                            fps=cam_cfg.get("fps", 30),
                            width=cam_cfg.get("width", 640),
                            height=cam_cfg.get("height", 480),
                            color_mode=cam_cfg.get("color_mode", "rgb"),
                        )
                    elif cam_type == "realsense":
                        camera_configs[cam_name] = RealSenseCameraConfig(
                            fps=cam_cfg.get("fps", 30),
                            width=cam_cfg.get("width", 640),
                            height=cam_cfg.get("height", 480),
                            enable_depth=cam_cfg.get("enable_depth", False),
                        )

                return SO100FollowerConfig(
                    port=self.port,
                    cameras=camera_configs,
                    calibration_dir=self.calibration_dir,
                    max_relative_target=self.max_relative_target,
                    **self.extra_params,
                )

            else:
                raise ValueError(f"Unsupported robot type: {self.robot_type}")

        except ImportError as e:
            raise ImportError(
                f"Failed to import LeRobot components for {self.robot_type}: {e}"
            )


@dataclass
class TeleoperatorConfig:
    """
    Teleoperator configuration.

    Configuration for teleoperation devices (leader arms, keyboards, etc.).
    """

    # Teleoperator type
    teleop_type: str  # e.g., "so100_leader", "keyboard", "gamepad"

    # Hardware connection (if applicable)
    port: Optional[str] = None
    baudrate: int = 1_000_000

    # Calibration
    calibration_dir: Optional[str] = None

    # Additional parameters
    extra_params: Dict[str, Any] = field(default_factory=dict)

    def to_lerobot_config(self):
        """Convert to LeRobot teleoperator configuration."""
        try:
            if self.teleop_type == "so100_leader":
                from lerobot.teleoperators.so100_leader import SO100LeaderConfig

                return SO100LeaderConfig(
                    port=self.port,
                    calibration_dir=self.calibration_dir,
                    **self.extra_params,
                )

            else:
                raise ValueError(f"Unsupported teleoperator type: {self.teleop_type}")

        except ImportError as e:
            raise ImportError(
                f"Failed to import LeRobot teleoperator for {self.teleop_type}: {e}"
            )
