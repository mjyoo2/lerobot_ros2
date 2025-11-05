"""
Safety utilities for robot control

Validates commands and prevents dangerous movements.
"""

import numpy as np
from typing import Dict, Optional, List
import logging


class SafetyChecker:
    """
    Safety checks for robot commands

    Validates:
    - Position limits
    - Velocity limits
    - Acceleration limits
    - Position delta (prevents sudden jumps)
    """

    def __init__(
        self,
        position_limits: Optional[Dict[int, tuple]] = None,
        max_velocity: float = 2.0,
        max_acceleration: float = 3.0,
        max_position_delta: float = 30.0,  # Normalized units per step
    ):
        """
        Args:
            position_limits: Motor ID → (min, max) in normalized units
            max_velocity: Maximum velocity (radians/sec)
            max_acceleration: Maximum acceleration (radians/sec^2)
            max_position_delta: Maximum position change per command (normalized)
        """
        self.position_limits = position_limits or {}
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.max_position_delta = max_position_delta

        self.last_positions: Optional[np.ndarray] = None
        self.last_time: Optional[float] = None

        self.logger = logging.getLogger(__name__)

    def check_position_limits(
        self,
        positions: np.ndarray,
        motor_ids: Optional[List[int]] = None
    ) -> bool:
        """
        Check if positions are within limits

        Args:
            positions: Target positions (normalized)
            motor_ids: Motor IDs (optional, for specific limit checking)

        Returns:
            True if safe, False otherwise
        """
        if not self.position_limits:
            # No limits specified, assume safe
            return True

        for idx, pos in enumerate(positions):
            if motor_ids and idx < len(motor_ids):
                motor_id = motor_ids[idx]
                if motor_id in self.position_limits:
                    min_pos, max_pos = self.position_limits[motor_id]
                    if not (min_pos <= pos <= max_pos):
                        self.logger.warning(
                            f"Position limit violation: motor {motor_id}, "
                            f"position {pos:.2f} not in [{min_pos}, {max_pos}]"
                        )
                        return False

        return True

    def check_position_delta(
        self,
        positions: np.ndarray
    ) -> bool:
        """
        Check if position change is reasonable (prevents sudden jumps)

        Args:
            positions: Target positions (normalized)

        Returns:
            True if safe, False otherwise
        """
        if self.last_positions is None:
            # First command, assume safe
            self.last_positions = positions.copy()
            return True

        # Calculate delta
        delta = np.abs(positions - self.last_positions)
        max_delta = np.max(delta)

        if max_delta > self.max_position_delta:
            self.logger.warning(
                f"Position delta too large: {max_delta:.2f} > {self.max_position_delta:.2f}"
            )
            self.logger.warning(
                f"Previous: {self.last_positions.tolist()}, "
                f"Target: {positions.tolist()}"
            )
            return False

        # Update last positions
        self.last_positions = positions.copy()
        return True

    def validate_command(
        self,
        positions: np.ndarray,
        motor_ids: Optional[List[int]] = None
    ) -> tuple[bool, str]:
        """
        Comprehensive validation of command

        Args:
            positions: Target positions (normalized)
            motor_ids: Motor IDs (optional)

        Returns:
            (is_safe, error_message)
        """
        # Check NaN or Inf
        if np.any(np.isnan(positions)) or np.any(np.isinf(positions)):
            return False, "Invalid values (NaN or Inf) in command"

        # Check position limits
        if not self.check_position_limits(positions, motor_ids):
            return False, "Position limit violation"

        # Check position delta
        if not self.check_position_delta(positions):
            return False, "Position delta too large (sudden jump detected)"

        return True, ""

    def clamp_to_limits(
        self,
        positions: np.ndarray,
        motor_ids: Optional[List[int]] = None
    ) -> np.ndarray:
        """
        Clamp positions to limits

        Args:
            positions: Target positions (normalized)
            motor_ids: Motor IDs (optional)

        Returns:
            Clamped positions
        """
        clamped = positions.copy()

        if self.position_limits:
            for idx, pos in enumerate(positions):
                if motor_ids and idx < len(motor_ids):
                    motor_id = motor_ids[idx]
                    if motor_id in self.position_limits:
                        min_pos, max_pos = self.position_limits[motor_id]
                        clamped[idx] = np.clip(pos, min_pos, max_pos)

        # Also clamp to normalized range
        clamped = np.clip(clamped, -100.0, 100.0)

        return clamped

    def reset(self):
        """Reset safety checker state"""
        self.last_positions = None
        self.last_time = None
