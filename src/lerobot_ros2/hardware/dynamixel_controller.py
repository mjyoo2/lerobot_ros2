"""
Dynamixel Motor Controller (Direct implementation without LeRobot)

This module directly uses the Dynamixel SDK to control motors.
"""

import time
from typing import List, Dict, Optional
import numpy as np

try:
    from dynamixel_sdk import *
except ImportError:
    raise ImportError(
        "Dynamixel SDK is not installed.\n"
        "Install with: pip install dynamixel-sdk"
    )

# Handle both direct execution and module import
try:
    from .find_port import find_dynamixel_port, get_available_ports
except ImportError:
    from find_port import find_dynamixel_port, get_available_ports


class DynamixelController:
    """
    Class for directly controlling Dynamixel motors

    Args:
        port: Serial port (e.g., '/dev/ttyUSB0', 'COM3')
        baudrate: Communication speed (Default: 1000000)
        motor_ids: List of motor IDs (e.g., [1, 2, 3, 4, 5, 6])
    """

    # Control table addresses (based on XL330, refer to manual for other models)
    ADDR_HOMING_OFFSET = 20
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_POSITION = 132
    ADDR_PRESENT_VELOCITY = 128
    ADDR_OPERATING_MODE = 11

    # Protocol version
    PROTOCOL_VERSION = 2.0

    # Default values
    TORQUE_ENABLE = 1
    TORQUE_DISABLE = 0

    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = 1000000,
        motor_ids: List[int] = None,
        calibration: Optional[Dict] = None,
    ):
        # Auto-detect port if not specified
        if port is None:
            print("🔍 Port not specified. Attempting auto-detection...")
            port = find_dynamixel_port(baudrate)
            if port is None:
                available = get_available_ports()
                if available:
                    print(f"⚠️  Could not automatically find Dynamixel port.")
                    print(f"   Available ports: {', '.join(available)}")
                    print(f"   Please specify port explicitly: DynamixelController(port='{available[0]}')")
                else:
                    print("❌ No USB serial ports found.")
                    print("   💡 Tip: Run python3 -m lerobot_ros2.hardware.find_port for detailed info.")
                raise ValueError("No USB serial ports found")
            else:
                print(f"✅ Port auto-detection successful: {port}")

        self.port = port
        self.baudrate = baudrate
        self.motor_ids = motor_ids or []
        self.calibration = calibration or {}  # motor_id -> MotorCalibration

        # Dynamixel SDK objects
        self.port_handler = None
        self.packet_handler = None

        self.is_connected = False

        # Position tracking
        self.current_positions = np.zeros(len(self.motor_ids), dtype=np.float32)
        self.target_positions = np.zeros(len(self.motor_ids), dtype=np.float32)

    def connect(self) -> bool:
        """Connect to motors"""
        try:
            # Create port handler
            self.port_handler = PortHandler(self.port)
            self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)

            # Open port
            if not self.port_handler.openPort():
                raise Exception(f"Failed to open port: {self.port}")

            # Set baudrate
            if not self.port_handler.setBaudRate(self.baudrate):
                raise Exception(f"Failed to set baudrate: {self.baudrate}")

            print(f"✓ Port connected successfully: {self.port} @ {self.baudrate} bps")

            # Check and initialize each motor
            for motor_id in self.motor_ids:
                # Ping test
                dxl_model_number, dxl_comm_result, dxl_error = self.packet_handler.ping(
                    self.port_handler, motor_id
                )

                if dxl_comm_result != COMM_SUCCESS:
                    raise Exception(f"Motor {motor_id} communication failed")
                elif dxl_error != 0:
                    raise Exception(f"Motor {motor_id} error: {self.packet_handler.getRxPacketError(dxl_error)}")

                print(f"✓ Motor {motor_id} connected successfully (Model: {dxl_model_number})")

                # Start with torque disabled (user must manually enable)
                self._write_1byte(motor_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)

            self.is_connected = True

            # Read initial positions (calibration may not exist, so read raw values)
            if self.calibration:
                # If calibration exists, read normalized values
                self.current_positions = self.read_positions(normalize=True)
            else:
                # If no calibration, read raw values (during calibration process)
                self.current_positions = self.read_positions_raw().astype(np.float32)

            self.target_positions = self.current_positions.copy()

            print(f"✓ Initialized {len(self.motor_ids)} motors successfully")
            return True

        except Exception as e:
            print(f"✗ Connection failed: {e}")
            self.is_connected = False
            return False

    def disconnect(self):
        """Disconnect from motors"""
        if not hasattr(self, 'is_connected') or not self.is_connected:
            return

        try:
            # Disable torque for all motors
            for motor_id in self.motor_ids:
                self._write_1byte(motor_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)

            # Close port
            if self.port_handler:
                self.port_handler.closePort()

            self.is_connected = False
            print("✓ Motors disconnected")

        except Exception as e:
            print(f"✗ Error during disconnect: {e}")

    def enable_torque(self, motor_ids: Optional[List[int]] = None):
        """
        Enable motor torque (sets current position as goal to prevent sudden movement)

        Args:
            motor_ids: List of motor IDs to enable. None = all motors
        """
        if not self.is_connected:
            raise Exception("Motors not connected")

        ids_to_enable = motor_ids if motor_ids is not None else self.motor_ids

        # CRITICAL: Set goal position to current position BEFORE enabling torque
        # This prevents motors from jumping to old goal positions
        for motor_id in ids_to_enable:
            try:
                # Read current position
                current_pos = self._read_4byte(motor_id, self.ADDR_PRESENT_POSITION)
                # Set as goal position
                self._write_4byte(motor_id, self.ADDR_GOAL_POSITION, current_pos)
            except Exception as e:
                print(f"  ⚠️  Failed to set goal for motor {motor_id}: {e}")

        # Now enable torque
        for motor_id in ids_to_enable:
            self._write_1byte(motor_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)

        print(f"✓ Torque enabled: {ids_to_enable}")

    def disable_torque(self, motor_ids: Optional[List[int]] = None):
        """
        Disable motor torque (motors can be moved by hand)

        Args:
            motor_ids: List of motor IDs to disable. None = all motors
        """
        if not self.is_connected:
            raise Exception("Motors not connected")

        ids_to_disable = motor_ids if motor_ids is not None else self.motor_ids

        for motor_id in ids_to_disable:
            self._write_1byte(motor_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)

        print(f"✓ Torque disabled: {ids_to_disable}")

    def set_homing_offset(self, motor_id: int, offset: int):
        """
        Set Homing Offset (stored in motor firmware)

        Dynamixel: Present_Position = Actual_Position + Homing_Offset

        Args:
            motor_id: Motor ID
            offset: Homing Offset value (signed int, -1,044,479 to 1,044,479)

        Note:
            Homing Offset is in EEPROM area, so torque must be disabled to write.
        """
        if not self.is_connected:
            raise Exception("Motors not connected")

        # Homing Offset is signed int32
        if offset < -1044479 or offset > 1044479:
            raise ValueError(f"Homing Offset out of range: {offset}")

        # Convert negative to unsigned (two's complement)
        if offset < 0:
            offset = (1 << 32) + offset

        self._write_4byte(motor_id, self.ADDR_HOMING_OFFSET, offset)

    def get_homing_offset(self, motor_id: int) -> int:
        """
        Read Homing Offset

        Args:
            motor_id: Motor ID

        Returns:
            Homing Offset value (signed int)
        """
        if not self.is_connected:
            raise Exception("Motors not connected")

        offset = self._read_4byte(motor_id, self.ADDR_HOMING_OFFSET)

        # Convert unsigned to signed
        if offset > 2147483647:  # 2^31 - 1
            offset = offset - (1 << 32)

        return offset

    def set_center_position(self) -> Dict[int, int]:
        """
        Set current physical position as center (2047)

        LeRobot approach: Map current position to 2047 (12-bit center)

        Returns:
            Dictionary of Homing Offsets for each motor

        Note:
            - Must be called with torque disabled
            - Position motors in neutral pose before running
            - This function does NOT re-enable torque (LeRobot approach)
        """
        if not self.is_connected:
            raise Exception("Motors not connected")

        offsets = {}

        for motor_id in self.motor_ids:
            # Read current physical position (raw, before Homing Offset applied)
            current_pos = self._read_4byte(motor_id, self.ADDR_PRESENT_POSITION)

            # 12-bit encoder center = 2047 (half of 4096 - 1)
            # Dynamixel: Present = Actual + Offset
            # Desired Present = 2047
            # Offset = 2047 - current_pos
            center = 2047
            offset = center - current_pos

            # Set Homing Offset (torque already disabled)
            self.set_homing_offset(motor_id, offset)

            offsets[motor_id] = offset
            print(f"  Motor {motor_id}: Physical {current_pos} → Logical center {center} (offset={offset})")

        return offsets

    def _normalize(self, positions_raw: np.ndarray) -> np.ndarray:
        """
        Convert raw values to normalized values (-100 to +100)

        LeRobot 2-point calibration (EXACT implementation):
        norm = (((val - min) / (max - min)) * 200) - 100

        Args:
            positions_raw: Raw position values (Present_Position, homing_offset already applied)

        Returns:
            Normalized values (-100 to +100)
        """
        if not self.calibration:
            raise RuntimeError("Calibration not loaded. Please run calibration first.")

        normalized = np.zeros(len(positions_raw), dtype=np.float32)

        for i, (motor_id, pos_raw) in enumerate(zip(self.motor_ids, positions_raw)):
            if motor_id not in self.calibration:
                raise ValueError(f"No calibration data for motor {motor_id}.")

            calib = self.calibration[motor_id]
            min_val = calib.range_min
            max_val = calib.range_max

            if min_val == max_val:
                raise ValueError(f"Motor {motor_id}: min and max are identical.")

            # Bound within range
            bounded_val = min(max_val, max(min_val, pos_raw))

            # LeRobot formula for RANGE_M100_100
            normalized[i] = (((bounded_val - min_val) / (max_val - min_val)) * 200) - 100

            # Dynamixel: drive_mode stored but NOT used in normalization

        return normalized

    def _unnormalize(self, positions_norm: np.ndarray) -> np.ndarray:
        """
        Convert normalized values to raw values

        LeRobot 2-point calibration (EXACT implementation):
        val = int(((norm + 100) / 200) * (max - min) + min)

        Args:
            positions_norm: Normalized values (-100 to +100)

        Returns:
            Raw position values (Goal_Position, homing_offset will be applied by firmware)
        """
        if not self.calibration:
            raise RuntimeError("Calibration not loaded. Please run calibration first.")

        raw = np.zeros(len(positions_norm), dtype=np.int32)

        for i, (motor_id, pos_norm) in enumerate(zip(self.motor_ids, positions_norm)):
            if motor_id not in self.calibration:
                raise ValueError(f"No calibration data for motor {motor_id}.")

            calib = self.calibration[motor_id]
            min_val = calib.range_min
            max_val = calib.range_max

            if min_val == max_val:
                raise ValueError(f"Motor {motor_id}: min and max are identical.")

            # Bound within -100 to +100
            bounded_norm = min(100.0, max(-100.0, pos_norm))

            # Dynamixel: drive_mode NOT used in normalization

            # LeRobot formula for RANGE_M100_100
            raw[i] = int(((bounded_norm + 100) / 200) * (max_val - min_val) + min_val)

        return raw

    def read_positions(self, normalize: bool = True) -> np.ndarray:
        """
        Read current motor positions (LeRobot-compatible)

        Args:
            normalize: If True (default), return normalized values (-100~+100).
                      If False, return raw values (0~4095, legacy mode).

        Returns:
            Normalized values (-100~+100) by default
        """
        if not self.is_connected:
            return self.current_positions

        # Read raw values
        positions_raw = self.read_positions_raw()

        if normalize:
            # Return normalized values (LeRobot default)
            normalized = self._normalize(positions_raw)
            self.current_positions = normalized
            return normalized.copy()
        else:
            # Legacy mode: return raw values
            return positions_raw.astype(np.float32)

    def read_positions_raw(self) -> np.ndarray:
        """
        Read current motor positions (raw values, 0~4095)

        Used in calibration etc.

        Returns:
            Raw position values for each motor (0~4095)
        """
        if not self.is_connected:
            return np.zeros(len(self.motor_ids), dtype=np.int32)

        positions = np.zeros(len(self.motor_ids), dtype=np.int32)
        for i, motor_id in enumerate(self.motor_ids):
            positions[i] = self._read_4byte(motor_id, self.ADDR_PRESENT_POSITION)

        return positions

    def write_positions(self, positions: np.ndarray, normalize: bool = True):
        """
        Write target positions (LeRobot-compatible)

        Args:
            positions: Target position array
            normalize: If True (default), positions are normalized values (-100~+100).
                      If False, positions are raw values (0~4095, legacy mode).
        """
        if not self.is_connected:
            return

        if len(positions) != len(self.motor_ids):
            raise ValueError(f"Position count mismatch: {len(positions)} != {len(self.motor_ids)}")

        self.target_positions = positions.copy()

        # Convert to raw values
        if normalize:
            # Normalized values → raw (LeRobot default)
            positions_raw = self._unnormalize(positions)
        else:
            # Legacy mode: positions are already raw values
            positions_raw = positions.astype(np.int32)

        # Write to motors
        for i, motor_id in enumerate(self.motor_ids):
            self._write_4byte(motor_id, self.ADDR_GOAL_POSITION, int(positions_raw[i]))

    def _read_1byte(self, motor_id: int, address: int) -> int:
        """Read 1 byte"""
        data, result, error = self.packet_handler.read1ByteTxRx(
            self.port_handler, motor_id, address
        )
        if result != COMM_SUCCESS or error != 0:
            raise Exception(f"Read failed (motor {motor_id}, address {address})")
        return data

    def _read_4byte(self, motor_id: int, address: int) -> int:
        """Read 4 bytes"""
        data, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, motor_id, address
        )
        if result != COMM_SUCCESS or error != 0:
            raise Exception(f"Read failed (motor {motor_id}, address {address})")
        return data

    def _write_1byte(self, motor_id: int, address: int, value: int):
        """Write 1 byte"""
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, address, value
        )
        if result != COMM_SUCCESS or error != 0:
            raise Exception(f"Write failed (motor {motor_id}, address {address})")

    def _write_4byte(self, motor_id: int, address: int, value: int):
        """Write 4 bytes"""
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, motor_id, address, value
        )
        if result != COMM_SUCCESS or error != 0:
            raise Exception(f"Write failed (motor {motor_id}, address {address})")

    def _raw_to_radians(self, raw_value: int) -> float:
        """
        Convert Dynamixel raw value to radians
        XL330: 0~4095 -> 0~2π
        """
        # 4096 position steps = 360 degrees = 2π radians
        return (raw_value / 4096.0) * 2.0 * np.pi

    def _radians_to_raw(self, radians: float) -> int:
        """
        Convert radians to Dynamixel raw value
        Based on XL330: 0~2π -> 0~4095
        """
        # Normalize to 0~2π range
        radians = radians % (2.0 * np.pi)
        raw = int((radians / (2.0 * np.pi)) * 4096.0)
        return max(0, min(4095, raw))  # Clamp to valid range

    def get_observation(self) -> Dict[str, np.ndarray]:
        """
        Get current observation (LeRobot-compatible format)

        Returns:
            {"observation.state": positions}
        """
        positions = self.read_positions()
        return {
            "observation.state": positions
        }

    def send_action(self, action: Dict[str, np.ndarray]):
        """
        Send action (LeRobot-compatible format)

        Args:
            action: {"action": positions}
        """
        if "action" in action:
            self.write_positions(action["action"])
        else:
            raise ValueError("Action dictionary missing 'action' key")

    def __del__(self):
        """Destructor"""
        self.disconnect()


def test_dynamixel():
    """Test code"""
    print("Dynamixel test starting...")

    # Configuration
    PORT = None  # Auto-detect (or specify like "/dev/ttyUSB0")
    BAUDRATE = 1000000
    MOTOR_IDS = [1, 2, 3]  # Change to your actual motor IDs

    # Create controller (with port auto-detection)
    try:
        controller = DynamixelController(PORT, BAUDRATE, MOTOR_IDS)
    except ValueError as e:
        print(f"\n❌ Error: {e}")
        print("\n💡 Solutions:")
        print("   1. Check if USB cable is connected")
        print("   2. If using WSL, attach USB device with usbipd:")
        print("      In Windows PowerShell (as Administrator):")
        print("      > usbipd wsl list")
        print("      > usbipd wsl attach --busid <BUSID>")
        print("   3. Specify port manually:")
        print("      controller = DynamixelController(port='/dev/ttyUSB0', ...)")
        return

    # Connect
    if not controller.connect():
        print("Connection failed")
        return

    # Read current positions
    positions = controller.read_positions()
    print(f"Current positions (radians): {positions}")
    print(f"Current positions (degrees): {np.rad2deg(positions)}")

    # Set target positions (all motors to 0 degrees)
    target = np.zeros(len(MOTOR_IDS), dtype=np.float32)
    print(f"\nMoving to target positions: {target}")
    controller.write_positions(target)

    # Wait briefly
    time.sleep(2)

    # Read new positions
    new_positions = controller.read_positions()
    print(f"New positions (radians): {new_positions}")
    print(f"New positions (degrees): {np.rad2deg(new_positions)}")

    # Disconnect
    controller.disconnect()
    print("\nTest complete")


if __name__ == "__main__":
    test_dynamixel()
