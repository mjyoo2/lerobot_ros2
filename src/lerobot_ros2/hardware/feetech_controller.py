"""
Feetech motor controller (native implementation)

Feetech STS3215 control module implemented directly with reference to LeRobot's FeetechMotorsBus
Provides simple interface for SO-100, SO-101 robots
"""

import time
from typing import List, Dict, Optional
import numpy as np

try:
    import scservo_sdk as scs
except ImportError:
    raise ImportError(
        "scservo_sdk is not installed.\n"
        "Install: pip install feetech-servo-sdk"
    )

try:
    from .find_port import find_dynamixel_port, get_available_ports
except ImportError:
    from find_port import find_dynamixel_port, get_available_ports


class FeetechController:
    """
    Class for directly controlling Feetech STS3215 servo motors

    Args:
        port: Serial port (e.g., '/dev/ttyACM0')
        baudrate: Communication speed (Default: 1000000)
        motor_ids: List of motor IDs (e.g., [1, 2, 3, 4, 5, 6])
    """

    # Control table addresses for STS3215
    ADDR_MIN_POSITION_LIMIT = 9
    ADDR_MAX_POSITION_LIMIT = 11
    ADDR_HOMING_OFFSET = 31
    ADDR_TORQUE_ENABLE = 40
    ADDR_ACCELERATION = 41
    ADDR_GOAL_POSITION = 42
    ADDR_GOAL_TIME = 44
    ADDR_GOAL_VELOCITY = 46
    ADDR_LOCK = 55  # LeRobot uses this
    ADDR_PRESENT_POSITION = 56
    ADDR_PRESENT_SPEED = 58
    ADDR_OPERATING_MODE = 33

    # Protocol version
    PROTOCOL_VERSION = 0  # SCS Protocol

    # Values
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
            print("🔍 Auto-detecting port...")
            port = find_dynamixel_port(baudrate)
            if port is None:
                available = get_available_ports()
                if available:
                    print(f"⚠️  Could not auto-detect port.")
                    print(f"   Available ports: {', '.join(available)}")
                    port = available[0]
                    print(f"   Using first port: {port}")
                else:
                    raise ValueError("No USB serial ports found")
            else:
                print(f"✅ Port auto-detected: {port}")

        self.port = port
        self.baudrate = baudrate
        self.motor_ids = motor_ids or []
        self.calibration = calibration or {}  # motor_id -> MotorCalibration

        # Feetech SDK objects
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
            self.port_handler = scs.PortHandler(self.port)
            self.packet_handler = scs.PacketHandler(self.PROTOCOL_VERSION)

            # Open port
            if not self.port_handler.openPort():
                raise Exception(f"Failed to open port: {self.port}")

            # Configure baudrate
            if not self.port_handler.setBaudRate(self.baudrate):
                raise Exception(f"Failed to set baudrate: {self.baudrate}")

            print(f"✓ Port connected successfully: {self.port} @ {self.baudrate} bps")

            # Check each motor
            found_motors = []
            for motor_id in self.motor_ids:
                # Ping test
                scs_model_number, scs_comm_result, scs_error = self.packet_handler.ping(
                    self.port_handler, motor_id
                )

                if scs_comm_result == scs.COMM_SUCCESS:
                    print(f"✓ Motor {motor_id} found (model: {scs_model_number})")
                    found_motors.append(motor_id)
                else:
                    print(f"⚠️  Motor {motor_id} not responding")

            if not found_motors:
                raise Exception("No motors found")

            self.is_connected = True
            self.motor_ids = found_motors

            # LeRobot approach: ONLY ping motors, DO NOT WRITE ANYTHING!
            # Calibration must be pre-written to motor EEPROM
            # Goal_Position will be set when enable_torque() is called

            # Read initial positions (read raw values if no calibration)
            if self.calibration:
                # If calibration exists, read normalized values
                self.current_positions = self.read_positions(normalize=True)
            else:
                # If no calibration, read raw values (during calibration process)
                self.current_positions = self.read_positions_raw().astype(np.float32)

            self.target_positions = self.current_positions.copy()

            print(f"✓ {len(found_motors)} motors initialized")
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
            # Disable all motor torques
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
        Enable motor torque (LeRobot approach)

        Args:
            motor_ids: List of motor IDs to enable. None = all motors

        Note:
            Sets Goal_Position to current position BEFORE enabling torque
            to prevent sudden movement.
        """
        if not self.is_connected:
            raise Exception("Motors not connected")

        ids_to_enable = motor_ids if motor_ids is not None else self.motor_ids

        import time

        # CRITICAL: Set Goal_Position to current position BEFORE enabling torque
        print(f"📍 Initializing Goal_Position before enabling torque...")
        for motor_id in ids_to_enable:
            try:
                # Read current position
                current_pos = self._read_2byte(motor_id, self.ADDR_PRESENT_POSITION)

                # Write as goal position
                self._write_2byte(motor_id, self.ADDR_GOAL_POSITION, current_pos)

                # CRITICAL: Wait for write to complete
                time.sleep(0.05)

                # Verify it was written correctly
                goal_readback = self._read_2byte(motor_id, self.ADDR_GOAL_POSITION)

                if abs(goal_readback - current_pos) > 5:
                    print(f"  ⚠️  Motor {motor_id}: Goal mismatch! Present={current_pos}, Goal={goal_readback}")
                else:
                    print(f"  ✓ Motor {motor_id}: Present={current_pos}, Goal={goal_readback}")

            except Exception as e:
                print(f"  ✗ Motor {motor_id} Goal_Position initialization failed: {e}")

        # Additional delay before enabling torque
        time.sleep(0.1)

        # Now enable torque + Lock
        for motor_id in ids_to_enable:
            self._write_1byte(motor_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            self._write_1byte(motor_id, self.ADDR_LOCK, 1)

        print(f"✓ Torque enabled: {ids_to_enable}")

    def disable_torque(self, motor_ids: Optional[List[int]] = None):
        """
        Disable motor torque (motors can be moved by hand)

        Args:
            motor_ids: List of motor IDs to disable. None = all motors

        Note:
            Also disables Lock register (LeRobot standard).
        """
        if not self.is_connected:
            raise Exception("Motors not connected")

        ids_to_disable = motor_ids if motor_ids is not None else self.motor_ids

        # LeRobot approach: Disable torque + Lock
        for motor_id in ids_to_disable:
            self._write_1byte(motor_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            self._write_1byte(motor_id, self.ADDR_LOCK, 0)

        # CRITICAL: Wait for motors to process lock disable before EEPROM write
        import time
        time.sleep(0.1)

        print(f"✓ Torque disabled: {ids_to_disable}")

    def torque_disabled(self, motor_ids: Optional[List[int]] = None):
        """
        Context manager that temporarily disables torque (LeRobot standard)

        Usage:
            with controller.torque_disabled():
                # Write to EEPROM safely
                controller.write_calibration()

        Returns:
            Context manager
        """
        from contextlib import contextmanager

        @contextmanager
        def _context():
            self.disable_torque(motor_ids)
            try:
                yield
            finally:
                self.enable_torque(motor_ids)

        return _context()

    def set_homing_offset(self, motor_id: int, offset: int):
        """
        Set Homing Offset (stored in motor firmware)

        Feetech: Present_Position = Actual_Position - Homing_Offset

        Args:
            motor_id: Motor ID
            offset: Homing Offset value (signed, -2047 to 2047)

        Note:
            Homing Offset is in EEPROM area, so torque must be disabled to write.
            Uses 11-bit sign-magnitude encoding (LeRobot standard).
        """
        if not self.is_connected:
            raise Exception("Motors not connected")

        # Validate range for 11-bit magnitude
        max_magnitude = (1 << 11) - 1  # 2047
        if abs(offset) > max_magnitude:
            raise ValueError(f"Homing Offset out of range: {offset} (max magnitude: {max_magnitude})")

        # Encode using 11-bit sign-magnitude (LeRobot standard)
        encoded = self._encode_sign_magnitude(offset, sign_bit_index=11)

        # Write to EEPROM (torque should be disabled)
        self._write_2byte(motor_id, self.ADDR_HOMING_OFFSET, encoded)

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

        # Read raw value
        encoded = self._read_2byte(motor_id, self.ADDR_HOMING_OFFSET)

        # Decode using 11-bit sign-magnitude (LeRobot standard)
        return self._decode_sign_magnitude(encoded, sign_bit_index=11)

    def set_center_position(self) -> Dict[int, int]:
        """
        Set current physical position as center (2047)

        LeRobot approach: Present_Position = Actual_Position - Homing_Offset
        To make Present = 2047, we need: Offset = Actual - 2047

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

        # First, reset homing offsets to 0 to get actual positions
        print("  Resetting homing offsets to 0...")
        for motor_id in self.motor_ids:
            self.set_homing_offset(motor_id, 0)

        # Small delay for motors to update
        import time
        time.sleep(0.05)

        # Now read actual positions and calculate offsets
        for motor_id in self.motor_ids:
            # Read actual physical position (with offset=0)
            actual_pos = self._read_2byte(motor_id, self.ADDR_PRESENT_POSITION)

            # 12-bit encoder center = 2047 (half of 4096 - 1)
            # Feetech formula: Present = Actual - Offset
            # To make Present = 2047: Offset = Actual - 2047
            center = 2047
            offset = actual_pos - center

            # Set Homing Offset
            self.set_homing_offset(motor_id, offset)

            offsets[motor_id] = offset
            print(f"  Motor {motor_id}: Actual {actual_pos} → Present {center} (offset={offset})")

        return offsets

    def write_calibration(self):
        """
        Write calibration to motors (LeRobot compatible)

        Writes homing_offset and position limits to motor EEPROM.
        IMPORTANT: Must be called with torque DISABLED!

        Usage:
            with controller.torque_disabled():
                controller.write_calibration()
        """
        if not self.is_connected:
            raise Exception("Motors not connected")

        if not self.calibration:
            print("⚠️  No calibration data")
            return

        import time

        for motor_id, calib in self.calibration.items():
            try:
                # Write homing offset (Protocol 0 only)
                self.set_homing_offset(motor_id, calib.homing_offset)
                time.sleep(0.05)  # Wait for EEPROM write

                # Write position limits
                self._write_2byte(motor_id, self.ADDR_MIN_POSITION_LIMIT, calib.range_min)
                time.sleep(0.05)  # Wait for EEPROM write

                self._write_2byte(motor_id, self.ADDR_MAX_POSITION_LIMIT, calib.range_max)
                time.sleep(0.05)  # Wait for EEPROM write

                print(f"  ✓ Motor {motor_id}: offset={calib.homing_offset}, range=[{calib.range_min}, {calib.range_max}]")
            except Exception as e:
                print(f"  ✗ Motor {motor_id} failed: {e}")

    def write_calibration_to_motors(self):
        """
        Write calibration data from self.calibration to motor EEPROM

        LeRobot approach: Apply homing_offset and position limits to motors

        Note:
            - Must be called with torque DISABLED (EEPROM area)
            - This makes motors use the calibration values
            - CRITICAL: After writing homing_offset, Present_Position changes immediately!
              We must update Goal_Position to the NEW Present_Position to prevent movement.
        """
        if not self.is_connected:
            raise Exception("Motors not connected")

        if not self.calibration:
            print("⚠️  No calibration data to write")
            return

        print("\n📝 Writing calibration to motor EEPROM...")

        for motor_id, calib in self.calibration.items():
            try:
                # Write position limits first (unsigned values)
                self._write_2byte(motor_id, self.ADDR_MIN_POSITION_LIMIT, calib.range_min)
                self._write_2byte(motor_id, self.ADDR_MAX_POSITION_LIMIT, calib.range_max)

                # Write homing offset
                # WARNING: This immediately changes Present_Position!
                # Formula: Present_Position = Actual_Position - Homing_Offset
                self.set_homing_offset(motor_id, calib.homing_offset)

                # CRITICAL: Read the NEW Present_Position and set it as Goal_Position
                # to prevent motor from moving when torque is enabled
                import time
                time.sleep(0.01)  # Small delay for motor to update
                new_present = self._read_2byte(motor_id, self.ADDR_PRESENT_POSITION)
                self._write_2byte(motor_id, self.ADDR_GOAL_POSITION, new_present)

                print(f"  ✓ Motor {motor_id}: offset={calib.homing_offset}, range=[{calib.range_min}, {calib.range_max}], present={new_present}")
            except Exception as e:
                print(f"  ✗ Motor {motor_id} failed: {e}")

        print("✓ Calibration written to motors\n")

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

            # Apply drive_mode (Feetech uses this, Dynamixel doesn't)
            if calib.drive_mode == 1:
                normalized[i] = -normalized[i]

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

            # Apply drive_mode BEFORE unnormalization (Feetech)
            bounded_norm = min(100.0, max(-100.0, pos_norm))
            if calib.drive_mode == 1:
                bounded_norm = -bounded_norm

            # LeRobot formula for RANGE_M100_100
            raw_val = int(((bounded_norm + 100) / 200) * (max_val - min_val) + min_val)

            # CRITICAL: Clip to range to prevent overflow/wrapping
            raw[i] = max(min_val, min(max_val, raw_val))

        return raw

    def read_position_limits(self):
        """Read position limits (for debugging)"""
        print("\n=== Position Limits (Hardware) ===")
        for motor_id in self.motor_ids:
            try:
                min_limit = self._read_2byte(motor_id, self.ADDR_MIN_POSITION_LIMIT)
                max_limit = self._read_2byte(motor_id, self.ADDR_MAX_POSITION_LIMIT)
                print(f"motor {motor_id}: Min={min_limit}, Max={max_limit}")
            except Exception as e:
                print(f"motor {motor_id}: Read failed - {e}")

    def read_homing_offsets(self):
        """Read homing offsets"""
        print("\n=== Homing Offsets ===")
        offsets = []
        for motor_id in self.motor_ids:
            try:
                offset = self._read_2byte_signed(motor_id, self.ADDR_HOMING_OFFSET)
                offsets.append(offset)
                print(f"motor {motor_id}: Offset={offset}")
            except Exception as e:
                print(f"motor {motor_id}: Read failed - {e}")
                offsets.append(0)
        return np.array(offsets, dtype=np.float32)

    def set_homing_offsets_from_current(self):
        """
        Set current position as homing offset

        Warning: This operation can cause motor movement, use with caution!
        Present_Position = Actual_Position - Homing_Offset

        Safe procedure:
        1. Save current Goal_Position
        2. Configure Homing Offset
        3. Reconfigure Goal_Position to 0 (maintain current position)
        """
        print("\n⚙️  Setting current position as homing offset...")

        # Read current absolute position (unsigned, before Homing Offset applied)
        current_positions = self.read_positions(use_sign_magnitude=False)

        for i, motor_id in enumerate(self.motor_ids):
            try:
                current_pos = int(current_positions[i])

                # Disable torque (for EEPROM write)
                self._write_1byte(motor_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)

                # Configure Homing Offset (unsigned value, 0~4095)
                self._write_2byte(motor_id, self.ADDR_HOMING_OFFSET, current_pos)

                # Re-enable torque
                self._write_1byte(motor_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)

                # Read current Present_Position after enabling torque (now 0)
                time.sleep(0.01)  # Wait for register update
                new_present = self._read_2byte(motor_id, self.ADDR_PRESENT_POSITION)

                # Configure Goal Position to current Present_Position
                # Present = Actual - Offset = current_pos - current_pos = 0
                # Goal = 0 (Present coordinate system) → Actual = current_pos (no movement)
                self._write_2byte(motor_id, self.ADDR_GOAL_POSITION, new_present)

                print(f"  motor {motor_id}: Offset={current_pos}, Goal=0 configured")
            except Exception as e:
                print(f"  motor {motor_id}: Configuration failed - {e}")

        print("✓ Homing Offset configured (current position is now 0)")

    def set_position_limits(self, min_limit: int, max_limit: int):
        """
        Configure position limits (EEPROM write)

        Note: Must disable torque to write to EEPROM
        """
        print(f"\n⚙️  Configuring position limits: {min_limit} ~ {max_limit}")

        for motor_id in self.motor_ids:
            try:
                # Disable torque
                self._write_1byte(motor_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)

                # Configure position limits (unsigned)
                self._write_2byte(motor_id, self.ADDR_MIN_POSITION_LIMIT, min_limit)
                self._write_2byte(motor_id, self.ADDR_MAX_POSITION_LIMIT, max_limit)

                # Re-enable torque
                self._write_1byte(motor_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)

                print(f"  motor {motor_id}: Configuration complete")
            except Exception as e:
                print(f"  motor {motor_id}: Configuration failed - {e}")

    def read_positions(self, normalize: bool = True, use_sign_magnitude: bool = False) -> np.ndarray:
        """
        Read current motor positions (LeRobot-compatible)

        Args:
            normalize: If True (default), return normalized values (-100~+100).
                      If False, return raw values (0~4095, legacy mode).
            use_sign_magnitude: True for Sign-Magnitude decoding (rarely used)

        Returns:
            Normalized values (-100~+100) by default
        """
        if not self.is_connected:
            return self.current_positions

        # Read raw values
        positions_raw = np.zeros(len(self.motor_ids), dtype=np.int32)
        for i, motor_id in enumerate(self.motor_ids):
            try:
                if use_sign_magnitude:
                    position_raw = self._read_2byte_signed(motor_id, self.ADDR_PRESENT_POSITION)
                else:
                    position_raw = self._read_2byte(motor_id, self.ADDR_PRESENT_POSITION)
                positions_raw[i] = position_raw
            except:
                pass

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

        Used during calibration

        Returns:
            Raw position values for each motor (0~4095)
        """
        return self.read_positions(normalize=False, use_sign_magnitude=False)

    def write_positions(self, positions: np.ndarray, normalize: bool = True, use_sign_magnitude: bool = False):
        """
        Write target positions (LeRobot-compatible)

        Args:
            positions: Target position array
            normalize: If True (default), positions are normalized values (-100~+100).
                      If False, positions are raw values (0~4095, legacy mode).
            use_sign_magnitude: True for Sign-Magnitude encoding (rarely used)
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

        # Default movement parameters (required for motors to actually move!)
        # Feetech servos need Goal_Velocity and Acceleration to be non-zero
        DEFAULT_VELOCITY = 1500  # Medium speed (0-4095 range)
        DEFAULT_ACCELERATION = 50  # Medium acceleration (0-254 range)

        for i, motor_id in enumerate(self.motor_ids):
            position_raw = int(positions_raw[i])

            # Debug log for motor 6 (gripper)
            if motor_id == 6:
                print(f"[DEBUG] Motor 6: norm={positions[i]:.2f}, raw_before_clip={position_raw}, ", end="")

            # Set movement parameters BEFORE writing Goal_Position
            # This is CRITICAL - without these, motors won't move!
            self._write_2byte(motor_id, self.ADDR_GOAL_VELOCITY, DEFAULT_VELOCITY)
            self._write_1byte(motor_id, self.ADDR_ACCELERATION, DEFAULT_ACCELERATION)

            if use_sign_magnitude:
                # Sign-Magnitude encoding
                self._write_2byte_signed(motor_id, self.ADDR_GOAL_POSITION, position_raw)
            else:
                # Clip to valid range (0~4095)
                position_raw_clipped = max(0, min(4095, position_raw))
                if motor_id == 6:
                    print(f"raw_after_clip={position_raw_clipped}, vel={DEFAULT_VELOCITY}, acc={DEFAULT_ACCELERATION}")
                # Write unsigned value
                self._write_2byte(motor_id, self.ADDR_GOAL_POSITION, position_raw_clipped)

    def _read_1byte(self, motor_id: int, address: int) -> int:
        """Read 1 byte"""
        data, result, error = self.packet_handler.read1ByteTxRx(
            self.port_handler, motor_id, address
        )
        if result != scs.COMM_SUCCESS:
            raise Exception(f"Read failed (motor {motor_id}, address {address})")
        return data

    def _read_2byte(self, motor_id: int, address: int) -> int:
        """Read 2 bytes"""
        data, result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, motor_id, address
        )
        if result != scs.COMM_SUCCESS:
            raise Exception(f"Read failed (motor {motor_id}, address {address})")
        return data

    def _encode_sign_magnitude(self, value: int, sign_bit_index: int) -> int:
        """
        Encode signed integer to sign-magnitude format (LeRobot standard)

        Args:
            value: Signed integer value
            sign_bit_index: Position of sign bit (e.g., 11 for Homing_Offset)

        Returns:
            Unsigned integer in sign-magnitude format
        """
        max_magnitude = (1 << sign_bit_index) - 1
        magnitude = abs(value)

        if magnitude > max_magnitude:
            raise ValueError(f"Magnitude {magnitude} exceeds {max_magnitude} (max for sign_bit_index={sign_bit_index})")

        direction_bit = 1 if value < 0 else 0
        return (direction_bit << sign_bit_index) | magnitude

    def _decode_sign_magnitude(self, encoded_value: int, sign_bit_index: int) -> int:
        """
        Decode sign-magnitude format to signed integer (LeRobot standard)

        Args:
            encoded_value: Unsigned integer in sign-magnitude format
            sign_bit_index: Position of sign bit (e.g., 11 for Homing_Offset)

        Returns:
            Signed integer value
        """
        direction_bit = (encoded_value >> sign_bit_index) & 1
        magnitude_mask = (1 << sign_bit_index) - 1
        magnitude = encoded_value & magnitude_mask
        return -magnitude if direction_bit else magnitude

    def _write_1byte(self, motor_id: int, address: int, value: int):
        """Write 1 byte"""
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, address, value
        )
        if result != scs.COMM_SUCCESS:
            raise Exception(f"Write failed (motor {motor_id}, address {address})")

    def _write_2byte(self, motor_id: int, address: int, value: int):
        """Write 2 bytes"""
        result, error = self.packet_handler.write2ByteTxRx(
            self.port_handler, motor_id, address, value
        )
        if result != scs.COMM_SUCCESS:
            err_msg = self.packet_handler.getTxRxResult(result)
            raise Exception(f"Write failed (motor {motor_id}, address {address}, value {value}): {err_msg}")

    def get_observation(self) -> Dict[str, np.ndarray]:
        """
        Return current observation (ROS2 compatible)

        Returns:
            {"observation.state": positions}
        """
        positions = self.read_positions()
        return {
            "observation.state": positions
        }

    def send_action(self, action: Dict[str, np.ndarray]):
        """
        Send action (ROS2 compatible)

        Args:
            action: {"action": positions}
        """
        if "action" in action:
            self.write_positions(action["action"])
        else:
            raise ValueError("Action dict missing 'action' key")

    def __del__(self):
        """Destructor"""
        self.disconnect()


def test_feetech():
    """Test code"""
    print("Feetech test starting...")

    # SO-101 configuration: 6 motors (ID 1-6)
    PORT = "/dev/ttyACM0"
    BAUDRATE = 1000000
    MOTOR_IDS = [1, 2, 3, 4, 5, 6]

    # Create controller
    controller = FeetechController(PORT, BAUDRATE, MOTOR_IDS)

    # Connect
    if not controller.connect():
        print("❌ Connection failed")
        return

    # Read current positions
    positions = controller.read_positions()
    print(f"\nCurrent positions (raw): {positions}")

    # Disconnect
    controller.disconnect()
    print("\n✓ Test complete")


if __name__ == "__main__":
    test_feetech()
