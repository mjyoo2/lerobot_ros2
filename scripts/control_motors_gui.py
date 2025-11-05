#!/usr/bin/env python3
"""
Motor Control GUI

Script for controlling motors via GUI.
Independent implementation (does not use lerobot package)

Usage:
    python scripts/control_motors_gui.py --config configs/robot/so100_config.yaml
"""

import argparse
import sys
import math
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

try:
    import pygame
except ImportError:
    print("Error: pygame not found. Install it: pip install pygame")
    sys.exit(1)

from lerobot_ros2.configs import load_config
from lerobot_ros2.hardware import DynamixelController, FeetechController
from lerobot_ros2.hardware.calibration import CalibrationManager

# GUI configuration
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
BG_COLOR = (30, 30, 30)
TEXT_COLOR = (250, 250, 250)
SLIDER_BG = (60, 60, 60)
SLIDER_FG = (100, 200, 100)
SLIDER_HANDLE = (240, 240, 240)
CURRENT_POS_COLOR = (250, 220, 40)
BUTTON_COLOR = (80, 80, 80)
BUTTON_HOVER = (110, 110, 110)
BUTTON_ACTIVE = (60, 200, 60)

SLIDER_HEIGHT = 40
SLIDER_MARGIN = 10
BUTTON_WIDTH = 120
BUTTON_HEIGHT = 40
FPS = 30


class MotorSlider:
    """Individual motor control slider (LeRobot normalized values)"""

    def __init__(self, name, motor_id, x, y, width, min_raw, max_raw, current_norm):
        self.name = name
        self.motor_id = motor_id
        self.x = x
        self.y = y
        self.width = width
        self.height = SLIDER_HEIGHT
        self.min_raw = min_raw  # For display only
        self.max_raw = max_raw  # For display only
        self.target_norm = current_norm  # Normalized value (-100 to +100)
        self.current_norm = current_norm  # Normalized value (-100 to +100)

        self.dragging = False
        self.slider_rect = pygame.Rect(x, y, width, SLIDER_HEIGHT)

    def handle_event(self, event):
        """Handle events"""
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.slider_rect.collidepoint(event.pos):
                self.dragging = True
                self._update_value_from_pos(event.pos[0])

        elif event.type == pygame.MOUSEBUTTONUP:
            self.dragging = False

        elif event.type == pygame.MOUSEMOTION:
            if self.dragging:
                self._update_value_from_pos(event.pos[0])

    def _update_value_from_pos(self, mouse_x):
        """Update value from mouse position (normalized -100~+100)"""
        relative_x = mouse_x - self.x
        relative_x = max(0, min(relative_x, self.width))

        ratio = relative_x / self.width
        # Map to normalized range (-100 to +100)
        self.target_norm = -100.0 + ratio * 200.0

    def update_current_position(self, norm):
        """Update current position (normalized value)"""
        self.current_norm = norm

    def draw(self, screen, font):
        """Draw to screen"""
        # Motor name
        name_surf = font.render(f"{self.name} (ID: {self.motor_id})", True, TEXT_COLOR)
        screen.blit(name_surf, (self.x, self.y - 25))

        # Slider background
        pygame.draw.rect(screen, SLIDER_BG, self.slider_rect, border_radius=5)

        # Display current position (yellow line) - normalized -100~+100 to 0~1
        current_ratio = (self.current_norm + 100.0) / 200.0
        current_x = self.x + int(current_ratio * self.width)
        pygame.draw.line(
            screen,
            CURRENT_POS_COLOR,
            (current_x, self.y),
            (current_x, self.y + self.height),
            3,
        )

        # Display target position (green handle) - normalized -100~+100 to 0~1
        target_ratio = (self.target_norm + 100.0) / 200.0
        handle_x = self.x + int(target_ratio * self.width)
        handle_y = self.y + self.height // 2

        # Handle circle
        pygame.draw.circle(screen, SLIDER_HANDLE, (handle_x, handle_y), 12)
        pygame.draw.circle(screen, SLIDER_FG, (handle_x, handle_y), 10)

        # Display values (normalized and raw values)
        # Convert normalized to approximate raw for display
        current_raw_approx = int((self.current_norm + 100) / 200 * (self.max_raw - self.min_raw) + self.min_raw)
        target_raw_approx = int((self.target_norm + 100) / 200 * (self.max_raw - self.min_raw) + self.min_raw)

        val_text = f"Target: {self.target_norm:+6.1f} (~{target_raw_approx:4d}) | Current: {self.current_norm:+6.1f} (~{current_raw_approx:4d})"
        val_surf = font.render(val_text, True, TEXT_COLOR)
        screen.blit(val_surf, (self.x, self.y + self.height + 5))


class MotorControlGUI:
    """Motor control GUI"""

    def __init__(self, config_path):
        # Load configuration
        self.robot_config = load_config(config_path)

        # Initialize pygame
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("Motor Control GUI")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 18)

        # Controller
        self.controller = None
        self.sliders = []
        self.connected = False
        self.torque_enabled = False

        # Try loading calibration
        self.calibrations = {}
        self._load_calibration()

        # Buttons
        self._setup_buttons()

    def _setup_buttons(self):
        """Button configuration"""
        btn_y = SCREEN_HEIGHT - 60
        self.connect_btn = pygame.Rect(20, btn_y, BUTTON_WIDTH, BUTTON_HEIGHT)
        self.torque_btn = pygame.Rect(150, btn_y, BUTTON_WIDTH, BUTTON_HEIGHT)
        self.center_btn = pygame.Rect(280, btn_y, BUTTON_WIDTH, BUTTON_HEIGHT)
        self.quit_btn = pygame.Rect(SCREEN_WIDTH - BUTTON_WIDTH - 20, btn_y, BUTTON_WIDTH, BUTTON_HEIGHT)

    def _load_calibration(self):
        """Load calibration"""
        calibration_dir = Path(self.robot_config.get("calibration_dir", "./calibration"))
        manager = CalibrationManager(calibration_dir)

        try:
            self.calibrations = manager.load()
            print(f"✓ Calibration loaded: {len(self.calibrations)} motors")
            return True
        except FileNotFoundError:
            print("⚠ No calibration file found")
            print("  Please run: python scripts/calibrate_motors.py --config <your_config.yaml>")
            self.calibrations = {}
            return False

    def _create_controller(self):
        """Create controller (with calibration)"""
        port = self.robot_config.get("port", "/dev/ttyUSB0")
        baudrate = self.robot_config.get("baudrate", 1000000)
        motor_type = self.robot_config.get("motor_type", "dynamixel").lower()

        motor_ids = [motor["id"] for motor in self.robot_config["motors"].values()]

        # Convert calibration from motor_name -> motor_id mapping
        calibration_by_id = {}
        if self.calibrations:
            for motor_name, calib in self.calibrations.items():
                calibration_by_id[calib.motor_id] = calib

        if motor_type == "feetech":
            self.controller = FeetechController(
                port=port,
                baudrate=baudrate,
                motor_ids=motor_ids,
                calibration=calibration_by_id
            )
        else:
            self.controller = DynamixelController(
                port=port,
                baudrate=baudrate,
                motor_ids=motor_ids,
                calibration=calibration_by_id
            )

    def _create_sliders(self):
        """Create sliders (LeRobot normalized values)"""
        self.sliders = []

        slider_y = 80
        slider_width = SCREEN_WIDTH - 40

        # Read current position (normalized values -100~+100)
        try:
            current_positions = self.controller.read_positions()  # normalize=True by default
        except Exception as e:
            print(f"   ✗ Failed to read positions: {e}")
            raise

        for i, (motor_name, motor_info) in enumerate(self.robot_config["motors"].items()):
            motor_id = motor_info["id"]

            # Calibration range (raw values, for display)
            if motor_name in self.calibrations:
                calib = self.calibrations[motor_name]
                min_raw = calib.range_min
                max_raw = calib.range_max
            else:
                # Default range
                min_raw = 0
                max_raw = 4095

            current_norm = current_positions[i]

            slider = MotorSlider(
                name=motor_name,
                motor_id=motor_id,
                x=20,
                y=slider_y,
                width=slider_width,
                min_raw=min_raw,
                max_raw=max_raw,
                current_norm=current_norm,
            )
            self.sliders.append(slider)

            slider_y += SLIDER_HEIGHT + SLIDER_MARGIN + 30

    def connect(self):
        """Connect to motors"""
        try:
            # Check calibration
            if not self.calibrations:
                print("✗ Cannot connect: No calibration data loaded")
                print("  Please run calibration first:")
                print("  python scripts/calibrate_motors.py --config <your_config.yaml>")
                return

            print("Connecting to motors...")
            self._create_controller()
            self.controller.connect()
            self.connected = True
            self._create_sliders()
            print("✓ Connected successfully")
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            self.connected = False

    def disconnect(self):
        """Disconnect from motors"""
        if self.controller and self.connected:
            self.controller.disconnect()
            self.connected = False
            print("✓ Disconnected")

    def toggle_torque(self):
        """Toggle torque"""
        if not self.connected:
            return

        try:
            if self.torque_enabled:
                self.controller.disable_torque()
                self.torque_enabled = False
                print("✓ Torque disabled")
            else:
                self.controller.enable_torque()
                self.torque_enabled = True

                # CRITICAL: Reset slider targets to current positions
                # This prevents motors from moving to old slider positions
                current_positions = self.controller.read_positions()
                for i, slider in enumerate(self.sliders):
                    slider.target_norm = current_positions[i]

                print("✓ Torque enabled")
        except Exception as e:
            print(f"✗ Toggle torque failed: {e}")

    def move_to_center(self):
        """Move to center position (normalized 0 = calibration midpoint)"""
        if not self.connected or not self.torque_enabled:
            return

        try:
            import numpy as np
            # Move all motors to 0 (normalized, calibration center)
            num_motors = len(self.robot_config["motors"])
            positions = np.zeros(num_motors, dtype=np.float32)
            self.controller.write_positions(positions)  # normalize=True by default

            # Update sliders
            for slider in self.sliders:
                slider.target_norm = 0.0

            print("✓ Moving to center position")
        except Exception as e:
            print(f"✗ Move to center failed: {e}")

    def update(self):
        """Update state (LeRobot normalized values)"""
        if not self.connected:
            return

        try:
            import numpy as np

            # Always read current position (if connected)
            current_positions = self.controller.read_positions()  # normalize=True by default
            for i, slider in enumerate(self.sliders):
                slider.update_current_position(current_positions[i])

            # LeRobot approach: Only write Goal_Position while dragging
            # Do NOT write every frame - this causes jitter and instability
            if self.torque_enabled:
                # Build target positions array from sliders
                target_positions = np.array([slider.target_norm for slider in self.sliders], dtype=np.float32)

                # Check if any slider changed (dragging or just changed)
                # Only write if there's a significant difference
                position_changed = False
                for i, (current, target) in enumerate(zip(current_positions, target_positions)):
                    if abs(current - target) > 1.0:  # 1.0 normalized units threshold
                        position_changed = True
                        break

                if position_changed:
                    self.controller.write_positions(target_positions)

        except Exception as e:
            print(f"✗ Update failed: {e}")

    def draw_button(self, rect, text, active=False):
        """Draw button"""
        mouse_pos = pygame.mouse.get_pos()
        is_hover = rect.collidepoint(mouse_pos)

        if active:
            color = BUTTON_ACTIVE
        elif is_hover:
            color = BUTTON_HOVER
        else:
            color = BUTTON_COLOR

        pygame.draw.rect(self.screen, color, rect, border_radius=5)

        text_surf = self.font.render(text, True, TEXT_COLOR)
        text_rect = text_surf.get_rect(center=rect.center)
        self.screen.blit(text_surf, text_rect)

    def draw(self):
        """Draw screen"""
        self.screen.fill(BG_COLOR)

        # Title
        title = self.font.render("Motor Control GUI", True, TEXT_COLOR)
        self.screen.blit(title, (20, 20))

        # Status display
        status_text = f"Status: {'Connected' if self.connected else 'Disconnected'}"
        if self.connected:
            status_text += f" | Torque: {'ON' if self.torque_enabled else 'OFF'}"

        status = self.small_font.render(status_text, True, TEXT_COLOR)
        self.screen.blit(status, (20, 50))

        # Sliders
        for slider in self.sliders:
            slider.draw(self.screen, self.small_font)

        # Buttons
        self.draw_button(self.connect_btn, "Connect" if not self.connected else "Disconnect")
        self.draw_button(self.torque_btn, "Torque", active=self.torque_enabled)
        self.draw_button(self.center_btn, "Center")
        self.draw_button(self.quit_btn, "Quit")

        pygame.display.flip()

    def handle_events(self):
        """Handle events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False

            # Button clicks
            if event.type == pygame.MOUSEBUTTONDOWN:
                if self.connect_btn.collidepoint(event.pos):
                    if self.connected:
                        self.disconnect()
                    else:
                        self.connect()

                elif self.torque_btn.collidepoint(event.pos):
                    self.toggle_torque()

                elif self.center_btn.collidepoint(event.pos):
                    self.move_to_center()

                elif self.quit_btn.collidepoint(event.pos):
                    return False

            # Slider events
            for slider in self.sliders:
                slider.handle_event(event)

        return True

    def run(self):
        """Main loop"""
        print("\n" + "="*60)
        print("Motor Control GUI (LeRobot-compatible)")
        print("="*60)
        print("\nInstructions:")
        print("  1. Click 'Connect' to connect to motors")
        print("  2. Click 'Torque' to enable motor control")
        print("  3. Drag sliders to control motor positions")
        print("     • Sliders use normalized values (-100 to +100)")
        print("     • 0 = calibration midpoint, ±100 = calibration limits")
        print("  4. Yellow line: current position")
        print("  5. Green circle: target position")
        print("  6. Click 'Center' to move all motors to neutral (0)")
        print("  7. Click 'Quit' or close window to exit")
        if not self.calibrations:
            print("\n❌ ERROR: No calibration data found!")
            print("   You MUST run calibration before using this GUI:")
            print("   python scripts/calibrate_motors.py --config <your_config.yaml>")
            print("\n   Press Ctrl+C to exit")
        else:
            print(f"\n✓ Calibration loaded for {len(self.calibrations)} motors")
        print()

        running = True
        while running:
            running = self.handle_events()
            self.update()
            self.draw()
            self.clock.tick(FPS)

        self.disconnect()
        pygame.quit()


def main():
    parser = argparse.ArgumentParser(description="Motor control GUI")
    parser.add_argument(
        "--config",
        type=str,
        required=True,
        help="Robot config file path",
    )

    args = parser.parse_args()

    # Run GUI
    try:
        gui = MotorControlGUI(args.config)
        gui.run()
    except KeyboardInterrupt:
        print("\n\nGUI terminated by user")
    except Exception as e:
        print(f"\n\nError: {e}")
        raise


if __name__ == "__main__":
    main()
