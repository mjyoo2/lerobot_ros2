#!/usr/bin/env python3
"""
Motor Registration and Discovery Script

Automatically discovers and registers connected motors.
Self-implemented (does not use lerobot package, for reference only)

Usage:
    # Find ports
    python scripts/register_motors.py --find-port

    # Scan motors on specific port
    python scripts/register_motors.py --port /dev/ttyUSB0 --scan

    # Generate motor configuration
    python scripts/register_motors.py --port /dev/ttyUSB0 --scan --output configs/robot/my_robot.yaml
"""

import argparse
import sys
from pathlib import Path
import yaml

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from lerobot_ros2.hardware.find_port import get_available_ports, get_port_info, test_port_connection
from lerobot_ros2.hardware.dynamixel_controller import DynamixelController
try:
    from lerobot_ros2.hardware.feetech_controller import FeetechController
    FEETECH_AVAILABLE = True
except ImportError:
    FEETECH_AVAILABLE = False


def find_ports():
    """Find available serial ports"""
    ports = get_available_ports()

    if not ports:
        print("No serial ports found")
        return []

    print(f"\nFound {len(ports)} serial port(s):")
    for i, port in enumerate(ports, 1):
        info = get_port_info(port)
        print(f"  {i}. {port}")
        print(f"     Description: {info.get('description', 'Unknown')}")
        print(f"     Hardware ID: {info.get('hwid', 'Unknown')}")
        print()

    return ports


def scan_dynamixel_port(port, baudrate=1000000):
    """
    Scan for Dynamixel motors

    Args:
        port: Serial port
        baudrate: Baudrate

    Returns:
        list: [(motor_id, model_name), ...]
    """
    print(f"\nScanning port: {port} (baudrate: {baudrate})")
    print("This may take a moment...")

    try:
        # Connect with temporary controller (no motor_ids)
        controller = DynamixelController(port=port, baudrate=baudrate, motor_ids=[])
        controller.connect()

        # Scan IDs (1-253)
        print("\nScanning motor IDs (1-253)...")
        found_motors = []

        for motor_id in range(1, 254):
            if motor_id % 50 == 0:
                print(f"  Progress: {motor_id}/253...")

            try:
                # Try ping
                dxl_model_number, dxl_comm_result, dxl_error = controller.packet_handler.ping(
                    controller.port_handler,
                    motor_id
                )

                if dxl_comm_result == 0:  # COMM_SUCCESS
                    # Convert model number to model name
                    model_name = _get_model_name_from_number(dxl_model_number, "dynamixel")
                    found_motors.append((motor_id, model_name))
                    print(f"  ✓ Found motor ID {motor_id}: {model_name} (model #{dxl_model_number})")

            except Exception as e:
                pass  # Ignore errors and continue

        controller.disconnect()

        if not found_motors:
            print("\n  No motors found on this port")
        else:
            print(f"\n✓ Found {len(found_motors)} motor(s)")

        return found_motors

    except Exception as e:
        print(f"Error scanning port: {e}")
        return []


def _get_model_name_from_number(model_number, motor_type="dynamixel"):
    """Convert motor model number to name"""
    if motor_type == "feetech":
        # Feetech/SCServo model mapping
        model_map = {
            0: "scs15",
            2307: "sts3215",  # Used in SO-101
            3501: "sms_sts3032",
            # More can be added
        }
    else:
        # Dynamixel model mapping
        model_map = {
            1020: "xl320",
            1060: "xl330-m077",
            1190: "xl330-m288",
            1200: "xl430-w250",
            1030: "xc330-t181",
            1040: "xc330-t288",
            350: "xm430-w350",
            1050: "xc430-w150",
            1080: "xc430-w240",
        }

    return model_map.get(model_number, f"unknown_model_{model_number}")


def scan_feetech_port(port, baudrate=1000000):
    """
    Scan for Feetech motors

    Args:
        port: Serial port
        baudrate: Baudrate

    Returns:
        list: [(motor_id, model_name), ...]
    """
    try:
        from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
    except ImportError:
        print("Error: scservo_sdk not found")
        print("Install: ./run.sh uv sync --extra feetech")
        return []

    print(f"\nScanning port: {port} (baudrate: {baudrate}, type: Feetech)")
    print("This may take a moment...")

    try:
        portHandler = PortHandler(port)
        packetHandler = PacketHandler(1.0)  # Feetech uses protocol 1.0

        if not portHandler.openPort():
            raise Exception(f"Failed to open port: {port}")

        if not portHandler.setBaudRate(baudrate):
            raise Exception(f"Failed to set baudrate: {baudrate}")

        # Scan IDs (1-253)
        print("\nScanning motor IDs (1-253)...")
        found_motors = []

        for motor_id in range(1, 254):
            if motor_id % 50 == 0:
                print(f"  Progress: {motor_id}/253...")

            try:
                # Try ping
                model_number, comm_result, error = packetHandler.ping(portHandler, motor_id)

                if comm_result == COMM_SUCCESS:
                    # Convert model number to model name
                    model_name = _get_model_name_from_number(model_number, "feetech")
                    found_motors.append((motor_id, model_name))
                    print(f"  ✓ Found motor ID {motor_id}: {model_name} (model #{model_number})")

            except Exception as e:
                pass  # Ignore errors and continue

        portHandler.closePort()

        if not found_motors:
            print("\n  No motors found on this port")
        else:
            print(f"\n✓ Found {len(found_motors)} motor(s)")

        return found_motors

    except Exception as e:
        print(f"Error scanning port: {e}")
        return []


def create_robot_config(found_motors, port, baudrate, motor_type="dynamixel"):
    """
    Generate robot configuration from scan results

    Args:
        found_motors: [(motor_id, model_name), ...]
        port: Serial port
        baudrate: Baudrate
        motor_type: Motor type

    Returns:
        dict: Robot configuration
    """
    if not found_motors:
        return None

    # Create motors dictionary
    motors = {}
    for motor_id, model_name in found_motors:
        motor_name = f"motor_{motor_id}"
        motors[motor_name] = {
            "id": motor_id,
            "model": model_name,
        }

    # Configuration dictionary
    config = {
        "robot_type": "custom",
        "motor_type": motor_type,
        "port": port,
        "baudrate": baudrate,
        "motors": motors,
        "calibration_dir": "./calibration/custom_robot",
    }

    return config


def main():
    parser = argparse.ArgumentParser(description="Motor registration and discovery")

    # Port related
    parser.add_argument(
        "--find-port",
        action="store_true",
        help="Find available serial ports",
    )
    parser.add_argument(
        "--port",
        type=str,
        help="Serial port (e.g., /dev/ttyUSB0)",
    )

    # Scan related
    parser.add_argument(
        "--scan",
        action="store_true",
        help="Scan for motors on port",
    )
    parser.add_argument(
        "--motor-type",
        type=str,
        choices=["dynamixel", "feetech"],
        default="dynamixel",
        help="Motor type (Default: dynamixel)",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=1000000,
        help="Baudrate (Default: 1000000)",
    )

    # Output related
    parser.add_argument(
        "--output",
        type=str,
        help="Config file save path (e.g., configs/robot/my_robot.yaml)",
    )

    args = parser.parse_args()

    # Find ports
    if args.find_port:
        ports = find_ports()
        if not ports:
            sys.exit(1)

        print("\nTo scan a port, run:")
        print(f"  python {sys.argv[0]} --port <PORT> --scan")
        return

    # Scan
    if args.scan:
        if not args.port:
            print("Error: --port is required for scanning")
            sys.exit(1)

        # Scan motors
        if args.motor_type == "dynamixel":
            found_motors = scan_dynamixel_port(args.port, args.baudrate)
        elif args.motor_type == "feetech":
            if not FEETECH_AVAILABLE:
                print("Error: Feetech support not available")
                print("Install: ./run.sh uv sync --extra feetech")
                sys.exit(1)
            found_motors = scan_feetech_port(args.port, args.baudrate)
        else:
            print(f"Error: Unknown motor type: {args.motor_type}")
            sys.exit(1)

        if not found_motors:
            print("\nNo motors found on this port")
            sys.exit(1)

        # Generate configuration
        config = create_robot_config(found_motors, args.port, args.baudrate, args.motor_type)

        if config:
            print("\n" + "="*60)
            print("Generated Robot Config:")
            print("="*60)
            print(yaml.dump(config, default_flow_style=False, sort_keys=False))

            # Save to file
            if args.output:
                output_path = Path(args.output)
                output_path.parent.mkdir(parents=True, exist_ok=True)

                with open(output_path, "w") as f:
                    yaml.dump(config, f, default_flow_style=False, sort_keys=False)

                print(f"\n✓ Config saved to: {output_path}")
                print("\nNext steps:")
                print(f"  1. Edit the config file to set correct motor names")
                print(f"  2. Run calibration:")
                print(f"     python scripts/calibrate_motors.py --config {output_path}")
            else:
                print("\nTo save this config, run with --output option:")
                print(f"  python {sys.argv[0]} --port {args.port} --scan --output configs/robot/my_robot.yaml")

        return

    # Show help if no arguments
    parser.print_help()


if __name__ == "__main__":
    main()
