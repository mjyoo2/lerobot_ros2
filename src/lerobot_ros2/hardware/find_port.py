#!/usr/bin/env python3
"""
USB Serial Port Finder for Robot Hardware

This utility helps find USB serial ports for Dynamixel/Feetech motors and other devices.
Works on Linux, macOS, and Windows (including WSL).
"""

import sys
import glob
import platform
from typing import List, Optional
import serial
import serial.tools.list_ports


def get_available_ports() -> List[str]:
    """
    Get list of available serial ports.

    Returns:
        List of port names (e.g., ['/dev/ttyUSB0', 'COM3'])
    """
    ports = []

    system = platform.system()

    if system == "Linux":
        # Standard USB serial ports
        ports.extend(glob.glob('/dev/ttyUSB*'))
        ports.extend(glob.glob('/dev/ttyACM*'))
        # Devices by-id (more stable identifiers)
        ports.extend(glob.glob('/dev/serial/by-id/*'))
    elif system == "Darwin":  # macOS
        ports.extend(glob.glob('/dev/tty.usb*'))
        ports.extend(glob.glob('/dev/cu.usb*'))
    elif system == "Windows":
        # Windows COM ports
        for i in range(256):
            try:
                port = f'COM{i}'
                s = serial.Serial(port)
                s.close()
                ports.append(port)
            except (OSError, serial.SerialException):
                pass

    # Also use pyserial's built-in detector
    detected_ports = serial.tools.list_ports.comports()
    for port_info in detected_ports:
        if port_info.device not in ports:
            ports.append(port_info.device)

    return sorted(ports)


def get_port_info(port: str) -> dict:
    """
    Get detailed information about a serial port.

    Args:
        port: Port name (e.g., '/dev/ttyUSB0')

    Returns:
        Dictionary with port information
    """
    try:
        port_info = None
        for p in serial.tools.list_ports.comports():
            if p.device == port:
                port_info = p
                break

        if port_info:
            return {
                'device': port_info.device,
                'description': port_info.description,
                'hwid': port_info.hwid,
                'vid': port_info.vid,
                'pid': port_info.pid,
                'serial_number': port_info.serial_number,
                'manufacturer': port_info.manufacturer,
                'product': port_info.product,
            }
        else:
            return {
                'device': port,
                'description': 'Unknown',
                'hwid': 'Unknown'
            }
    except Exception as e:
        return {
            'device': port,
            'error': str(e)
        }


def test_port_connection(port: str, baudrate: int = 1000000, timeout: float = 0.5) -> bool:
    """
    Test if a port can be opened successfully.

    Args:
        port: Port name
        baudrate: Baud rate for testing (default: 1000000 for Dynamixel)
        timeout: Connection timeout in seconds

    Returns:
        True if port can be opened, False otherwise
    """
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        ser.close()
        return True
    except Exception:
        return False


def find_dynamixel_port(baudrate: int = 1000000) -> Optional[str]:
    """
    Attempt to find the Dynamixel motor port automatically.

    Args:
        baudrate: Baud rate to test (default: 1000000)

    Returns:
        Port name if found, None otherwise
    """
    ports = get_available_ports()

    for port in ports:
        if test_port_connection(port, baudrate):
            # Additional heuristics for Dynamixel devices
            info = get_port_info(port)
            description = (info.get('description') or '').lower()
            manufacturer = (info.get('manufacturer') or '').lower()

            # Common USB-to-serial chips used with Dynamixel
            if any(keyword in description or keyword in manufacturer for keyword in
                   ['ft232', 'ftdi', 'cp210', 'silicon labs', 'ch340', 'prolific']):
                return port

    # If no specific match, return first available port that opens
    for port in ports:
        if test_port_connection(port, baudrate):
            return port

    return None


def print_port_info_table(ports: List[str]):
    """Print a formatted table of port information."""
    if not ports:
        print("No serial ports found.")
        return

    print("\n" + "="*100)
    print(f"{'Port':<20} {'Description':<40} {'Manufacturer':<20} {'VID:PID':<15}")
    print("="*100)

    for port in ports:
        info = get_port_info(port)
        device = info.get('device', port)
        desc = info.get('description', 'Unknown') or 'Unknown'
        mfr = info.get('manufacturer', 'Unknown') or 'Unknown'
        vid = info.get('vid')
        pid = info.get('pid')

        if vid and pid:
            vid_pid = f"{vid:04X}:{pid:04X}"
        else:
            vid_pid = "N/A"

        # Truncate long descriptions
        if len(desc) > 38:
            desc = desc[:35] + "..."
        if len(mfr) > 18:
            mfr = mfr[:15] + "..."

        print(f"{device:<20} {desc:<40} {mfr:<20} {vid_pid:<15}")

    print("="*100 + "\n")


def main():
    """Main function for command-line usage."""
    print("🔍 Searching for USB serial ports...\n")

    ports = get_available_ports()

    if not ports:
        print("❌ No serial ports found.")
        print("\n💡 Troubleshooting tips:")
        print("  1. Check if your device is connected via USB")
        print("  2. On Linux, check permissions: sudo chmod 666 /dev/ttyUSB0")
        print("  3. On WSL, you may need to use usbipd to attach USB devices:")
        print("     - Install usbipd-win on Windows")
        print("     - Run: usbipd wsl list")
        print("     - Run: usbipd wsl attach --busid <BUSID>")
        print("  4. Install drivers if needed (FTDI, CH340, etc.)")
        sys.exit(1)

    print(f"✅ Found {len(ports)} serial port(s):")
    print_port_info_table(ports)

    # Try to find Dynamixel port
    print("🔌 Testing ports for Dynamixel compatibility (baudrate=1000000)...\n")

    dynamixel_port = find_dynamixel_port()

    if dynamixel_port:
        print(f"✅ Recommended port for Dynamixel: {dynamixel_port}")
        info = get_port_info(dynamixel_port)
        print(f"   Description: {info.get('description', 'Unknown')}")
        print(f"   Manufacturer: {info.get('manufacturer', 'Unknown')}")
    else:
        print("⚠️  Could not automatically detect Dynamixel port.")
        print("   Try each port manually or check your connections.")

    # Test each port
    print("\n📋 Port accessibility test:")
    for port in ports:
        accessible = test_port_connection(port)
        status = "✅ Accessible" if accessible else "❌ Not accessible"
        print(f"   {port}: {status}")

    print("\n💡 Usage in code:")
    print(f"   from lerobot_ros2.hardware.find_port import find_dynamixel_port")
    print(f"   port = find_dynamixel_port()")
    print(f"   if port:")
    print(f"       print(f'Using port: {{port}}')")


if __name__ == "__main__":
    main()
