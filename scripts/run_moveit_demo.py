#!/usr/bin/env python3
"""
MoveIt Demo Helper Script

This script:
1. Checks URDF generation (generates if missing)
2. Checks ROS2 Bridge execution
3. Executes MoveIt launch file

Usage:
    # Automatic configuration and execution
    python scripts/run_moveit_demo.py --config configs/robot/so100_config.yaml

    # Generate URDF only
    python scripts/run_moveit_demo.py --config configs/robot/so100_config.yaml --urdf-only

    # Help
    python scripts/run_moveit_demo.py --help
"""

import argparse
import subprocess
import sys
import time
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from lerobot_ros2.configs import load_config


def check_urdf_exists(urdf_path):
    """Check if URDF file exists"""
    return Path(urdf_path).exists()


def generate_urdf(config_path, urdf_path):
    """Generate URDF"""
    print("\n" + "="*80)
    print("📋 Generating URDF...")
    print("="*80)

    cmd = [
        sys.executable,
        "scripts/generate_urdf.py",
        "--config", config_path,
        "--output", urdf_path
    ]

    try:
        result = subprocess.run(cmd, check=True, capture_output=True, text=True)
        print(result.stdout)
        print(f"✅ URDF generation complete: {urdf_path}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ URDF generation failed:")
        print(e.stderr)
        return False


def check_ros2_bridge_running():
    """Check if ROS2 Bridge is running"""
    try:
        result = subprocess.run(
            ["ros2", "node", "list"],
            capture_output=True,
            text=True,
            timeout=5
        )
        return "robot_bridge" in result.stdout
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return False


def launch_moveit(config_path, urdf_path):
    """Launch MoveIt"""
    print("\n" + "="*80)
    print("🚀 Launching MoveIt...")
    print("="*80)
    print()
    print("When RViz opens:")
    print("  1. Check if 'Planning Group' is set to 'arm' in MotionPlanning panel")
    print("  2. Drag the Interactive Marker (orange sphere) with mouse")
    print("  3. Click 'Plan' button → check path")
    print("  4. Click 'Execute' button → robot moves!")
    print()
    print("💡 Tip: If Interactive Marker is not visible, check 'Query Start State'")
    print("="*80)
    print()

    cmd = [
        "ros2", "launch",
        "launch/moveit_demo.launch.py",
        f"robot_config:={config_path}",
        f"urdf:={urdf_path}"
    ]

    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"\n❌ MoveIt launch failed: {e}")
        return False
    except KeyboardInterrupt:
        print("\n\n🛑 MoveIt terminated")
        return True

    return True


def main():
    parser = argparse.ArgumentParser(
        description="MoveIt demo helper",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
  # Full configuration and execution
  python scripts/run_moveit_demo.py --config configs/robot/so100_config.yaml

  # Generate URDF only
  python scripts/run_moveit_demo.py --config configs/robot/so100_config.yaml --urdf-only

Prerequisites:
  1. Run ROS2 Bridge:
     python scripts/run_ros2_bridge.py --config configs/robot/so100_config.yaml

  2. Complete calibration (recommended):
     python scripts/calibrate_motors.py --config configs/robot/so100_config.yaml

  3. Check MoveIt installation:
     sudo apt install ros-humble-moveit

RViz Usage:
  - Drag Interactive Marker (orange sphere) to specify target position
  - Click 'Plan' button to plan path
  - Click 'Execute' button to execute
  - Click 'Plan & Execute' button to do both at once
        """
    )

    parser.add_argument(
        "--config",
        type=str,
        required=True,
        help="Robot config file path (e.g., configs/robot/so100_config.yaml)"
    )
    parser.add_argument(
        "--urdf",
        type=str,
        default="robot.urdf",
        help="URDF output path (Default: robot.urdf)"
    )
    parser.add_argument(
        "--urdf-only",
        action="store_true",
        help="Generate URDF only and exit"
    )
    parser.add_argument(
        "--skip-bridge-check",
        action="store_true",
        help="Skip ROS2 Bridge execution check"
    )

    args = parser.parse_args()

    # Check config file
    config_path = Path(args.config)
    if not config_path.exists():
        print(f"❌ Config file not found: {args.config}")
        return 1

    print("\n" + "="*80)
    print("🤖 MoveIt Demo Configuration")
    print("="*80)
    print(f"Config: {args.config}")
    print(f"URDF: {args.urdf}")
    print("="*80)
    print()

    # Load configuration
    try:
        robot_config = load_config(args.config)
        print(f"✅ Configuration loaded: {len(robot_config.get('motors', {}))} motors")
    except Exception as e:
        print(f"❌ Configuration load failed: {e}")
        return 1

    # Check/generate URDF
    if not check_urdf_exists(args.urdf):
        print(f"\n⚠️  URDF file not found: {args.urdf}")
        print("   Generating automatically...")
        if not generate_urdf(args.config, args.urdf):
            return 1
    else:
        print(f"\n✅ URDF file found: {args.urdf}")
        response = input("   Regenerate URDF? (y/N): ")
        if response.lower() == 'y':
            if not generate_urdf(args.config, args.urdf):
                return 1

    # Generate URDF only and exit
    if args.urdf_only:
        print("\n✅ URDF generation complete!")
        print(f"\nView URDF:")
        print(f"  ros2 launch urdf_tutorial display.launch.py model:={args.urdf}")
        return 0

    # Check ROS2 Bridge execution
    if not args.skip_bridge_check:
        print("\n" + "="*80)
        print("🔍 Checking ROS2 Bridge...")
        print("="*80)

        if check_ros2_bridge_running():
            print("✅ ROS2 Bridge is running")
        else:
            print("\n⚠️  ROS2 Bridge is not running!")
            print()
            print("Run it in another terminal first:")
            print(f"  python scripts/run_ros2_bridge.py --config {args.config}")
            print()
            response = input("Continue anyway? (y/N): ")
            if response.lower() != 'y':
                return 1

    # Launch MoveIt
    if not launch_moveit(args.config, args.urdf):
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
