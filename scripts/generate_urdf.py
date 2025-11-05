#!/usr/bin/env python3
"""
Automatic URDF Generation Script

Automatically generates URDF from robot config file.
If calibration data exists, uses actual joint limits.

Usage:
    python scripts/generate_urdf.py --config configs/robot/my_robot.yaml --output robot.urdf
"""

import argparse
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from lerobot_ros2.configs import load_config
from lerobot_ros2.hardware.calibration import CalibrationManager


URDF_TEMPLATE = """<?xml version="1.0"?>
<robot name="{robot_name}">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

{joints_and_links}

  <!-- End effector (gripper) -->
  <link name="end_effector">
    <visual>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

</robot>
"""

JOINT_TEMPLATE = """
  <!-- Joint {joint_num} {calib_info}-->
  <joint name="{joint_name}" type="revolute">
    <parent link="{parent_link}"/>
    <child link="{child_link}"/>
    <origin xyz="0 0 {height}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="{lower_limit}" upper="{upper_limit}" effort="10.0" velocity="2.0"/>
    <dynamics damping="0.1" friction="0.1"/>
  </joint>

  <!-- Link {joint_num} -->
  <link name="{child_link}">
    <visual>
      <geometry>
        <cylinder length="{length}" radius="0.02"/>
      </geometry>
      <origin xyz="0 0 {length_half}" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="{length}" radius="0.02"/>
      </geometry>
      <origin xyz="0 0 {length_half}" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <origin xyz="0 0 {length_half}" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>
"""


def position_to_radians(position, resolution=4096):
    """
    Convert Present_Position value to radians

    Args:
        position: Present_Position value (0~4095)
        resolution: Motor resolution (Default: 4096)

    Returns:
        Radians (0 ~ 2π)
    """
    import math
    return (position / resolution) * 2 * math.pi


def calculate_joint_limits(calibration, resolution=4096):
    """
    Calculate joint limits from calibration data (radians)

    Configures neutral position (center) as 0 radians

    Args:
        calibration: MotorCalibration object
        resolution: Motor resolution (Default: 4096)

    Returns:
        (lower_limit, upper_limit) in radians
    """
    import math

    # Configure center as 0
    center = (calibration.range_min + calibration.range_max) / 2

    # Calculate limits based on neutral position
    lower_limit = (calibration.range_min - center) * (2 * math.pi / resolution)
    upper_limit = (calibration.range_max - center) * (2 * math.pi / resolution)

    return lower_limit, upper_limit


def generate_urdf(robot_config, output_path):
    """Generate URDF"""

    robot_name = robot_config.get("robot_type", "robot")
    motors = robot_config.get("motors", {})
    num_motors = len(motors)

    print(f"🤖 Generating URDF for {num_motors} joints...")

    # Try loading calibration
    calibrations = {}
    calibration_dir = Path(robot_config.get("calibration_dir", "./calibration"))
    if calibration_dir.exists():
        try:
            manager = CalibrationManager(calibration_dir)
            calibrations = manager.load()
            print(f"✓ Calibration loaded: {len(calibrations)} motors")
        except Exception as e:
            print(f"⚠️  Calibration load failed: {e}")
            print("   Using default joint limits (-1.75 ~ 1.75 radians)")
    else:
        print(f"⚠️  No calibration directory: {calibration_dir}")
        print("   Using default joint limits (-1.75 ~ 1.75 radians)")

    # Generate joints and links
    joints_and_links = ""
    link_length = 0.1  # Each link length (m)

    for i, (motor_name, motor_info) in enumerate(motors.items(), start=1):
        parent_link = "base_link" if i == 1 else f"link_{i-1}"
        child_link = f"link_{i}"
        joint_name = f"joint_{i}"
        motor_id = motor_info["id"]

        # Calculate joint limits
        if motor_name in calibrations:
            calib = calibrations[motor_name]
            lower_limit, upper_limit = calculate_joint_limits(calib)
            calib_info = f"(calibrated: {calib.range_min}~{calib.range_max} → {lower_limit:.3f}~{upper_limit:.3f} rad) "
        else:
            # Default: ±100 degrees (±1.75 radians)
            lower_limit = -1.75
            upper_limit = 1.75
            calib_info = "(default limits) "

        # Last joint connects to end_effector
        if i == num_motors:
            # Gripper joint (usually last joint)
            joints_and_links += JOINT_TEMPLATE.format(
                joint_num=i,
                calib_info=calib_info,
                joint_name=joint_name,
                parent_link=parent_link,
                child_link=child_link,
                height=link_length,
                length=link_length * 0.5,
                length_half=link_length * 0.25,
                lower_limit=f"{lower_limit:.6f}",
                upper_limit=f"{upper_limit:.6f}",
            )

            # End effector connection
            joints_and_links += f"""
  <!-- Gripper joint -->
  <joint name="gripper_joint" type="fixed">
    <parent link="{child_link}"/>
    <child link="end_effector"/>
    <origin xyz="0 0 {link_length * 0.5}" rpy="0 0 0"/>
  </joint>
"""
        else:
            joints_and_links += JOINT_TEMPLATE.format(
                joint_num=i,
                calib_info=calib_info,
                joint_name=joint_name,
                parent_link=parent_link,
                child_link=child_link,
                height=link_length,
                length=link_length,
                length_half=link_length / 2,
                lower_limit=f"{lower_limit:.6f}",
                upper_limit=f"{upper_limit:.6f}",
            )

    # Generate URDF
    urdf_content = URDF_TEMPLATE.format(
        robot_name=robot_name,
        joints_and_links=joints_and_links,
    )

    # Save file
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(urdf_content)

    print(f"✅ URDF saved: {output_path}")
    print()
    print("URDF Contents:")
    print(f"  - Robot name: {robot_name}")
    print(f"  - Joints: {num_motors}")
    print(f"  - Links: {num_motors + 1} (base + {num_motors} links)")
    print(f"  - End effector: end_effector")
    print()
    print("Joint names:")
    for i in range(1, num_motors + 1):
        print(f"  - joint_{i}")
    print()


def main():
    parser = argparse.ArgumentParser(description="Automatic URDF generation")
    parser.add_argument(
        "--config",
        type=str,
        required=True,
        help="Robot config file",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="robot.urdf",
        help="Output URDF file (Default: robot.urdf)",
    )

    args = parser.parse_args()

    # Load configuration
    print(f"📋 Loading config: {args.config}")
    robot_config = load_config(args.config)

    # Generate URDF
    generate_urdf(robot_config, args.output)

    print("💡 Next steps:")
    print(f"  1. Review the URDF: cat {args.output}")
    print(f"  2. Visualize: ros2 launch urdf_tutorial display.launch.py model:={args.output}")
    print(f"  3. Use with MoveIt (coming soon)")


if __name__ == "__main__":
    main()
