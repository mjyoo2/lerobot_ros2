# SO-101 Complete Setup Guide

Complete guide for setting up the SO-101 robotic arm with LeRobot-ROS2, from URDF download to MoveIt integration.

---

## Table of Contents

1. [Overview](#overview)
2. [Prerequisites](#prerequisites)
3. [Quick Start (5 Minutes)](#quick-start-5-minutes)
4. [Detailed Setup](#detailed-setup)
5. [URDF Setup](#urdf-setup)
6. [MoveIt Integration](#moveit-integration)
7. [Troubleshooting](#troubleshooting)

---

## Overview

The SO-101 is a 6-DOF robotic arm compatible with LeRobot-ROS2. This guide covers:

- **Hardware setup**: Motor discovery, calibration, safety configuration
- **URDF configuration**: Downloading and using the official SO-101 URDF
- **ROS2 integration**: Publishing joint states, receiving commands
- **MoveIt integration**: Motion planning with collision avoidance

**System Requirements:**
- Ubuntu 22.04 or later
- ROS2 Humble/Jazzy
- Python 3.10+
- Feetech STS3215 motors (or compatible Dynamixel)

---

## Prerequisites

### Install Dependencies

```bash
# Install Python package
pip install -e .
pip install pygame  # For GUI control

# Install ROS2 (if not already installed)
# For Ubuntu 22.04 - ROS2 Humble:
sudo apt install ros-humble-desktop

# For Ubuntu 24.04 - ROS2 Jazzy:
sudo apt install ros-jazzy-desktop

# Install MoveIt (for advanced motion planning)
sudo apt install ros-${ROS_DISTRO}-moveit
```

### USB Device Access

```bash
# Give permission to access serial ports
sudo usermod -aG dialout $USER
# Log out and log back in for this to take effect

# Or temporarily for current session:
sudo chmod 666 /dev/ttyUSB0  # Replace with your port
```

---

## Quick Start (5 Minutes)

Get your SO-101 running in 5 simple steps:

```bash
# 1. Find USB port and scan motors
python scripts/register_motors.py --port /dev/ttyUSB0 --scan --output configs/robot/so101_scanned.yaml

# 2. Calibrate motors (hands-on, follow prompts)
python scripts/calibrate_motors.py --config configs/robot/so101_scanned.yaml

# 3. Synchronize Goal Position (CRITICAL - prevents sudden movements!)
python scripts/fix_goal_position.py --config configs/robot/so101_scanned.yaml

# 4. Set speed limit for safety (recommended for first use)
python scripts/set_motor_speed_limit.py --config configs/robot/so101_scanned.yaml --speed 200

# 5A. Control with GUI
python scripts/control_motors_gui.py --config configs/robot/so101_scanned.yaml

# OR 5B. Control with ROS2
python scripts/run_ros2_bridge.py --config configs/robot/so101_scanned.yaml
```

**Once running (Option 5B - ROS2):**

```bash
# In another terminal, move to home position
ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

---

## Detailed Setup

### Step 1: Motor Discovery and Configuration

#### Find Available USB Ports

```bash
python scripts/register_motors.py --find-port
```

**Output:**
```
Available ports:
  ✓ /dev/ttyUSB0 - USB Serial
  ✓ /dev/ttyACM0 - USB CDC
```

#### Scan and Register Motors

```bash
python scripts/register_motors.py \
    --port /dev/ttyUSB0 \
    --scan \
    --output configs/robot/so101_scanned.yaml
```

**What this does:**
- Scans for all connected motors
- Reads motor IDs and current positions
- Generates a config file with motor mapping

**Generated file:** `configs/robot/so101_scanned.yaml`

```yaml
robot_type: so101
port: /dev/ttyUSB0
baudrate: 1000000
calibration_dir: calibration/so101

motors:
  motor_1:
    id: 1
    joint_name: shoulder_pan
  motor_2:
    id: 2
    joint_name: shoulder_lift
  motor_3:
    id: 3
    joint_name: elbow_flex
  motor_4:
    id: 4
    joint_name: wrist_flex
  motor_5:
    id: 5
    joint_name: wrist_roll
  motor_6:
    id: 6
    joint_name: gripper

joint_names:
  - shoulder_pan
  - shoulder_lift
  - elbow_flex
  - wrist_flex
  - wrist_roll
  - gripper
```

---

### Step 2: Motor Calibration

Calibration sets the neutral position and working range for each motor.

```bash
python scripts/calibrate_motors.py --config configs/robot/so101_scanned.yaml
```

#### Calibration Process:

**Phase 1: Set Neutral Position**
1. Motors are **torque-disabled** (you can move them by hand)
2. Move the robot to a natural **middle pose**
3. Press **Enter** to record neutral position

**Phase 2: Record Range**
1. **Slowly** move each joint through its **full range of motion**
2. Move to minimum position, then to maximum position
3. Press **Enter** when done

**Phase 3: Save**
- Calibration data is saved to motor firmware (Homing Offset)
- Backup saved to `calibration/so101/calibration.json`

**Generated file:** `calibration/so101/calibration.json`

```json
{
  "1": {
    "homing_offset": -1224,
    "range_min": 933,
    "range_max": 2981,
    "drive_mode": 0
  },
  "2": { ... },
  ...
}
```

#### Tips for Good Calibration:

- **Neutral position**: A pose where the robot can move equally in all directions
- **Range recording**: Move slowly and smoothly
- **Physical limits**: Don't force beyond mechanical limits
- **Even coverage**: Move all joints through their full range

---

### Step 3: Goal Position Synchronization (CRITICAL!)

After calibration, you **MUST** run this script to prevent sudden movements when torque is enabled.

```bash
python scripts/fix_goal_position.py --config configs/robot/so101_scanned.yaml
```

**What this does:**
- Reads current motor positions
- Sets `Goal_Position` = `Present_Position` for all motors
- Ensures smooth torque activation

**⚠️ If you skip this step:**
- Motors will jump to previous `Goal_Position` when torque is enabled
- Can cause damage or injury!

---

### Step 4: Speed Limit Configuration (Recommended)

Set maximum speed for safe operation:

```bash
# Slow (safe for testing)
python scripts/set_motor_speed_limit.py --config configs/robot/so101_scanned.yaml --speed 200

# Medium (normal use)
python scripts/set_motor_speed_limit.py --config configs/robot/so101_scanned.yaml --speed 500

# Fast
python scripts/set_motor_speed_limit.py --config configs/robot/so101_scanned.yaml --speed 800
```

**Speed values:**
- `100-200`: Very slow (safe, recommended for first use)
- `300-500`: Medium (normal operation)
- `600-1000`: Fast

---

### Step 5: Motor Control

#### Option A: GUI Control

```bash
python scripts/control_motors_gui.py --config configs/robot/so101_scanned.yaml
```

**Usage:**
1. Click **Connect** → Connect to motors
2. Click **Torque** → Enable motor torque
3. Drag **sliders** → Control each motor
   - Yellow line: Current position
   - Green circle: Target position
4. Click **Center** → Move all motors to neutral (0)
5. Click **Quit** → Exit

#### Option B: ROS2 Control

```bash
python scripts/run_ros2_bridge.py --config configs/robot/so101_scanned.yaml
```

**In another terminal:**

```bash
# Monitor current position
ros2 topic echo /joint_states

# Move to neutral position
ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

# Move specific joints (shoulder_pan=50, elbow_flex=-30)
ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray "data: [50.0, 0.0, -30.0, 0.0, 0.0, 0.0]"
```

**Coordinate system:**
- Range: `-100` to `+100` (normalized)
- `0`: Calibrated neutral position
- `±100`: Calibrated min/max limits

---

## URDF Setup

### Download SO-101 Official URDF

The SO-101 has an official URDF with accurate kinematics and meshes.

```bash
python scripts/download_so_arm_urdf.py --model so101 --output urdf/
```

**Downloaded file:** `urdf/so101_new-calib.urdf` (16KB) + meshes in `urdf/assets/` (16MB)

**Joint names in URDF:**
1. `shoulder_pan` (Motor ID 1)
2. `shoulder_lift` (Motor ID 2)
3. `elbow_flex` (Motor ID 3)
4. `wrist_flex` (Motor ID 4)
5. `wrist_roll` (Motor ID 5)
6. `gripper` (Motor ID 6)

### Verify URDF in RViz

```bash
ros2 launch urdf_tutorial display.launch.py model:=urdf/so101_new-calib.urdf
```

**You should see:**
- 3D robot model in RViz
- Joint sliders in GUI
- TF tree visualizing coordinate frames

---

### Joint Mapping

The ROS2 bridge automatically maps motor IDs to URDF joint names using the config file:

| Motor ID | Config Name | URDF Joint Name | Description |
|----------|-------------|-----------------|-------------|
| 1 | motor_1 | shoulder_pan | Base rotation |
| 2 | motor_2 | shoulder_lift | Shoulder pitch |
| 3 | motor_3 | elbow_flex | Elbow pitch |
| 4 | motor_4 | wrist_flex | Wrist pitch |
| 5 | motor_5 | wrist_roll | Wrist roll |
| 6 | motor_6 | gripper | Gripper |

---

## MoveIt Integration

Use MoveIt for advanced motion planning with obstacle avoidance and Cartesian control.

### Prerequisites

```bash
# Install MoveIt
sudo apt install ros-${ROS_DISTRO}-moveit

# Verify installation
ros2 pkg list | grep moveit
```

### Setup and Launch

#### Method 1: Integrated Launch (Recommended)

```bash
#  Terminal 1: ROS2 Bridge
python scripts/run_ros2_bridge.py --config configs/robot/so101_scanned.yaml

# Terminal 2: MoveIt Demo (auto-generates URDF and config)
python scripts/run_moveit_demo.py --config configs/robot/so101_scanned.yaml
```

#### Method 2: Use Official URDF

```bash
# Terminal 1: ROS2 Bridge
python scripts/run_ros2_bridge.py --config configs/robot/so101_scanned.yaml

# Terminal 2: MoveIt with SO-101 URDF
./start_moveit.sh
```

**Contents of `start_moveit.sh`:**
```bash
#!/bin/bash
ros2 launch launch/moveit_demo.launch.py \
    robot_config:=configs/robot/so101_scanned.yaml \
    urdf:=urdf/so101_new-calib.urdf \
    use_real_robot:=true
```

### Using MoveIt in RViz

Once RViz opens:

**1. Initial Setup:**
- MotionPlanning panel should be visible
- Planning Group: Select **"arm"**
- Planning Library: **OMPL** (default)

**2. Set Goal:**
- Drag the **Interactive Marker** (orange sphere at end-effector)
- Move it to desired position and orientation

**3. Plan Motion:**
- Click **"Plan"** button
- Green trajectory shows planned path
- Check for collisions

**4. Execute:**
- Click **"Execute"** → Robot moves!
- Or use **"Plan & Execute"** for one-click operation

### MoveIt Features

- **Collision Detection**: Automatic self-collision avoidance
- **Path Planning**: OMPL algorithms (RRT, RRTConnect, etc.)
- **Cartesian Paths**: Move end-effector in straight lines
- **Joint-Space Planning**: Direct joint angle targets

---

## Troubleshooting

### Diagnostic Tool

First, run the diagnostic script to identify issues:

```bash
python scripts/diagnose_motor_calibration.py --config configs/robot/so101_scanned.yaml
```

**This checks:**
- ✅ Homing Offset properly set
- ✅ Goal Position = Present Position
- ✅ Position Limits correct
- ⚠️ Risk of sudden movement on torque enable

---

### Common Issues

#### 1. Motors Jump When Torque is Enabled

**Cause:** Goal Position ≠ Present Position

**Fix:**
```bash
python scripts/fix_goal_position.py --config configs/robot/so101_scanned.yaml
```

---

#### 2. Cannot Find Motors

**Symptoms:**
```
❌ Hardware connection failed
Error: No motors found on port /dev/ttyUSB0
```

**Fix:**

```bash
# Check available ports
ls /dev/tty*

# Find correct port
python scripts/register_motors.py --find-port

# Check USB permissions
sudo chmod 666 /dev/ttyUSB0

# For WSL users - attach USB device (in Windows PowerShell as Admin):
usbipd wsl list
usbipd wsl attach --busid <BUSID>
```

---

#### 3. Motors Moving Too Fast

**Fix:**
```bash
# Set lower speed limit (100-200 recommended)
python scripts/set_motor_speed_limit.py --config configs/robot/so101_scanned.yaml --speed 150
```

---

#### 4. URDF Visualization Mismatch in RViz

**Symptoms:** Robot position in RViz doesn't match physical robot (joints 5-6 appear rotated ~90 degrees)

**Cause:** URDF origin offset in wrist_roll joint definition

**Note:** This is a **cosmetic issue only**. The robot functions correctly, but the visual representation in RViz may look rotated. Physical robot movement is accurate.

**Workaround:** Accept the visual mismatch, or manually edit the URDF (not recommended if auto-generated from OnShape).

---

#### 5. Re-Calibration Procedure

If you need to re-calibrate:

```bash
# 1. Calibrate
python scripts/calibrate_motors.py --config configs/robot/so101_scanned.yaml

# 2. Sync Goal Position (REQUIRED after calibration!)
python scripts/fix_goal_position.py --config configs/robot/so101_scanned.yaml

# 3. Set speed limit (optional)
python scripts/set_motor_speed_limit.py --config configs/robot/so101_scanned.yaml --speed 200
```

---

#### 6. MoveIt Planning Fails

**Possible causes:**

1. **Joint limits too restrictive**
   - Edit `config/moveit/joint_limits.yaml`
   - Increase `max_velocity` or `max_acceleration`

2. **Goal is unreachable**
   - Check if target is within workspace
   - Try different approach (rotate end-effector)

3. **Self-collision detected**
   - SRDF has collision pairs disabled for adjacent links
   - Check `config/moveit/*.srdf`

4. **Planning timeout**
   - Increase timeout in MoveIt config:
     ```yaml
     planning_time: 10.0  # seconds
     ```

---

## File Structure

After setup, your project should look like:

```
LeRobot_ros2/
├── configs/robot/
│   └── so101_scanned.yaml          # Motor configuration
│
├── calibration/so101/
│   └── calibration.json            # Calibration data
│
├── urdf/
│   ├── so101_new-calib.urdf        # Official SO-101 URDF
│   └── assets/                     # 3D meshes (16MB)
│
├── scripts/
│   ├── register_motors.py          # Motor discovery
│   ├── calibrate_motors.py         # Calibration
│   ├── fix_goal_position.py        # Goal Position sync
│   ├── set_motor_speed_limit.py    # Speed configuration
│   ├── control_motors_gui.py       # GUI control
│   ├── run_ros2_bridge.py          # ROS2 bridge
│   ├── run_moveit_demo.py          # MoveIt integration
│   └── diagnose_motor_calibration.py  # Diagnostics
│
├── launch/
│   └── moveit_demo.launch.py       # MoveIt launch file
│
└── config/moveit/                   # Auto-generated MoveIt config
    ├── *.srdf
    ├── joint_limits.yaml
    └── ros2_controllers.yaml
```

---

## Next Steps

Once your SO-101 is set up:

1. **Basic Control**: [Hardware Setup Guide](HARDWARE_SETUP.md)
2. **Advanced Planning**: [MoveIt Integration Guide](MOVEIT_INTEGRATION.md)
3. **Script Reference**: [Scripts Reference](SCRIPTS_REFERENCE.md)

---

## Summary

### Essential Commands

```bash
# Hardware Setup (run once)
python scripts/register_motors.py --port /dev/ttyUSB0 --scan --output configs/robot/so101_scanned.yaml
python scripts/calibrate_motors.py --config configs/robot/so101_scanned.yaml

# After Every Calibration (CRITICAL!)
python scripts/fix_goal_position.py --config configs/robot/so101_scanned.yaml

# Daily Use
python scripts/run_ros2_bridge.py --config configs/robot/so101_scanned.yaml
# OR
python scripts/control_motors_gui.py --config configs/robot/so101_scanned.yaml

# MoveIt (Advanced)
./start_moveit.sh
```

### Safety Checklist

- ✅ Run `fix_goal_position.py` after every calibration
- ✅ Set speed limit for first use (`--speed 200`)
- ✅ Always have hand on robot during initial tests
- ✅ Use `diagnose_motor_calibration.py` if anything seems wrong
- ✅ Keep emergency stop accessible

---

**Your SO-101 is ready to use! 🎉**
