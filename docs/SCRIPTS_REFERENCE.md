# Scripts Reference

Complete reference for all LeRobot-ROS2 scripts with usage examples, options, and workflows.

---

## Table of Contents

1. [Hardware Setup Scripts](#hardware-setup-scripts)
2. [Control Scripts](#control-scripts)
3. [MoveIt Scripts](#moveit-scripts)
4. [URDF Scripts](#urdf-scripts)
5. [Utility Scripts](#utility-scripts)
6. [Workflows](#workflows)

---

## Hardware Setup Scripts

### register_motors.py

**Purpose:** Find USB ports and scan for connected motors.

**Usage:**
```bash
python scripts/register_motors.py [OPTIONS]
```

**Options:**
- `--find-port`: Auto-detect available USB ports
- `--port PORT`: Specify USB port (e.g., `/dev/ttyUSB0`)
- `--scan`: Scan for connected motors on the port
- `--output FILE`: Save configuration to YAML file
- `--baudrate RATE`: Set baudrate (default: 1000000)

**Examples:**

```bash
# Find available ports
python scripts/register_motors.py --find-port

# Scan motors and save config
python scripts/register_motors.py \
    --port /dev/ttyUSB0 \
    --scan \
    --output configs/robot/my_robot.yaml

# Scan with custom baudrate
python scripts/register_motors.py \
    --port /dev/ttyUSB0 \
    --scan \
    --baudrate 57600 \
    --output configs/robot/custom.yaml
```

**Output:**
```
🔍 Scanning port /dev/ttyUSB0...
✓ Found motor ID: 1 (Model: STS3215)
✓ Found motor ID: 2 (Model: STS3215)
✓ Found motor ID: 3 (Model: STS3215)
✓ Found motor ID: 4 (Model: STS3215)
✓ Found motor ID: 5 (Model: STS3215)
✓ Found motor ID: 6 (Model: STS3215)

✅ Configuration saved to: configs/robot/my_robot.yaml
```

---

### calibrate_motors.py

**Purpose:** Calibrate motor neutral positions and working ranges.

**Usage:**
```bash
python scripts/calibrate_motors.py --config CONFIG [OPTIONS]
```

**Options:**
- `--config FILE`: Robot configuration file (required)
- `--group GROUP`: Calibrate specific motor group (if defined in config)

**Examples:**

```bash
# Calibrate all motors
python scripts/calibrate_motors.py --config configs/robot/my_robot.yaml

# Calibrate specific group
python scripts/calibrate_motors.py \
    --config configs/robot/my_robot.yaml \
    --group arm
```

**Process:**
1. **Neutral Position**: Motors disabled, position robot manually, press Enter
2. **Range Recording**: Move joints slowly through full range, press Enter
3. **Save**: Data written to motor firmware and JSON file

**Output:**
```
🎯 Calibrating motors from: configs/robot/my_robot.yaml
🔧 Calibrating 6 motors...

=== Phase 1: Set Neutral Position ===
⚙️  Torque disabled - you can move motors by hand
📍 Position robot in neutral pose, then press Enter...

=== Phase 2: Record Range ===
🔄 Slowly move each joint through full range...
Recording... (Ctrl+C when done)

✅ Calibration complete!
📁 Saved to: calibration/my_robot/calibration.json
```

---

### fix_goal_position.py

**Purpose:** Synchronize Goal Position with Present Position to prevent sudden movements.

**Usage:**
```bash
python scripts/fix_goal_position.py --config CONFIG
```

**Options:**
- `--config FILE`: Robot configuration file (required)

**Example:**

```bash
python scripts/fix_goal_position.py --config configs/robot/my_robot.yaml
```

**Output:**
```
🔧 Synchronizing Goal Position...

Motor 1: Goal=2048 → Present=2050 ✓
Motor 2: Goal=2048 → Present=2051 ✓
Motor 3: Goal=2048 → Present=2049 ✓
Motor 4: Goal=2048 → Present=2052 ✓
Motor 5: Goal=2048 → Present=2047 ✓
Motor 6: Goal=2048 → Present=2050 ✓

✅ All Goal Positions synchronized!
```

**When to use:**
- **After every calibration** (CRITICAL!)
- When motors jump unexpectedly on torque enable
- After motor firmware updates

---

### set_motor_speed_limit.py

**Purpose:** Configure maximum motor velocity for safety.

**Usage:**
```bash
python scripts/set_motor_speed_limit.py --config CONFIG --speed SPEED
```

**Options:**
- `--config FILE`: Robot configuration file (required)
- `--speed VALUE`: Speed limit (100-1000, default: 300)

**Examples:**

```bash
# Very slow (safe for testing)
python scripts/set_motor_speed_limit.py \
    --config configs/robot/my_robot.yaml \
    --speed 150

# Medium speed (normal use)
python scripts/set_motor_speed_limit.py \
    --config configs/robot/my_robot.yaml \
    --speed 500

# Fast
python scripts/set_motor_speed_limit.py \
    --config configs/robot/my_robot.yaml \
    --speed 800
```

**Speed Guidelines:**
- `100-200`: Very slow, safe for initial testing
- `300-500`: Medium, normal operation
- `600-1000`: Fast, use with caution

---

## Control Scripts

### control_motors_gui.py

**Purpose:** GUI-based motor control with real-time position sliders.

**Usage:**
```bash
python scripts/control_motors_gui.py --config CONFIG [OPTIONS]
```

**Options:**
- `--config FILE`: Robot configuration file (required)
- `--rate HZ`: Update rate in Hz (default: 30)

**Example:**

```bash
python scripts/control_motors_gui.py --config configs/robot/my_robot.yaml
```

**GUI Controls:**
- **Connect**: Connect to motors
- **Torque**: Enable/disable motor torque
- **Sliders**: Control individual motors
  - Yellow line: Current position
  - Green circle: Target position
- **Center**: Move all motors to neutral (0)
- **Quit**: Disconnect and exit

---

### run_ros2_bridge.py

**Purpose:** ROS2 bridge for publishing joint states and receiving commands.

**Usage:**
```bash
python scripts/run_ros2_bridge.py --config CONFIG [OPTIONS]
```

**Options:**
- `--config FILE`: Robot configuration file (required)
- `--rate HZ`: Publishing rate in Hz (default: 30.0)

**Examples:**

```bash
# Standard operation
python scripts/run_ros2_bridge.py --config configs/robot/my_robot.yaml

# High frequency
python scripts/run_ros2_bridge.py \
    --config configs/robot/my_robot.yaml \
    --rate 50
```

**ROS2 Topics:**

**Published:**
- `/joint_states` (sensor_msgs/JointState) - Current motor positions in radians
- `/camera/{name}/image_raw` (sensor_msgs/Image) - Camera images (if configured)

**Subscribed:**
- `/joint_commands` (std_msgs/Float64MultiArray) - Target positions (normalized or radians)
- `/joint_trajectory_controller/joint_trajectory` (trajectory_msgs/JointTrajectory) - MoveIt trajectories

**Control Examples:**

```bash
# Move to neutral
ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray \
    "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

# Move specific joints
ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray \
    "data: [50.0, -30.0, 20.0, 0.0, 0.0, 0.0]"

# Monitor current position
ros2 topic echo /joint_states
```

---

### diagnose_motor_calibration.py

**Purpose:** Diagnose calibration issues and detect potential problems.

**Usage:**
```bash
python scripts/diagnose_motor_calibration.py --config CONFIG
```

**Options:**
- `--config FILE`: Robot configuration file (required)

**Example:**

```bash
python scripts/diagnose_motor_calibration.py --config configs/robot/my_robot.yaml
```

**Output:**
```
🔍 Diagnostic Report
================================================================================

Motor 1 (shoulder_pan):
  ✅ Homing Offset: -1224 (set)
  ✅ Goal Position: 2050 = Present Position: 2050
  ✅ Position Limits: 933 - 2981
  ✅ Status: OK

Motor 2 (shoulder_lift):
  ✅ Homing Offset: -987 (set)
  ⚠️  Goal Position: 2100 ≠ Present Position: 2050 (Δ=50)
      → Run: python scripts/fix_goal_position.py --config ...
  ✅ Position Limits: 800 - 3100
  ⚠️  Status: NEEDS SYNC

...

================================================================================
Summary:
  ✅ 5 motors OK
  ⚠️  1 motor needs Goal Position sync
================================================================================
```

---

## MoveIt Scripts

### run_moveit_demo.py

**Purpose:** Integrated MoveIt demo launcher with automatic URDF generation.

**Usage:**
```bash
python scripts/run_moveit_demo.py --config CONFIG [OPTIONS]
```

**Options:**
- `--config FILE`: Robot configuration file (required)
- `--urdf PATH`: URDF file path (default: `robot.urdf`)
- `--urdf-only`: Generate URDF and exit
- `--skip-bridge-check`: Don't check if ROS2 bridge is running

**Examples:**

```bash
# Full MoveIt demo
python scripts/run_moveit_demo.py --config configs/robot/my_robot.yaml

# Generate URDF only
python scripts/run_moveit_demo.py \
    --config configs/robot/my_robot.yaml \
    --urdf-only

# Use custom URDF
python scripts/run_moveit_demo.py \
    --config configs/robot/my_robot.yaml \
    --urdf custom_robot.urdf
```

**What it does:**
1. Checks if URDF exists, generates if missing
2. Verifies ROS2 bridge is running
3. Launches MoveIt + RViz

---

### execute_moveit_trajectory.py

**Purpose:** Action server for executing MoveIt trajectories on real hardware.

**Usage:**
```bash
python scripts/execute_moveit_trajectory.py
```

**No options** - runs as ROS2 action server.

**Action Interface:**
- **Name**: `/robot_controller/follow_joint_trajectory`
- **Type**: `control_msgs/action/FollowJointTrajectory`

**Use with MoveIt:**
1. Run this script in background
2. Launch MoveIt
3. Use "Plan & Execute" button in RViz

---

### setup_moveit.py

**Purpose:** Generate MoveIt configuration files from robot config.

**Usage:**
```bash
python scripts/setup_moveit.py --config CONFIG [OPTIONS]
```

**Options:**
- `--config FILE`: Robot configuration file (required)
- `--output-dir DIR`: Output directory (default: `moveit_config`)

**Example:**

```bash
python scripts/setup_moveit.py \
    --config configs/robot/my_robot.yaml \
    --output-dir config/moveit
```

**Generated Files:**
- `ros2_controllers.yaml`: ROS2 control configuration
- `{robot}.srdf`: Semantic Robot Description
- `joint_limits.yaml`: Joint velocity/acceleration limits
- `README.txt`: Next steps instructions

---

## URDF Scripts

### generate_urdf.py

**Purpose:** Generate URDF from robot configuration.

**Usage:**
```bash
python scripts/generate_urdf.py --config CONFIG [OPTIONS]
```

**Options:**
- `--config FILE`: Robot configuration file (required)
- `--output FILE`: Output URDF file path (default: `robot.urdf`)

**Example:**

```bash
python scripts/generate_urdf.py \
    --config configs/robot/my_robot.yaml \
    --output urdf/my_robot.urdf
```

---

### download_so_arm_urdf.py

**Purpose:** Download official SO-ARM series URDFs (SO-100, SO-101).

**Usage:**
```bash
python scripts/download_so_arm_urdf.py --model MODEL [OPTIONS]
```

**Options:**
- `--model NAME`: Robot model (so100, so101)
- `--output DIR`: Output directory (default: `urdf/`)

**Examples:**

```bash
# Download SO-101 URDF
python scripts/download_so_arm_urdf.py --model so101 --output urdf/

# Download SO-100 URDF
python scripts/download_so_arm_urdf.py --model so100 --output urdf/
```

**Downloads:**
- URDF file: `urdf/so101_new-calib.urdf`
- 3D meshes: `urdf/assets/*.stl`

---

## Utility Scripts

### find_motors.py

**Purpose:** Quickly scan for motors without saving config.

**Usage:**
```bash
python scripts/find_motors.py [OPTIONS]
```

**Options:**
- `--port PORT`: USB port to scan
- `--baudrate RATE`: Baudrate (default: 1000000)

**Example:**

```bash
python scripts/find_motors.py --port /dev/ttyUSB0
```

---

## Workflows

### Complete Hardware Setup

```bash
# 1. Find port and register motors
python scripts/register_motors.py \
    --port /dev/ttyUSB0 \
    --scan \
    --output configs/robot/my_robot.yaml

# 2. Calibrate
python scripts/calibrate_motors.py --config configs/robot/my_robot.yaml

# 3. Sync Goal Position (CRITICAL!)
python scripts/fix_goal_position.py --config configs/robot/my_robot.yaml

# 4. Set safe speed
python scripts/set_motor_speed_limit.py \
    --config configs/robot/my_robot.yaml \
    --speed 200

# 5. Test with GUI
python scripts/control_motors_gui.py --config configs/robot/my_robot.yaml
```

---

### ROS2 Control Workflow

```bash
# Terminal 1: Run bridge
python scripts/run_ros2_bridge.py --config configs/robot/my_robot.yaml

# Terminal 2: Monitor state
ros2 topic echo /joint_states

# Terminal 3: Send commands
ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray \
    "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

---

### MoveIt Workflow

```bash
# Terminal 1: ROS2 bridge
python scripts/run_ros2_bridge.py --config configs/robot/my_robot.yaml

# Terminal 2: Trajectory executor (optional, for Plan & Execute)
python scripts/execute_moveit_trajectory.py

# Terminal 3: MoveIt demo
python scripts/run_moveit_demo.py --config configs/robot/my_robot.yaml
```

---

### Re-Calibration Workflow

```bash
# 1. Re-calibrate
python scripts/calibrate_motors.py --config configs/robot/my_robot.yaml

# 2. MUST sync Goal Position after calibration!
python scripts/fix_goal_position.py --config configs/robot/my_robot.yaml

# 3. Optional: Reset speed limit
python scripts/set_motor_speed_limit.py \
    --config configs/robot/my_robot.yaml \
    --speed 200

# 4. Test
python scripts/control_motors_gui.py --config configs/robot/my_robot.yaml
```

---

### Troubleshooting Workflow

```bash
# 1. Run diagnostic
python scripts/diagnose_motor_calibration.py --config configs/robot/my_robot.yaml

# 2. If Goal Position mismatch detected:
python scripts/fix_goal_position.py --config configs/robot/my_robot.yaml

# 3. If calibration looks wrong:
python scripts/calibrate_motors.py --config configs/robot/my_robot.yaml
python scripts/fix_goal_position.py --config configs/robot/my_robot.yaml

# 4. Test again
python scripts/control_motors_gui.py --config configs/robot/my_robot.yaml
```

---

## Script Dependencies

```
register_motors.py
  ↓
calibrate_motors.py
  ↓
fix_goal_position.py (REQUIRED after calibration!)
  ↓
set_motor_speed_limit.py (optional)
  ↓
control_motors_gui.py OR run_ros2_bridge.py
```

---

## Common Options Summary

| Script | --config | --port | --speed | --output | --urdf |
|--------|----------|--------|---------|----------|--------|
| register_motors.py | - | ✓ | - | ✓ | - |
| calibrate_motors.py | ✓ | - | - | - | - |
| fix_goal_position.py | ✓ | - | - | - | - |
| set_motor_speed_limit.py | ✓ | - | ✓ | - | - |
| control_motors_gui.py | ✓ | - | - | - | - |
| run_ros2_bridge.py | ✓ | - | - | - | - |
| run_moveit_demo.py | ✓ | - | - | - | ✓ |
| generate_urdf.py | ✓ | - | - | ✓ | - |
| diagnose_motor_calibration.py | ✓ | - | - | - | - |

---

## Quick Reference Card

```bash
# SETUP (once)
register_motors.py --port /dev/ttyUSB0 --scan --output configs/robot/my_robot.yaml
calibrate_motors.py --config configs/robot/my_robot.yaml

# AFTER EVERY CALIBRATION (critical!)
fix_goal_position.py --config configs/robot/my_robot.yaml

# DAILY USE
control_motors_gui.py --config configs/robot/my_robot.yaml
# OR
run_ros2_bridge.py --config configs/robot/my_robot.yaml

# TROUBLESHOOT
diagnose_motor_calibration.py --config configs/robot/my_robot.yaml

# MOVEIT
run_moveit_demo.py --config configs/robot/my_robot.yaml
```

---

For more detailed information, see:
- [SO-101 Setup Guide](SO101_SETUP_GUIDE.md)
- [Hardware Setup Guide](HARDWARE_SETUP.md)
- [MoveIt Integration Guide](MOVEIT_INTEGRATION.md)
