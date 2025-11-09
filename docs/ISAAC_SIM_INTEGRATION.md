# Isaac Sim Integration Guide

Complete guide for setting up Isaac Sim with LeRobot-ROS2 for robot simulation and teleoperation.

---

## Table of Contents

1. [Overview](#overview)
2. [Installation](#installation)
3. [ROS2 Bridge Setup](#ros2-bridge-setup)
4. [Robot Model Import](#robot-model-import)
5. [Action Graph Creation](#action-graph-creation)
6. [Teleoperation Setup](#teleoperation-setup)
7. [Troubleshooting](#troubleshooting)
8. [FAQ](#faq)
9. [Resources](#resources)

---

## Overview

This guide covers:
- Installing Isaac Sim and configuring ROS2 bridge
- Importing robot models (URDF → USD)
- Creating Action Graphs for ROS2 communication
- Setting up real robot ↔ Isaac Sim teleoperation
- Common issues and solutions

### System Data Flow

```
Real Robot (manual movement with --no-torque)
    ↓
RobotBridge publishes /joint_states
    ↓
TeleopRelay relays to /isaac_joint_commands
    ↓
Isaac Sim ROS2 Action Graph
    ↓
Isaac Sim robot follows movements
```

### Key ROS2 Topics

- `/joint_states` - Real robot joint states (published by RobotBridge)
- `/isaac_joint_commands` - Commands to Isaac Sim (published by TeleopRelay)
- `/isaac_joint_states` - Isaac Sim robot state (published by Isaac Sim)

---

## Installation

### Prerequisites

- **Isaac Sim**: Version 5.1.0+ (latest stable recommended)
- **ROS2**: Humble (Ubuntu 22.04) or Jazzy (Ubuntu 24.04)
- **Ubuntu**: 22.04 or 24.04
- **Python**: 3.11 (Isaac Sim only supports Python 3.11)
- **GPU**: NVIDIA RTX series or better
- **LeRobot-ROS2**: Installed and working with your real robot

**Important**: Isaac Sim only works with Python 3.11. Ensure ROS2 is sourced before launching Isaac Sim.

### Option 1: Standalone Installation (Recommended)

Isaac Sim 5.1.0+ supports standalone installation.

1. **Download Isaac Sim**:
   - Official page: https://developer.nvidia.com/isaac-sim
   - Choose your platform (Linux x86_64 recommended)

2. **Extract**:
   ```bash
   # Extract to desired location
   # Example: ~/isaac-sim/ or /opt/isaac-sim/
   ```

3. **Run post-install script** (Linux):
   ```bash
   cd isaac-sim
   ./post_install.sh
   ```

4. **Launch Isaac Sim**:
   ```bash
   # Linux
   ./isaac-sim.selector.sh

   # Windows
   isaac-sim.selector.bat
   ```

5. Click **Start** button to launch application

   **Note**: First launch may take several minutes. Wait even if window appears blank.

### Option 2: Omniverse Launcher (Traditional)

1. Download NVIDIA Omniverse Launcher: https://www.nvidia.com/en-us/omniverse/
2. Install and launch Omniverse Launcher
3. Go to **Exchange** tab
4. Find **Isaac Sim** and click **Install**
5. Launch from **Library** tab after installation

### Verify Installation

Test with a basic example:
1. Launch Isaac Sim
2. File → Open → Load a built-in example (e.g., Simple Room + Franka Robot)
3. Click Play button (▶) to start simulation

---

## ROS2 Bridge Setup

Official docs: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html

Isaac Sim includes built-in ROS2 support through the **ROS2 Bridge** extension.

### Step 1: Configure ROS2 Environment (Before Launching Isaac Sim)

**Critical**: Set up ROS2 environment **before** launching Isaac Sim!

```bash
# Source ROS2 (Ubuntu 22.04 - Humble)
source /opt/ros/humble/setup.bash

# Source ROS2 (Ubuntu 24.04 - Jazzy)
source /opt/ros/jazzy/setup.bash

# Set ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0

# Set RMW implementation (FastDDS recommended)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Now launch Isaac Sim
cd ~/isaac-sim  # or your Isaac Sim installation path
./isaac-sim.selector.sh
```

**Or add permanently to ~/.bashrc**:
```bash
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc
source ~/.bashrc
```

**Important**: Environment variables set after Isaac Sim is already running will not take effect!

### Step 2: Enable ROS2 Bridge Extension

1. Launch Isaac Sim (after setting environment variables)
2. Go to **Window → Extensions**
3. Search for "ROS"
4. Enable these extensions:
   - `omni.isaac.ros2_bridge` (required)
   - `omni.isaac.ros2_bridge-humble` (if using ROS2 Humble)
   - `omni.isaac.ros2_bridge-jazzy` (if using ROS2 Jazzy)
5. Close Extensions window

### Step 3: Configure Domain ID in Isaac Sim

Isaac Sim and LeRobot-ROS2 must use the same ROS_DOMAIN_ID.

In Isaac Sim:
- **Edit → Preferences → ROS**
- Set **Domain ID** to match your system (default: 0)

### Step 4: Multi-Machine Communication (Optional)

For communication between Isaac Sim and ROS2 on different machines, create `fastdds.xml`:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_transport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>
</profiles>
```

Set environment variable:
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds.xml
```

---

## Robot Model Import

Import your robot's URDF/USD model into Isaac Sim.

### Method 1: Import URDF Directly (Recommended)

1. In Isaac Sim, go to **Isaac Utils → Workflows → URDF Importer**

2. Click **Browse** and select your robot URDF file:
   - For SO-101: `/home/y2/lerobot_ros2/urdf/so101_new_calib.urdf`

3. Configuration options:
   ```
   Input File: /path/to/your/robot.urdf
   Output Directory: /path/to/output/
   Output Filename: robot_imported.usd

   ✅ Fix Base Link (if robot base should be fixed)
   ✅ Import Inertia Tensor (recommended)

   Joint Drive Type: Position
   Joint Drive Stiffness: 0        ← Important! (lower = smoother)
   Joint Drive Damping: 5000       ← Important! (higher = less vibration)
   ```

4. Click **Import**

5. Verify output USD file is created

### Method 2: Add Robot to Scene

1. **File → New** (create new stage)

2. **Create World**:
   - Right-click in Stage panel → **Create → Xform**
   - Name: `World`

3. **Add Robot**:
   - Right-click `/World` → **Add → Reference**
   - Select file: `/path/to/robot_imported.usd`
   - Robot appears as `/World/robot_name`

4. **Adjust Robot Position** (optional):
   - Select robot in Stage panel
   - In Property panel, adjust Transform → Translate

### Verify Robot Import

1. Robot should appear in **Stage** panel
2. Check all joints are listed in robot hierarchy
3. Test joint movement:
   - Select a joint in Stage
   - In **Property** panel, adjust joint angle
   - Robot should move in viewport

---

## Action Graph Creation

Action Graph connects ROS2 topics to robot joints in Isaac Sim.

### Method 1: Manual UI Creation (Recommended)

MCP-based creation has technical limitations. **Creating Action Graph through UI is most reliable.**

#### Prerequisites

- ✅ Isaac Sim is running
- ✅ Robot loaded at `/World/robot_name`
- ✅ ROS2 bridge extensions enabled

#### Step-by-Step

1. **Open Action Graph Editor**:
   - **Window → Visual Scripting → Action Graph**

2. **Create New Graph**:
   - Click **"New Action Graph"** button
   - Graph created at `/ActionGraph`

3. **Add Nodes**:

   Search for and drag these nodes into graph:

   **a. On Playback Tick**
   - Type: `omni.graph.action.OnPlaybackTick`
   - Purpose: Triggers graph execution each frame

   **b. ROS2 Context**
   - Type: `omni.isaac.ros2_bridge.ROS2Context`
   - Settings: domain_id = `0`
   - Purpose: ROS2 connection context

   **c. Isaac Read Simulation Time**
   - Type: `omni.isaac.core_nodes.IsaacReadSimulationTime`
   - Purpose: Provides timestamps for ROS messages

   **d. ROS2 Publish Joint State**
   - Type: `omni.isaac.ros2_bridge.ROS2PublishJointState`
   - Settings:
     - topicName: `/isaac_joint_states`
     - targetPrim: Browse → select `/World/robot_name`
   - Purpose: Publishes robot state to ROS2

   **e. ROS2 Subscribe Joint State**
   - Type: `omni.isaac.ros2_bridge.ROS2SubscribeJointState`
   - Settings:
     - topicName: `/isaac_joint_commands`
   - Purpose: Receives commands from ROS2

   **f. Isaac Articulation Controller**
   - Type: `omni.isaac.core_nodes.IsaacArticulationController`
   - Settings:
     - robotPath: `/World/robot_name`
   - Purpose: Controls robot joints

4. **Connect Nodes**:

   Drag from output ports to input ports:

   ```
   OnPlaybackTick.tick → PublishJointState.execIn
   OnPlaybackTick.tick → SubscribeJointState.execIn
   OnPlaybackTick.tick → ArticulationController.execIn

   ROS2Context.context → PublishJointState.context
   ROS2Context.context → SubscribeJointState.context

   ReadSimTime.simulationTime → PublishJointState.timeStamp

   SubscribeJointState.jointNames → ArticulationController.jointNames
   SubscribeJointState.positionCommand → ArticulationController.positionCommand
   SubscribeJointState.velocityCommand → ArticulationController.velocityCommand
   SubscribeJointState.effortCommand → ArticulationController.effortCommand
   ```

5. **Save**:
   - **File → Save As**
   - Name: `robot_teleop.usd`

#### Visual Reference

```
[OnPlaybackTick]
       |
       +---> [PublishJointState] ← [ReadSimTime]
       |           ↑
       |           | [ROS2Context]
       |           ↓
       +---> [SubscribeJointState]
       |           |
       |           ↓
       +---> [ArticulationController]
```

### Method 2: Script Editor (Alternative)

If you prefer scripting, use Isaac Sim's **Script Editor**:

1. Window → Script Editor
2. Paste the following code:

```python
import omni.graph.core as og
import usdrt.Sdf

robot_path = "/World/robot_name"  # Change to your robot path

og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
            ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
            ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
            ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
            ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
            ("Context.outputs:context", "PublishJointState.inputs:context"),
            ("Context.outputs:context", "SubscribeJointState.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
            ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("ArticulationController.inputs:robotPath", robot_path),
            ("PublishJointState.inputs:topicName", "isaac_joint_states"),
            ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
            ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(robot_path)]),
        ],
    },
)

print("Action Graph created!")
```

3. Press **Run** button (or Ctrl+Enter)

**Note**: This works in Script Editor but may fail in MCP due to environment differences.

### Verify Action Graph

After setup:
1. ✅ Action Graph visible in graph editor
2. ✅ Click **Play** button (▶) in Isaac Sim
3. ✅ Verify ROS2 topics in terminal:

```bash
ros2 topic list
# Should show:
#   /isaac_joint_states
#   /isaac_joint_commands

ros2 topic info /isaac_joint_states
# Publishers: 1

ros2 topic echo /isaac_joint_states --no-arr
```

---

## Teleoperation Setup

Real robot ↔ Isaac Sim bidirectional teleoperation.

### Quick Start (If Already Configured)

#### 1. Launch Isaac Sim

```bash
# Set environment variables first
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

cd ~/isaac-sim
./isaac-sim.sh
```

- File → Open saved robot scene + action graph
- Click **PLAY** button (▶)

#### 2. Terminal 1: Real Robot Bridge (Torque OFF)

```bash
cd /home/y2/lerobot_ros2
source /opt/ros/humble/setup.bash

python src/lerobot_ros2/data_collection/robot_bridge.py \
    --config configs/robot/so101_scanned.yaml \
    --rate 30 \
    --no-torque
```

**Expected output**: `⚠️  Torque disabled - motors can be moved by hand`

#### 3. Terminal 2: Teleop Relay

```bash
cd /home/y2/lerobot_ros2
source /opt/ros/humble/setup.bash

python scripts/isaac_teleop_relay.py
```

#### 4. Test

Move the real robot **by hand slowly** → Isaac Sim robot should follow in real-time!

### Detailed Setup Steps

#### Terminal 1: Real Robot Bridge

```bash
cd /home/y2/lerobot_ros2
source /opt/ros/humble/setup.bash

# --no-torque option: Motor torque OFF (manual movement enabled)
python src/lerobot_ros2/data_collection/robot_bridge.py \
    --config configs/robot/so101_scanned.yaml \
    --rate 30 \
    --no-torque
```

**Verification**:
```
[INFO] ⚠️  Torque disabled - motors can be moved by hand
[INFO]    (Commands will be ignored)
[INFO] 🔍 Joint-Motor mapping verification:
[INFO]   [0] joint_names[0] = 'shoulder_pan' → motor_ids[0] = 1
...
[INFO] ✓ Hardware connected successfully
```

#### Terminal 2: Teleop Relay

```bash
cd /home/y2/lerobot_ros2
source /opt/ros/humble/setup.bash

python scripts/isaac_teleop_relay.py
```

**Verification**:
```
================================================================================
Isaac Sim Teleoperation Relay Started
================================================================================
Subscribing to: /joint_states (real robot)
Publishing to:  /isaac_joint_commands (Isaac Sim)

Move the real robot to see Isaac Sim follow!
================================================================================
```

#### Terminal 3: Isaac Sim

Should already be running with **PLAY** button (▶) pressed.

#### Terminal 4: Monitoring (Optional)

```bash
source /opt/ros/humble/setup.bash

# List topics
ros2 topic list

# Real-time data
ros2 topic echo /joint_states --no-arr
ros2 topic echo /isaac_joint_commands --no-arr

# Message frequency
ros2 topic hz /joint_states
ros2 topic hz /isaac_joint_commands
```

### Testing

1. **Move real robot by hand slowly**
2. **Verify Isaac Sim robot follows in real-time**
3. **Move each joint individually to verify mapping**

---

## Troubleshooting

**Detailed troubleshooting guide**: See `trouble-shooting/isaac-sim-teleop-issues.md`

### Quick Checklist

#### Isaac Sim Robot Not Moving

1. ✅ Isaac Sim **PLAY** button (▶) pressed?
2. ✅ `ROS_DOMAIN_ID=0` set before launching Isaac Sim?
3. ✅ `ros2 topic info /isaac_joint_commands` → Subscription count: 1?
4. ✅ Action Graph running? (green play icon in graph editor)

#### Robot Vibrating/Oscillating

Adjust Joint Drive parameters:
- **Damping**: 5000-10000 (higher = smoother)
- **Stiffness**: 0 (lower = more stable)

In Stage panel, select each joint → Property → `drive:angular:physics` → adjust values

#### Joint Mapping Issues

```bash
# Check bridge logs
[INFO] 🔍 Joint-Motor mapping verification:
[INFO]   [0] joint_names[0] = 'shoulder_pan' → motor_ids[0] = 1
```

Verify order matches, then re-import URDF in Isaac Sim if needed

#### Gripper Stuck at -0.174

URDF gripper limit is incorrect. Check URDF and re-import in Isaac Sim:
```bash
grep -A 2 'joint name="gripper"' urdf/so101_new_calib.urdf
```

Ensure `lower="-1.74533"` (not `-0.174533`)

#### ROS2 Topics Not Visible

1. **Shutdown** Isaac Sim
2. Set environment variables:
   ```bash
   source /opt/ros/humble/setup.bash
   export ROS_DOMAIN_ID=0
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   ```
3. **Restart** Isaac Sim

### Rendering Issues

**Problem**: Isaac Sim shows black screen, FPS=0, renderer shows as `None`

**Solutions**:

1. **Use alternative renderer** (pxr instead of rtx):
   ```bash
   cd /isaac-sim
   ./isaac-sim.sh \
       --/app/renderer/activeRenderer="pxr" \
       --/rtx/enabled=false
   ```

2. **Run headless** (no rendering, physics only):
   ```bash
   cd /isaac-sim
   ./isaac-sim.headless.sh
   ```

   **Note**: Headless mode is sufficient for ROS2 teleop since rendering is not required for physics simulation and ROS2 communication.

3. **Check WSL2 driver detection**: In some WSL2 environments, Isaac Sim may misdetect NVIDIA driver version. Use pxr renderer or headless mode as workaround.

### Common Issues

**Node Type Not Found**

Problem: Cannot find `omni.isaac.ros2_bridge.ROS2Context`

Solution: Enable ROS2 bridge extension first:
- Window → Extensions
- Search: "ros2 bridge"
- Enable `omni.isaac.ros2_bridge`

**Graph Not Executing**

Solution: OnPlaybackTick node needs to trigger. In standalone scripts:
```python
og.Controller.set(og.Controller.attribute("/ActionGraph/OnPlaybackTick.state:enableImpulse"), True)
```

In UI mode, simulation playback automatically triggers it.

---

## FAQ

### Q1: I set environment variables after launching Isaac Sim and it doesn't work

**A**: Isaac Sim reads environment variables at startup. **Shutdown Isaac Sim**, set environment variables, then **restart**.

### Q2: What happens if I run without --no-torque?

**A**: Motor torque is enabled, robot becomes rigid. Cannot move by hand, only controllable via ROS2 commands. Use `--no-torque` for teleop!

### Q3: Why do only gripper and wrist_roll have offset?

**A**: URDF joint limits are smaller than actual robot range, causing Isaac Sim to clamp values. Re-import URDF with correct limits.

### Q4: How to do bidirectional control (Isaac Sim → Real Robot)?

**A**: Run RobotBridge without `--no-torque`:
```bash
python src/lerobot_ros2/data_collection/robot_bridge.py \
    --config configs/robot/so101_scanned.yaml
```

**Warning**: Real robot may move suddenly. Ensure safety precautions!

### Q5: Which method is better - Manual UI or Script Editor?

**A**:
- **Manual UI**: Most reliable, easier debugging, better control
- **Script Editor**: Faster if you're familiar with code

Start with Manual UI, especially for first-time setup.

### Q6: Robot is floating in air in Isaac Sim

**A**: Enable "Fix Base Link" option when importing URDF. This fixes robot base to ground.

---

## Resources

### Official Documentation

- **Isaac Sim 5.1.0 Docs**: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/
- **Quick Install Guide**: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/quick-install.html
- **ROS2 Install Guide**: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html
- **ROS2 Bridge Tutorials**: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/

### Project Documentation

- **Main Guide**: `CLAUDE.md`
- **Troubleshooting**: `trouble-shooting/isaac-sim-teleop-issues.md`
- **SO-101 Setup**: `docs/SO101_SETUP_GUIDE.md`
- **Scripts Reference**: `docs/SCRIPTS_REFERENCE.md`

### Utilities

Real-time joint monitoring:
```bash
# Check joint differences (Real vs Isaac Sim)
./run.sh scripts/check_joint_diff.py

# Analyze joint ranges
./run.sh scripts/analyze_joint_range.py
```

---

## Summary: Quick Checklist

### Initial Setup (One-time)
- [ ] Install Isaac Sim
- [ ] Install ROS2 Humble/Jazzy
- [ ] Convert URDF to USD
- [ ] Create ROS2 Action Graph
- [ ] Adjust joint drive parameters (prevent vibration)
- [ ] Save scene

### Each Run
- [ ] Set environment variables (`ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`)
- [ ] Launch Isaac Sim + Load scene + Press PLAY
- [ ] Terminal 1: RobotBridge (`--no-torque`)
- [ ] Terminal 2: TeleopRelay
- [ ] Test by moving real robot

### Verification
- [ ] ROS2 topics `/joint_states`, `/isaac_joint_commands` active
- [ ] Isaac Sim robot follows real robot in real-time
- [ ] Joint mapping is correct
- [ ] Movement is smooth without vibration

---

**Created for LeRobot-ROS2 Teleoperation System**
**Isaac Sim Version**: 5.1.0+
**Supported ROS2**: Humble (Ubuntu 22.04), Jazzy (Ubuntu 24.04)
**Last Updated**: 2025-01-09
