# LeRobot-ROS2 Documentation

Complete documentation for the LeRobot-ROS2 motor control system.

---

## 📚 Documentation Index

### Getting Started

- **[SO-101 Complete Setup Guide](SO101_SETUP_GUIDE.md)** ⭐ START HERE
  - Complete setup from hardware to MoveIt
  - URDF download and configuration
  - Step-by-step calibration
  - ROS2 and MoveIt integration
  - Comprehensive troubleshooting

### Reference Guides

- **[Scripts Reference](SCRIPTS_REFERENCE.md)**
  - Complete reference for all 13 scripts
  - Usage examples and options
  - Practical workflows
  - Command-line options

---

## 🎯 Choose Your Path

### Path 1: Basic Motor Control

**Goal:** Control motors with GUI or ROS2 topics

**Follow these steps:**
1. Read [SO-101 Setup Guide](SO101_SETUP_GUIDE.md) sections:
   - Prerequisites
   - Quick Start (5 minutes)
   - Detailed Setup (Steps 1-5)
2. Reference [Scripts Reference](SCRIPTS_REFERENCE.md) as needed

**Time:** ~15 minutes

---

### Path 2: Advanced MoveIt Integration

**Goal:** Motion planning with collision avoidance

**Follow these steps:**
1. Complete Path 1 first (basic motor control)
2. Read [SO-101 Setup Guide](SO101_SETUP_GUIDE.md) sections:
   - URDF Setup
   - MoveIt Integration
3. Reference [Scripts Reference](SCRIPTS_REFERENCE.md) for MoveIt scripts

**Time:** ~30 minutes additional

---

### Path 3: Development and Customization

**Goal:** Understand architecture and customize the system

**Follow these steps:**
1. Complete Path 1 and/or Path 2
2. Read project overview: [../CLAUDE.md](../CLAUDE.md)
3. Study source code in `src/lerobot_ros2/`
4. Reference implementation patterns

**Time:** Varies

---

## 🚀 Quick Links

### Essential Commands

```bash
# Hardware Setup
python scripts/register_motors.py --port /dev/ttyUSB0 --scan --output configs/robot/my_robot.yaml
python scripts/calibrate_motors.py --config configs/robot/my_robot.yaml
python scripts/fix_goal_position.py --config configs/robot/my_robot.yaml

# Basic Control
python scripts/control_motors_gui.py --config configs/robot/my_robot.yaml
# OR
python scripts/run_ros2_bridge.py --config configs/robot/my_robot.yaml

# MoveIt
python scripts/run_moveit_demo.py --config configs/robot/my_robot.yaml

# Diagnostics
python scripts/diagnose_motor_calibration.py --config configs/robot/my_robot.yaml
```

---

## 📖 Documentation Structure

```
docs/
├── README.md                   # This file - documentation index
├── SO101_SETUP_GUIDE.md       # Complete setup guide (START HERE)
└── SCRIPTS_REFERENCE.md       # All scripts reference

Root Directory:
├── CLAUDE.md                  # Project overview and developer guide
├── QUICKSTART.md             # Legacy Korean quick start
├── MOVEIT_BRIDGE_GUIDE.md    # Legacy Korean MoveIt guide
├── MOVEIT_SCRIPTS_GUIDE.md   # Legacy Korean scripts guide
└── URDF_SETUP.md             # Legacy Korean URDF guide
```

---

## 🎓 Learn By Example

### Example 1: First-Time Setup

```bash
# 1. Scan and register (creates config file)
python scripts/register_motors.py \
    --port /dev/ttyUSB0 \
    --scan \
    --output configs/robot/my_robot.yaml

# 2. Calibrate (hands-on, follow prompts)
python scripts/calibrate_motors.py --config configs/robot/my_robot.yaml

# 3. Sync Goal Position (prevents sudden movements!)
python scripts/fix_goal_position.py --config configs/robot/my_robot.yaml

# 4. Set safe speed (recommended)
python scripts/set_motor_speed_limit.py --config configs/robot/my_robot.yaml --speed 200

# 5. Test with GUI
python scripts/control_motors_gui.py --config configs/robot/my_robot.yaml
```

### Example 2: ROS2 Control

```bash
# Terminal 1: Run ROS2 bridge
python scripts/run_ros2_bridge.py --config configs/robot/my_robot.yaml

# Terminal 2: Monitor joint states
ros2 topic echo /joint_states

# Terminal 3: Send commands
ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray \
    "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

### Example 3: MoveIt Motion Planning

```bash
# Terminal 1: ROS2 bridge
python scripts/run_ros2_bridge.py --config configs/robot/my_robot.yaml

# Terminal 2: MoveIt demo (auto-generates URDF)
python scripts/run_moveit_demo.py --config configs/robot/my_robot.yaml

# In RViz:
# 1. Drag Interactive Marker to goal position
# 2. Click "Plan" → see green trajectory
# 3. Click "Execute" → robot moves!
```

---

## 🔧 Troubleshooting Quick Reference

### Problem: Motors jump when torque enabled

**Solution:**
```bash
python scripts/fix_goal_position.py --config configs/robot/my_robot.yaml
```

---

### Problem: Cannot find motors

**Solution:**
```bash
# Find available ports
python scripts/register_motors.py --find-port

# Check USB connection
ls /dev/tty*

# Fix permissions
sudo chmod 666 /dev/ttyUSB0
```

---

### Problem: Motors move too fast

**Solution:**
```bash
python scripts/set_motor_speed_limit.py --config configs/robot/my_robot.yaml --speed 150
```

---

### Problem: Need to re-calibrate

**Solution:**
```bash
# 1. Calibrate
python scripts/calibrate_motors.py --config configs/robot/my_robot.yaml

# 2. MUST sync Goal Position!
python scripts/fix_goal_position.py --config configs/robot/my_robot.yaml

# 3. Test
python scripts/control_motors_gui.py --config configs/robot/my_robot.yaml
```

---

### Problem: Something is wrong, not sure what

**Solution:**
```bash
# Run diagnostics
python scripts/diagnose_motor_calibration.py --config configs/robot/my_robot.yaml
```

---

## 💡 Tips and Best Practices

### Safety First

- ✅ Always run `fix_goal_position.py` after calibration
- ✅ Start with low speed limits (`--speed 150-200`)
- ✅ Keep hand on robot during initial tests
- ✅ Use `diagnose_motor_calibration.py` when in doubt

### Calibration Tips

- 📍 Choose a neutral pose where robot can move equally in all directions
- 🔄 Move joints slowly and smoothly during range recording
- 🚫 Don't force beyond mechanical limits
- ✅ Test immediately after calibration with GUI

### ROS2 Tips

- 📊 Monitor `/joint_states` to see current positions
- ⚡ Use normalized coordinates (-100 to +100) for predictability
- 🔄 ROS2 bridge auto-detects radians vs normalized
- 📈 Default publishing rate is 30 Hz (adjustable with `--rate`)

### MoveIt Tips

- 🎯 Start with simple goals (small movements)
- 👁️ Always check planned trajectory before executing
- ⚠️ Use collision-free paths
- 📉 Adjust joint limits if planning fails frequently

---

## 🆘 Getting Help

1. **Check Documentation:**
   - [SO-101 Setup Guide](SO101_SETUP_GUIDE.md)
   - [Scripts Reference](SCRIPTS_REFERENCE.md)

2. **Run Diagnostics:**
   ```bash
   python scripts/diagnose_motor_calibration.py --config configs/robot/my_robot.yaml
   ```

3. **Check Common Issues:**
   - Each guide has a troubleshooting section
   - See Quick Reference above

4. **Ask for Help:**
   - GitHub Issues
   - Include diagnostic output
   - Describe what you tried

---

## 📦 What's in Each Guide

### SO-101 Setup Guide
- ✅ Complete hardware setup
- ✅ URDF download and configuration
- ✅ Calibration procedures
- ✅ ROS2 integration
- ✅ MoveIt setup and usage
- ✅ Comprehensive troubleshooting
- ✅ File structure reference

### Scripts Reference
- ✅ All 13 scripts documented
- ✅ Command-line options
- ✅ Usage examples
- ✅ Workflows and patterns
- ✅ Quick reference card
- ✅ Dependencies and relationships

---

## 🚀 Next Steps

1. **New User?** Start with [SO-101 Setup Guide](SO101_SETUP_GUIDE.md)

2. **Need Script Help?** Check [Scripts Reference](SCRIPTS_REFERENCE.md)

3. **Developer?** Read [../CLAUDE.md](../CLAUDE.md) for architecture

4. **Ready to Control?** Run the quick start commands!

---

**Happy robot controlling! 🤖**
