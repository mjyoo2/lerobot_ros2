# LeRobot-ROS2 Motor Control System

A simple and intuitive control system for Feetech/Dynamixel motors with ROS2 integration and MoveIt support.

## 🚀 Quick Start

**[👉 Start with SO-101 Setup Guide](docs/SO101_SETUP_GUIDE.md)** ← Begin here!

## ⚡ Get Started in 5 Minutes

```bash
# 1. Install
pip install -e .
pip install pygame

# 2. Find port and scan motors
python scripts/register_motors.py --port /dev/ttyUSB0 --scan --output configs/robot/my_robot.yaml

# 3. Calibrate
python scripts/calibrate_motors.py --config configs/robot/my_robot.yaml

# 4. Sync Goal Position (REQUIRED!)
python scripts/fix_goal_position.py --config configs/robot/my_robot.yaml

# 5. Set speed limit (recommended for safety)
python scripts/set_motor_speed_limit.py --config configs/robot/my_robot.yaml --speed 200

# 6A. Control with GUI
python scripts/control_motors_gui.py --config configs/robot/my_robot.yaml

# OR 6B. Control with ROS2
python scripts/run_ros2_bridge.py --config configs/robot/my_robot.yaml
```

**Using ROS2:**
```bash
# Terminal 1: Run bridge
python scripts/run_ros2_bridge.py --config configs/robot/my_robot.yaml

# Terminal 2: Control motors
ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

## 📚 Available Scripts

### Basic Control
| Script | Description | When to Use |
|--------|-------------|-------------|
| `register_motors.py` | Find ports, scan motors | Once at setup |
| `calibrate_motors.py` | Calibrate (neutral position, range) | Once at setup |
| `fix_goal_position.py` | Sync Goal Position | **After every calibration** |
| `set_motor_speed_limit.py` | Set speed limits | As needed |
| `control_motors_gui.py` | GUI motor control | Use (Method A) |
| `run_ros2_bridge.py` | ROS2 topics control | Use (Method B) |
| `diagnose_motor_calibration.py` | Troubleshoot issues | When problems occur |

### MoveIt Integration (Advanced)
| Script | Description | When to Use |
|--------|-------------|-------------|
| `generate_urdf.py` | Generate URDF robot model | For MoveIt |
| `setup_moveit.py` | Generate MoveIt config | For MoveIt |
| `download_so_arm_urdf.py` | Download official SO-ARM URDFs | For SO-100/SO-101 |
| `run_moveit_demo.py` | Launch MoveIt + RViz | For motion planning |
| `execute_moveit_trajectory.py` | Execute MoveIt trajectories | For "Plan & Execute" |

## 🎯 Key Features

- ✅ **Simple workflow**: Control everything with 13 scripts
- ✅ **Safe calibration**: Set ranges by manually moving motors
- ✅ **Intuitive GUI**: Real-time motor control with sliders
- ✅ **ROS2 integration**: Programmable control via topics
- ✅ **MoveIt support**: Control gripper position in 3D space (advanced)
- ✅ **Feetech/Dynamixel support**: Works with STS3215, Dynamixel motors
- ✅ **Auto port detection**: Automatically finds USB ports

## 🔧 Troubleshooting

If you encounter issues:

```bash
# 1. Run diagnostics
python scripts/diagnose_motor_calibration.py --config configs/robot/my_robot.yaml

# 2. Check Goal Position sync
python scripts/fix_goal_position.py --config configs/robot/my_robot.yaml
```

**Common issues:**
- **Motors jump when torque enabled** → Run `fix_goal_position.py`
- **Cannot find motors** → Run `register_motors.py --find-port`
- **Motors move too fast** → Run `set_motor_speed_limit.py --speed 200`

## 📖 Documentation

- **[SO-101 Complete Setup Guide](docs/SO101_SETUP_GUIDE.md)** - Complete SO-101 setup from scratch
  - URDF download and configuration
  - Hardware setup and calibration
  - ROS2 and MoveIt integration
  - Troubleshooting guide

- **[Scripts Reference](docs/SCRIPTS_REFERENCE.md)** - Complete script reference
  - Detailed usage for each script
  - Command examples and options
  - Practical workflows
  - Troubleshooting

- **[CLAUDE.md](CLAUDE.md)** - Project overview and developer guide
  - Architecture and design decisions
  - Common commands and workflows
  - Reference implementations

### Legacy Documentation (Korean)
- **[QUICKSTART.md](QUICKSTART.md)** - 기본 모터 제어 시작하기 (Korean)
- **[MOVEIT_BRIDGE_GUIDE.md](MOVEIT_BRIDGE_GUIDE.md)** - MoveIt 통합 상세 가이드 (Korean)
- **[MOVEIT_SCRIPTS_GUIDE.md](MOVEIT_SCRIPTS_GUIDE.md)** - MoveIt 스크립트 가이드 (Korean)
- **[URDF_SETUP.md](URDF_SETUP.md)** - SO-101 URDF 사용 가이드 (Korean)

## 📁 Project Structure

```
LeRobot_ros2/
├── docs/                      # 📚 Complete documentation
│   ├── SO101_SETUP_GUIDE.md   # Complete SO-101 setup guide
│   └── SCRIPTS_REFERENCE.md   # All scripts reference
│
├── scripts/                   # 🔧 All control scripts
│   ├── register_motors.py     # Motor discovery
│   ├── calibrate_motors.py    # Calibration
│   ├── fix_goal_position.py   # Goal Position sync
│   ├── set_motor_speed_limit.py  # Speed configuration
│   ├── control_motors_gui.py  # GUI control
│   ├── run_ros2_bridge.py     # ROS2 bridge
│   ├── diagnose_motor_calibration.py  # Diagnostics
│   ├── generate_urdf.py       # URDF generation
│   ├── download_so_arm_urdf.py  # Download official URDFs
│   ├── setup_moveit.py        # MoveIt config generation
│   ├── run_moveit_demo.py     # MoveIt launcher
│   └── execute_moveit_trajectory.py  # Trajectory executor
│
├── src/lerobot_ros2/          # 💻 Core library
│   ├── hardware/              # Motor/camera controllers
│   ├── data_collection/       # ROS2 bridge
│   ├── configs/               # Config management
│   └── utils/                 # Utilities
│
├── configs/robot/             # ⚙️  Robot configurations
│   └── *.yaml                 # Robot config files
│
├── calibration/               # 📊 Calibration data
│   └── {robot}/
│       └── calibration.json
│
├── urdf/                      # 🤖 URDF models
│   ├── *.urdf                 # Robot descriptions
│   └── assets/                # 3D meshes
│
├── launch/                    # 🚀 ROS2 launch files
│   └── moveit_demo.launch.py
│
└── README.md                  # This file
```

## 🆘 Support

- **Getting Started**: [SO-101 Setup Guide](docs/SO101_SETUP_GUIDE.md)
- **Script Reference**: [Scripts Reference](docs/SCRIPTS_REFERENCE.md)
- **Project Guide**: [CLAUDE.md](CLAUDE.md)
- **GitHub Issues**: Report issues or ask questions

---

**New to this project?**
- **Basic motor control**: [SO-101 Setup Guide](docs/SO101_SETUP_GUIDE.md)
- **Advanced motion planning**: [MOVEIT_BRIDGE_GUIDE.md](MOVEIT_BRIDGE_GUIDE.md)

## 🎓 System Overview

### Supported Hardware

**Motors:**
- Feetech STS3215 series
- Dynamixel X-series (XM430, XM540)
- Communication: Serial (RS-485 or TTL)

**Robots:**
- SO-100 (5-DOF arm)
- SO-101 (6-DOF arm)
- Custom robots with Feetech/Dynamixel motors

### Coordinate Systems

The system uses three coordinate systems:

1. **Present_Position** (Motor firmware)
   - Range: 0-4095 (12-bit)
   - Raw motor encoder values

2. **Normalized** (Internal)
   - Range: -100 to +100
   - Calibration-based coordinates
   - `0` = neutral position

3. **Radians** (ROS2/URDF)
   - Standard ROS2 unit
   - Used in `/joint_states` and MoveIt

### Control Flow

```
User → ROS2 Topics → RobotBridge → Motors
                          ↓
                    /joint_states → RViz/MoveIt
```

## 🔐 Safety Features

- **Calibration-based limits**: Motors restricted to calibrated range
- **Speed limits**: Configurable maximum velocity
- **Goal Position sync**: Prevents sudden movements
- **Safety checker**: Validates commands before execution
- **Emergency stop**: Disconnect button in GUI

## 📦 Installation

### Requirements

- Python 3.10+
- Ubuntu 22.04+ or WSL2
- ROS2 Humble or Jazzy (optional, for ROS2 features)

### Install Package

```bash
# Clone repository
git clone https://github.com/yourusername/LeRobot_ros2.git
cd LeRobot_ros2

# Install in development mode
pip install -e .

# Install GUI support
pip install pygame

# Install ROS2 (optional, for ROS2 features)
# For Ubuntu 22.04:
sudo apt install ros-humble-desktop

# For Ubuntu 24.04:
sudo apt install ros-jazzy-desktop
```

### USB Permissions

```bash
# Add user to dialout group
sudo usermod -aG dialout $USER
# Log out and log back in

# Or temporary permission:
sudo chmod 666 /dev/ttyUSB0
```

## 🚀 Quick Test

Verify installation:

```bash
# Test motor discovery
python scripts/register_motors.py --find-port

# Test with GUI (after setup)
python scripts/control_motors_gui.py --config configs/robot/example_robot.yaml
```

## 🤝 Contributing

Contributions welcome! Please:

1. Follow existing code style
2. Add tests for new features
3. Update documentation
4. Submit pull requests to `main` branch

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

**Ready to get started? Follow the [SO-101 Setup Guide](docs/SO101_SETUP_GUIDE.md)!** 🎉
