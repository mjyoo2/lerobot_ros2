#!/bin/bash
# MoveIt helper script

echo "=================================="
echo "🚀 Starting MoveIt + ROS2 Bridge"
echo "=================================="
echo ""

# Library path configuration
export LD_LIBRARY_PATH=/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:$LD_LIBRARY_PATH

# Configuration file
CONFIG=${1:-"configs/robot/so101_scanned.yaml"}

echo "Config file: $CONFIG"
echo ""
echo "Step 1: Starting ROS2 Bridge..."
echo "   (Running in new terminal)"
echo ""

# Run ROS2 Bridge in background
gnome-terminal --title="ROS2 Bridge" -- bash -c "
    cd $(pwd)
    echo '=================================='
    echo '🔌 Running ROS2 Bridge...'
    echo '=================================='
    echo ''
    ./run.sh scripts/run_ros2_bridge.py --config $CONFIG
    exec bash
" &

# Wait for bridge initialization
echo "Waiting 3 seconds (Bridge initialization)..."
sleep 3

echo ""
echo "Step 2: Starting MoveIt..."
echo ""

# Launch MoveIt
ros2 launch launch/moveit_demo.launch.py robot_config:=$CONFIG

echo ""
echo "=================================="
echo "✅ Terminated"
echo "=================================="
