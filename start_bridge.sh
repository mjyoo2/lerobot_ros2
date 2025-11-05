#!/bin/bash
# ROS2 Bridge startup script

echo "=================================="
echo "🔌 Starting ROS2 Bridge"
echo "=================================="
echo ""

CONFIG=${1:-"configs/robot/so101_scanned.yaml"}

echo "Config: $CONFIG"
echo ""

./run.sh scripts/run_ros2_bridge.py --config $CONFIG
