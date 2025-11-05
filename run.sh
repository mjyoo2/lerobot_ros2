#!/bin/bash

if [ $# -eq 0 ]; then
    echo "Usage: ./run.sh <script> [args...]"
    echo ""
    echo "Examples:"
    echo "  ./run.sh scripts/calibrate_motors.py --config configs/robot/so101_scanned.yaml"
    echo "  ./run.sh scripts/run_ros2_bridge.py --config configs/robot/so101_scanned.yaml"
    echo "  ./run.sh scripts/run_moveit_demo.py --config configs/robot/so101_scanned.yaml"
    exit 1
fi

# Add uv to PATH if installed in ~/.local/bin
export PATH="/root/.local/bin:$HOME/.local/bin:$PATH"


if command -v uv &> /dev/null; then
    uv run "$@"
elif command -v python3 &> /dev/null; then
    python3 "$@"
else
    python "$@"
fi
