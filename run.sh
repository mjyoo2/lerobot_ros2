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

# Add uv to PATH if installed in ~/.local/bin or ~/.cargo/bin
export PATH="$HOME/.cargo/bin:$HOME/.local/bin:/root/.local/bin:$PATH"

# Check if uv is available
if command -v uv &> /dev/null; then
    # Check if uv.lock exists but .venv doesn't exist or is outdated
    if [ -f "uv.lock" ] && [ ! -d ".venv" ]; then
        echo "🔄 First run detected. Installing dependencies with uv..."
        uv sync
    fi

    # Run the script with uv (automatically handles virtual environment)
    uv run "$@"
else
    echo "⚠️  uv not found. Installing uv..."
    echo "   Run: curl -LsSf https://astral.sh/uv/install.sh | sh"
    echo ""
    echo "   Or install manually from: https://github.com/astral-sh/uv"
    echo ""
    echo "   After installation, run: source ~/.cargo/env"
    echo ""
    echo "🔄 Falling back to python3..."

    # Fallback: check if dependencies are installed
    if ! python3 -c "import dynamixel_sdk" 2>/dev/null; then
        echo "⚠️  Dependencies not installed. Installing with pip..."
        pip install -e .
    fi

    python3 "$@"
fi
