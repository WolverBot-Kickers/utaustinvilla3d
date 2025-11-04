#!/bin/bash
# Build script for wbk_yolo package
# Works with both local ROS2 and Docker

set -e

WORKSPACE_DIR="/Users/jacobmazelin/Main-Desktop/All_Code/UM/Wolverbot_Kickers/utaustinvilla3d"
cd "$WORKSPACE_DIR"

# Check if we're in Docker or local
if [ -f "/.dockerenv" ] || [ -n "${ROS_DISTRO}" ]; then
    # In Docker - source ROS2
    source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
    echo "Building in Docker environment..."
else
    # Local - try to source ROS2 if available
    if [ -f "$HOME/ros2_jazzy/setup.bash" ]; then
        source "$HOME/ros2_jazzy/setup.bash"
        echo "Using local ROS2 installation..."
    elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
        echo "Using system ROS2 installation..."
    else
        echo "ERROR: ROS2 not found!"
        echo "Please either:"
        echo "  1. Use Docker: ./docker-ros2.sh colcon build --packages-select wbk_yolo"
        echo "  2. Install ROS2 locally"
        exit 1
    fi
fi

# Add colcon to PATH if needed
export PATH="$HOME/Library/Python/3.9/bin:$PATH"

# Build
echo "Building wbk_yolo package..."
colcon build --packages-select wbk_yolo

echo ""
echo "Build complete! Source the workspace with:"
echo "  source install/setup.bash"

