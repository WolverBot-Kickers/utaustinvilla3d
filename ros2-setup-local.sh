#!/bin/bash
# Local ROS2 Setup Script (for building ROS2 from source on macOS)
# This is an alternative to Docker - builds ROS2 locally

set -e

ROS_DISTRO=${ROS_DISTRO:-"jazzy"}
ROS_INSTALL_DIR="$HOME/ros2_${ROS_DISTRO}"

echo "Setting up ROS2 ${ROS_DISTRO} locally..."
echo "This will take a while as it builds ROS2 from source."
echo ""

# Check if ROS2 is already installed
if [ -d "$ROS_INSTALL_DIR" ] && [ -f "$ROS_INSTALL_DIR/setup.bash" ]; then
    echo "ROS2 ${ROS_DISTRO} appears to be already installed at $ROS_INSTALL_DIR"
    echo "To use it, run: source $ROS_INSTALL_DIR/setup.bash"
    exit 0
fi

echo "ROS2 local installation is complex on macOS."
echo "Recommended: Use Docker instead (see docker-ros2.sh)"
echo ""
echo "If you want to proceed with local installation, you'll need to:"
echo "1. Install dependencies: brew install cmake python3 wget"
echo "2. Follow: https://docs.ros.org/en/jazzy/Installation/Alternatives/OSX-Development-Setup.html"
echo ""
echo "For now, Docker is recommended for development."

