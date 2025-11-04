#!/bin/bash
# ROS2 Docker Helper Script for macOS
# This script helps run ROS2 commands in a Docker container

set -e

# Configuration
ROS_DISTRO=${ROS_DISTRO:-"jazzy"}
IMAGE_NAME="osrf/ros:${ROS_DISTRO}-desktop"
CONTAINER_NAME="ros2-dev-${ROS_DISTRO}"
WORKSPACE_PATH="/Users/jacobmazelin/Main-Desktop/All_Code/UM/Wolverbot_Kickers/utaustinvilla3d"
WORKSPACE_MOUNT="/workspace"

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}ROS2 Docker Helper${NC}"
echo "Distro: ${ROS_DISTRO}"
echo ""

# Check if container exists
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo -e "${GREEN}Container ${CONTAINER_NAME} exists${NC}"
else
    echo "Creating container ${CONTAINER_NAME}..."
    docker run -it -d \
        --name "${CONTAINER_NAME}" \
        --network host \
        -v "${WORKSPACE_PATH}:${WORKSPACE_MOUNT}" \
        -w "${WORKSPACE_MOUNT}" \
        "${IMAGE_NAME}" \
        /bin/bash
    echo -e "${GREEN}Container created${NC}"
fi

# Start container if not running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Starting container..."
    docker start "${CONTAINER_NAME}"
fi

# Execute command in container
if [ $# -eq 0 ]; then
    echo "Opening interactive shell in container..."
    echo "Run 'source /opt/ros/${ROS_DISTRO}/setup.bash' to source ROS2"
    docker exec -it "${CONTAINER_NAME}" /bin/bash
else
    echo "Executing: $*"
    # Properly quote all arguments and pass them to bash -c
    # Use printf %q to escape each argument, then join them
    QUOTED_ARGS=()
    for arg in "$@"; do
        QUOTED_ARGS+=("$(printf '%q' "$arg")")
    done
    
    # If command is colcon build/list, automatically add --base-paths src
    # to avoid picking up root-level CMakeLists.txt
    CMD_ARGS="${QUOTED_ARGS[*]}"
    if [[ "$1" == "colcon" ]] && [[ "$2" == "build" || "$2" == "list" ]]; then
        if [[ "$CMD_ARGS" != *"--base-paths"* ]]; then
            CMD_ARGS="${CMD_ARGS} --base-paths src"
            echo "Note: Automatically adding --base-paths src to colcon command"
        fi
    fi
    
    CMD="source /opt/ros/${ROS_DISTRO}/setup.bash && cd ${WORKSPACE_MOUNT} && ${CMD_ARGS}"
    # Use -t instead of -it for non-interactive commands (builds, etc.)
    if [ -t 0 ]; then
        docker exec -it "${CONTAINER_NAME}" bash -c "${CMD}"
    else
        docker exec -t "${CONTAINER_NAME}" bash -c "${CMD}"
    fi
fi

