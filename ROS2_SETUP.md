# ROS2 Setup Guide for macOS

This guide helps you set up ROS2 for the wbk_yolo package on macOS.

## Quick Start (Recommended: Docker)

### 1. Install colcon (already done!)
```bash
# Already installed via pip3
export PATH="$HOME/Library/Python/3.9/bin:$PATH"
```

### 2. Pull ROS2 Docker Image
```bash
docker pull osrf/ros:jazzy-desktop
```

### 3. Build Your Package
```bash
# Using the helper script
./docker-ros2.sh colcon build --packages-select wbk_yolo

# Or manually
./docker-ros2.sh bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace && colcon build --packages-select wbk_yolo"
```

### 4. Run Your Nodes
```bash
# Interactive shell in container
./docker-ros2.sh

# Then inside the container:
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
ros2 run wbk_yolo distance_node
```

## Alternative: Local ROS2 Installation

If you prefer not to use Docker, you can build ROS2 from source on macOS. This is more complex but gives you native performance.

See: https://docs.ros.org/en/jazzy/Installation/Alternatives/OSX-Development-Setup.html

## Helper Scripts

- **`docker-ros2.sh`** - Runs ROS2 commands in Docker container
- **`build-wbk-yolo.sh`** - Builds the wbk_yolo package (works with Docker or local)
- **`ros2-setup-local.sh`** - Guide for local installation

## Usage Examples

### Build Package
```bash
./docker-ros2.sh colcon build --packages-select wbk_yolo
```

### Run Node
```bash
./docker-ros2.sh ros2 run wbk_yolo distance_node
```

### Interactive Shell
```bash
./docker-ros2.sh
# Then inside:
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
ros2 run wbk_yolo yolo_node
ros2 run wbk_yolo distance_node
```

### Check Topics
```bash
./docker-ros2.sh ros2 topic list
./docker-ros2.sh ros2 topic echo /vision/ball_distance
```

## Troubleshooting

### Docker issues
- Make sure Docker Desktop is running
- Check: `docker ps`

### Colcon not found
- Add to ~/.zshrc: `export PATH="$HOME/Library/Python/3.9/bin:$PATH"`
- Then: `source ~/.zshrc`

### ROS2 not found in container
- Make sure you source: `source /opt/ros/jazzy/setup.bash`
- Check ROS_DISTRO: `echo $ROS_DISTRO`

## Next Steps

1. Fix the `package.xml` typo (line 4: `<n>` → `<name>`)
2. Pull Docker image: `docker pull osrf/ros:jazzy-desktop`
3. Build package: `./docker-ros2.sh colcon build --packages-select wbk_yolo`
4. Test node: `./docker-ros2.sh ros2 run wbk_yolo distance_node`

