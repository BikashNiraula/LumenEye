# LumenEye
LumenEye 4-DOF Robotic Lamp – A ROS 2 + Gazebo project featuring a 4-DOF lamp that dynamically provides shadow-free workspace illumination. Uses camera-based perception for tool and hand detection, with real-time planning and control visualized in RViz.

## 🚀 Setup & Run

Build, source, and launch the project in one go:

```bash
# Source ROS 2 Humble installation
source /opt/ros/humble/setup.bash

# Build workspace with symlink installs (for development)
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Launch the system
ros2 launch lss_arm_controller lumeneye.launch.py
