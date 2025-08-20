# ==============================
# TurtleBot3 Gazebo Simulation (ROS 2 Jazzy) on VirtualBox
# ==============================

# 1) Update system
sudo apt update && sudo apt upgrade -y

# 2) Install VirtualBox guest additions (graphics integration)
sudo apt install -y virtualbox-guest-utils virtualbox-guest-x11

# 3) Install ROS 2 Jazzy Desktop + dev tools
sudo apt install -y ros-jazzy-desktop ros-dev-tools

# 4) Source ROS setup in .bashrc
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc

# 5) Install TurtleBot3 packages
sudo apt install -y 'ros-jazzy-turtlebot3*'

# 6) Set TurtleBot3 model (Burger)
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc

# ==============================
# Launch Simulation
# ==============================

# Option A: Empty world
ros2 launch turtlebot3_gazebo empty_world.launch.py

# Option B: Prebuilt TurtleBot3 world
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# ==============================
# Teleoperation (new terminal)
# ==============================
ros2 run turtlebot3_teleop teleop_keyboard

# ==============================
# Troubleshooting (VirtualBox + Gazebo)
# ==============================

# If Gazebo shows black screen / crashes → run with software rendering:
export LIBGL_ALWAYS_SOFTWARE=1
gz sim -r empty.sdf
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# If still unstable → give VM more RAM/CPU/VRAM or disable 3D acceleration in VirtualBox.
# ==============================

# Next time after reboot:
# Just run → (if black screen, add LIBGL_ALWAYS_SOFTWARE=1 first)
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 run turtlebot3_teleop teleop_keyboard
