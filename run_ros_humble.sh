# ============================
# File: run_ros_humble.sh
# Purpose: Host-side launcher. Starts a ROS 2 Humble desktop container with GUI (X11) and host networking.
# Usage:  ./run_ros_humble.sh
# ============================
#!/usr/bin/env bash
set -euo pipefail

IMAGE="${IMAGE:-osrf/ros:humble-desktop}"

# Allow GUI apps from Docker for this session
xhost +local:docker >/dev/null 2>&1 || true

# Start container
exec docker run -it --rm \
  -e DISPLAY="${DISPLAY}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --network=host \
  --name ros_humble_tb3 \
  "${IMAGE}" bash


# ============================
# File: launch_gazebo.sh
# Purpose: Inside-container helper. Launches Gazebo with TurtleBot3 world using ROS2 Humble.
# Usage:  ./launch_gazebo.sh
# ============================
#!/usr/bin/env bash
set -euo pipefail

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

# Ensure TB3 Gazebo packages are present (idempotent)
if ! ros2 pkg list | grep -q '^turtlebot3_gazebo$'; then
  apt update
  apt install -y ros-humble-turtlebot3-gazebo
fi

# Launch Gazebo with ROS integration
exec ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py


# ============================
# File: spawn_robot.sh
# Purpose: Inside-container helper. Spawns TurtleBot3 if it didn’t appear automatically.
# Usage:  ./spawn_robot.sh
# ============================
#!/usr/bin/env bash
set -euo pipefail

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

MODEL_FILE="/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_${TURTLEBOT3_MODEL}/model.sdf"

# Wait for /spawn_entity service (Gazebo must be running from launch_gazebo.sh)
echo "[spawn_robot] Waiting for Gazebo /spawn_entity service..."
timeout 30 bash -c 'until ros2 service list | grep -q "/spawn_entity"; do sleep 1; done' || {
  echo "[spawn_robot] ERROR: /spawn_entity not available. Did you run launch_gazebo.sh?"
  exit 1
}

ros2 run gazebo_ros spawn_entity.py \
  -entity "turtlebot3_${TURTLEBOT3_MODEL}" \
  -file "${MODEL_FILE}" \
  -x 0 -y 0 -z 0.1


# ============================
# File: teleop.sh
# Purpose: Inside-container helper. Starts keyboard teleoperation for TurtleBot3.
# Usage:  ./teleop.sh
# ============================
#!/usr/bin/env bash
set -euo pipefail

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

# Show quick keys help
cat <<'EOF'
[teleop] Keys:
  i = forward,  , = backward,  j = turn left,  l = turn right,  k = stop
  w/x = increase/decrease linear speed,  e/c = increase/decrease angular speed
EOF

exec ros2 run teleop_twist_keyboard teleop_twist_keyboard


# ============================
# File: debug_topics.sh
# Purpose: Inside-container helper. Quick diagnostics to verify data flow.
# Usage:  ./debug_topics.sh
# ============================
#!/usr/bin/env bash
set -euo pipefail

echo "[debug] Nodes:"
ros2 node list || true
echo
echo "[debug] Topics:"
ros2 topic list || true
echo
echo "[debug] Echo /cmd_vel (Ctrl+C to stop):"
ros2 topic echo /cmd_vel


# ============================
# File: make_exec.sh
# Purpose: Convenience script (host or container). Marks all helper scripts as executable.
# Usage:  ./make_exec.sh
# ============================
#!/usr/bin/env bash
set -euo pipefail
chmod +x run_ros_humble.sh launch_gazebo.sh spawn_robot.sh teleop.sh debug_topics.sh
echo "Marked scripts executable."
