#!/usr/bin/env bash
set -eo pipefail

# Lunar Rover — ROS 2 Jazzy + Gazebo Harmonic workspace setup (WSL)
# Run inside Ubuntu 24.04 WSL.

WS_ROOT=~/lunar-rover-quantum

# Auto-detect Windows project path (adjust drive letter if needed)
if [ -d "/mnt/d/lunar-rover-quantum/sim_files" ]; then
  WIN_SRC="/mnt/d/lunar-rover-quantum/sim_files"
elif [ -d "/mnt/c/Users/Rakshitha/lunar-rover-quantum/sim_files" ]; then
  WIN_SRC="/mnt/c/Users/Rakshitha/lunar-rover-quantum/sim_files"
else
  echo "Set WIN_SRC to your sim_files folder path"
  exit 1
fi

DEST="$WS_ROOT/src/rover_simulation"
mkdir -p "$DEST"/{urdf,worlds,config,launch,models,rover_simulation}

cp "$WIN_SRC/rover.urdf.xacro"              "$DEST/urdf/"
cp "$WIN_SRC/lunar_terrain.world"           "$DEST/worlds/"
cp "$WIN_SRC/config/"*                      "$DEST/config/" 2>/dev/null || true
cp "$WIN_SRC/rviz_config.rviz"              "$DEST/config/" 2>/dev/null || true
cp "$WIN_SRC/launch/"*.py                   "$DEST/launch/"
cp "$WIN_SRC/package.xml"                   "$DEST/"
cp "$WIN_SRC/CMakeLists.txt"                "$DEST/"
cp "$WIN_SRC/setup.py"                      "$DEST/"
cp "$WIN_SRC/rover_simulation/"*.py         "$DEST/rover_simulation/"
cp -r "$WIN_SRC/models/"*                   "$DEST/models/" 2>/dev/null || true

# Terrain assets (generate if missing)
mkdir -p "$WIN_SRC/worlds" "$DEST/worlds"
if [ ! -f "$DEST/worlds/heightmap.png" ]; then
  echo "Generating terrain textures (requires Python + Pillow + scipy)..."
  python3 "$WIN_SRC/generate_heightmap.py" || true
  python3 "$WIN_SRC/generate_textures.py" 2>/dev/null || true
  cp "$WIN_SRC/worlds/"*.png "$DEST/worlds/" 2>/dev/null || true
fi

chmod +x "$DEST/rover_simulation/"*.py

echo "Files copied to $DEST"

source /opt/ros/jazzy/setup.bash
cd "$WS_ROOT"
colcon build --symlink-install
echo "Workspace built"

BASHRC="$HOME/.bashrc"
WS_SOURCE="source $WS_ROOT/install/setup.bash"
grep -Fxq "$WS_SOURCE" "$BASHRC" || echo "$WS_SOURCE" >> "$BASHRC"

source "$WS_ROOT/install/setup.bash"
echo ""
echo "Before launch, kill stale Gazebo:  pkill -9 gz 2>/dev/null || true"
echo ""
echo "Terminal 1 — start sim (NO Gazebo window by default):"
echo "  ros2 launch rover_simulation demo.launch.py mode:=creep"
echo ""
echo "Terminal 2 — open Gazebo window (while Terminal 1 runs):"
echo "  bash /mnt/d/lunar-rover-quantum/sim_files/open_gazebo_gui.sh"
echo ""
echo "Or RViz instead of Gazebo:"
echo "  ros2 launch rover_simulation demo.launch.py mode:=creep use_rviz:=true"
echo ""
echo "Check rover moving:"
echo "  ros2 topic echo /odom --field pose.pose.position"
echo ""
echo "Terminal 2 — keyboard:"
echo "  ros2 run rover_simulation rover_keyboard.py"
echo ""
echo "Web UI (open in browser on Windows):"
echo "  file:///mnt/d/lunar-rover-quantum/web/index.html"
