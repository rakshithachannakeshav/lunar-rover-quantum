#!/usr/bin/env bash
# One-shot install of everything this project needs on Ubuntu 24.04 (Noble):
#   ROS 2 Jazzy desktop, Gazebo Harmonic + ros_gz, colcon/rosdep, sim tools,
#   and the Python stack (Qiskit, Qiskit-Aer, NetworkX, SciPy, OpenCV, ...)
#   for the system python3 that ROS nodes use.
#
# Usage (as root, or as a sudo user):
#   sudo bash scripts/install_all_jazzy.sh [linux-user]
# linux-user defaults to $SUDO_USER; it gets rosdep + pip --user packages.
set -eo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "Run with sudo: sudo bash $0" >&2
  exit 1
fi

TARGET_USER="${1:-${SUDO_USER:-ubuntu}}"
. /etc/os-release
if [ "$VERSION_CODENAME" != "noble" ]; then
  echo "Ubuntu 24.04 (noble) required for ROS 2 Jazzy; this is $PRETTY_NAME" >&2
  exit 1
fi
export DEBIAN_FRONTEND=noninteractive

echo "=== 1/6 locale + base tools ==="
apt-get update
apt-get install -y locales software-properties-common curl wget gnupg lsb-release \
  git build-essential cmake
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
add-apt-repository -y universe

echo "=== 2/6 ROS 2 apt repository ==="
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
  > /etc/apt/sources.list.d/ros2.list

echo "=== 3/6 Gazebo (OSRF) apt repository ==="
wget -q https://packages.osrfoundation.org/gazebo.gpg \
  -O /usr/share/keyrings/pkgs.osrfoundation.org-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs.osrfoundation.org-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
  > /etc/apt/sources.list.d/gazebo-stable.list
apt-get update

echo "=== 4/6 ROS 2 Jazzy + Gazebo Harmonic + sim tools ==="
apt-get install -y \
  ros-jazzy-desktop \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  gz-harmonic ros-jazzy-ros-gz \
  ros-jazzy-xacro ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-robot-state-publisher ros-jazzy-rviz2 ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-tf-transformations \
  ros-jazzy-tf2-ros ros-jazzy-tf2-tools ros-jazzy-urdf ros-jazzy-visualization-msgs

echo "=== 5/6 Python packages for the system python3 ==="
apt-get install -y python3-pip python3-pytest python3-numpy python3-scipy \
  python3-networkx python3-matplotlib python3-pil python3-yaml python3-opencv \
  python3-pandas python3-tqdm python3-seaborn python3-pynput
# Ubuntu 24.04 is PEP 668 "externally managed": --user + --break-system-packages.
# Qiskit 2.x requires NumPy 2, but ROS 2 Jazzy, python3-opencv and python3-scipy
# are built against Ubuntu's NumPy 1.26 (NumPy 2 in ~/.local breaks them with
# "numpy.core.multiarray failed to import"). Qiskit 1.x works with NumPy 1.26.
sudo -u "$TARGET_USER" -H python3 -m pip install --user --break-system-packages \
  "qiskit>=1.2,<2" "qiskit-aer>=0.15,<0.18" "qiskit-algorithms>=0.3.0" "numpy<2" "scipy<1.12"

echo "=== 6/6 rosdep ==="
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init
sudo -u "$TARGET_USER" -H rosdep update

echo "=== verify ==="
bash -c "source /opt/ros/jazzy/setup.bash && ros2 doctor --report >/dev/null 2>&1; echo ROS_DISTRO=\$ROS_DISTRO"
gz sim --version | head -1
sudo -u "$TARGET_USER" -H python3 -c \
  "import qiskit, qiskit_aer, networkx, scipy, numpy, cv2, PIL, matplotlib; print('python deps ok, qiskit', qiskit.__version__, 'aer', qiskit_aer.__version__)"
echo "=== ALL DEPENDENCIES INSTALLED ==="
