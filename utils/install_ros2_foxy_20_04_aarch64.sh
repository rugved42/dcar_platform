#!/bin/bash

set -e  # Exit immediately if a command exits with a non-zero status
set -o pipefail

echo "======================================"
echo " Installing ROS 2 Foxy on Ubuntu 20.04 "
echo " Architecture: aarch64                "
echo "======================================"

# STEP 1: Setup Locale
echo "[1/7] Setting locale..."
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# STEP 2: Setup Sources
echo "[2/7] Setting up sources..."
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# STEP 3: Add ROS 2 GPG Key
echo "[3/7] Adding ROS 2 GPG key..."
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -

# STEP 4: Add ROS 2 Repository
echo "[4/7] Adding ROS 2 repository..."
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# STEP 5: Install Development Tools and ROS 2 Packages
echo "[5/7] Installing ROS 2 Foxy packages..."
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstool \
    python3-argcomplete \
    ros-foxy-desktop

# (Optional) Install more packages
# sudo apt install -y ros-foxy-navigation2 ros-foxy-nav2-bringup

# STEP 6: Initialize rosdep
echo "[6/7] Initializing rosdep..."
sudo rosdep init || echo "rosdep already initialized."
rosdep update

# STEP 7: Environment Setup
echo "[7/7] Setting up environment variables..."
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "======================================"
echo " ROS 2 Foxy Installation Complete! 🎉 "
echo " Remember to source ROS2:             "
echo "   source /opt/ros/foxy/setup.bash    "
echo "======================================"
