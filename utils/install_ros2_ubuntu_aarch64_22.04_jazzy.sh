#!/bin/bash

set -e  # Exit on any error

echo "🧼 Removing conflicting/outdated PPA..."
#sudo add-apt-repository --remove -y ppa:ubuntu-raspi2/ppa || true
#sudo rm -f /etc/apt/sources.list.d/ubuntu-raspi2-ppa-jammy.list

echo "🔄 Updating package index..."
sudo apt update && sudo apt upgrade -y

echo "📦 Installing dependencies..."
sudo apt install -y software-properties-common curl gnupg lsb-release

echo "🔐 Setting up ROS 2 GPG key and repository..."
# Correct key setup for Jammy (Ubuntu 22.04)
sudo mkdir -p /etc/apt/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /etc/apt/keyrings/ros-archive-keyring.gpg

echo "📋 Adding ROS 2 repository..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "🔄 Updating package index with ROS 2 sources..."
sudo apt update

echo "🌱 Installing ROS 2 Humble (Desktop version)..."
sudo apt install -y ros-humble-desktop

echo "✅ Sourcing ROS 2 setup..."
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi
source ~/.bashrc

echo "📦 Installing ROS 2 build tools and utilities..."
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool build-essential

echo "🔧 Initializing rosdep..."
sudo rosdep init || true
rosdep update

echo "🎉 ROS 2 Humble installed successfully!"
