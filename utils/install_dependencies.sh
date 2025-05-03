
set -e  # Exit on any error
set -o pipefail

echo "======================================="
echo " Installing project dependencies... "
echo "======================================="

# 1. Update package lists
echo "[1/3] Updating apt repositories..."
sudo apt update

# 2. Install libgpiod-dev
echo "[2/3] Installing libgpiod-dev (for GPIO control)..."
sudo apt install -y libgpiod-dev

echo "installing picam4 stuff"
#sudo apt install -y raspi-config

echo "rp camera_stuff"
sudo apt install -y libcamera-apps v4l-utils


sudo apt install -y ros-foxy-serial-driver
