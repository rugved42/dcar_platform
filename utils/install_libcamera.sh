#!/bin/bash

set -e  # Exit on error

INSTALL_DIR="$HOME/libcamera_install"
LIBCAMERA_DIR="$HOME/libcamera"
MESON_VERSION=">=0.63"

echo "📦 Installing build dependencies..."
sudo apt update
sudo apt install -y \
  git \
  cmake \
  ninja-build \
  build-essential \
  libgnutls28-dev \
  openssl \
  python3-pip \
  python3-setuptools \
  python3-wheel \
  python3-pybind11 \
  python3-jinja2 \
  python3-yaml \
  libtiff-dev \
  libjpeg-dev \
  libpng-dev \
  libavcodec-dev \
  libavformat-dev \
  libswscale-dev \
  libv4l-dev \
  v4l-utils

echo "📦 Installing latest Meson via pip (user local)..."
pip3 install --user "meson$MESON_VERSION"
pip3 install --user ninja

export PATH="$HOME/.local/bin:$PATH"

echo "📁 Cloning libcamera..."
cd ~
git clone https://git.linuxtv.org/libcamera.git || true
cd libcamera

echo "⚙️ Setting up Meson build with local install prefix..."
meson setup build --prefix="$INSTALL_DIR" --buildtype=release

echo "🔨 Building libcamera..."
ninja -C build

echo "📥 Installing to $INSTALL_DIR (no sudo needed)..."
ninja -C build install

echo "✅ libcamera built and installed locally at $INSTALL_DIR"

echo "📌 Add the following to your ~/.bashrc:"
echo "export PATH=\"$INSTALL_DIR/bin:\$PATH\""
echo "export LD_LIBRARY_PATH=\"$INSTALL_DIR/lib:\$LD_LIBRARY_PATH\""
echo "export PKG_CONFIG_PATH=\"$INSTALL_DIR/lib/pkgconfig:\$PKG_CONFIG_PATH\""
