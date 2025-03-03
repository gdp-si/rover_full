#!/bin/bash
# install-realsense-dependencies.sh
# Install dependencies for  the Intel Realsense library librealsense2 on a Jetson Nano Developer Kit
# Copyright (c) 2016-19 Jetsonhacks
# MIT License

# red=$(tput setaf 1)
green=$(tput setaf 2)
reset=$(tput sgr0)
# e.g. echo "${red}red text ${green}green text${reset}"
echo "${green}Adding Universe repository and updating${reset}"
apt-add-repository universe
apt-get update
echo "${green}Adding dependencies, graphics libraries and tools${reset}"
apt-get install libssl-dev libusb-1.0-0-dev pkg-config -y

# Graphics libraries - for SDK's OpenGL-enabled examples
apt-get install libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev -y

# QtCreator for development; not required for librealsense core library
apt-get install qtcreator -y

# Add Python 3 support
apt-get install -y python3 python3-dev

# Install librealsense2 udev rules, scripts and tools
mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp >/dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" |
    tee /etc/apt/sources.list.d/librealsense.list
apt-get update

apt-get install -y librealsense2-dkms librealsense2-utils
