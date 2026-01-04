#!/usr/bin/env bash

set -e

mkdir -p ~/pincer_ws/src
cd ~/pincer_ws/src

git clone -b testing https://github.com/thisisdipanjan/pincerRover.git

sudo apt update && sudo apt upgrade -y

sudo apt install -y curl wget git vim nano htop tmux screen \
net-tools iputils-ping dnsutils software-properties-common build-essential \
cmake unzip pkg-config python3-pip python3-argcomplete python3-venv \
python3-setuptools libssl-dev libboost-all-dev libusb-1.0-0-dev \
libudev-dev libserial-dev python3-opencv python3-numpy python3-yaml python3-pillow \
libeigen3-dev iproute2 traceroute nmap netcat tcpdump

sudo apt install -y python3-colcon-common-extensions python3-rosdep \
python3-vcstool python3-rosinstall-generator python3-rosinstall

cd ~/pincer_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

echo "source ~/pincer_ws/install/setup.bash" >> ~/.bashrc
source ~/pincer_ws/install/setup.bash

echo "pincer_ws ready for development"
