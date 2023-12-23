#!/bin/bash
set -ex

# setup ros environment in Docker Container
source "/opt/ros/$ROS_DISTRO/setup.bash" --
cd /
# create workspace directory where package will be built
mkdir workspace
cd workspace
cp -r ${GITHUB_WORKSPACE}/. src/
# Update and install packages
apt-get update
apt-get -y install python3-pip
# Install catkin build
pip3 install -U catkin_tools
# Install gen3_compliant_controllers package deps
apt-get install python3-rosdep
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y
# Install format checker
apt-get install clang-format-10 -y
apt-get update

# Install pinocchio
apt install -qqy lsb-release gnupg2 curl
echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
apt-get update
apt install -qqy robotpkg-py3*-pinocchio

export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH # Adapt your desired python version here
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH

#build
catkin build 