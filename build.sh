#!/usr/bin/zsh

echo "Cleaning"
rm -rf build install log
sleep 3

echo "Building"
colcon build
sleep 3

echo "Sourcing the install file"
source install/setup.zsh