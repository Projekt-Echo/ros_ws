#!/usr/bin/zsh

echo "Cleaning"
rm -rf build install log
sleep 3

echo "Building"
colcon build
sleep 3

source install/setup.zsh