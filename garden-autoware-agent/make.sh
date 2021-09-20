#!/bin/bash
if [ ! -d $PWD/src/ros-bridge ]; then
    ln -s ~/ros-bridge $PWD/src/
fi
. ~/autoware.ai/install/setup.bash
catkin_make
