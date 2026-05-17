#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

if [ -f /root/ros2_PFE/install/setup.bash ]; then
    source /root/ros2_PFE/install/setup.bash
fi

exec "$@"
