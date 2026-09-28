#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
exec ros2 launch rm_vision_bringup vision_bringup.launch.py "$@"
