#!/bin/bash
set -e

# setup ros environment
source "/home/bridge/ros2_foxy_bridge_ws/install/setup.bash"
exec "$@"
