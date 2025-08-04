#!/bin/bash
set -e

# Source the ROS 2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash
uv run blsm

# Execute the provided command or start a shell if no command is given
exec "$@"