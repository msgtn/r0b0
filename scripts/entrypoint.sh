#!/bin/bash
set -e

# Source the ROS 2 environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Run directly from the venv instead of uv run for faster startup
/r0b0/.venv/bin/blsm

# Execute the provided command or start a shell if no command is given
exec "$@"
