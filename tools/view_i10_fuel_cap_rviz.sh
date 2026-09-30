#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
exec /usr/bin/python3 "$(dirname "${BASH_SOURCE[0]}")/view_i10_fuel_cap_rviz.py" "$@"
