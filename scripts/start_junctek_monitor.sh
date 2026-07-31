#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
source /home/sahabat/sahabat_ws/install/setup.bash

exec ros2 run shbat_pkg junctek_monitor
