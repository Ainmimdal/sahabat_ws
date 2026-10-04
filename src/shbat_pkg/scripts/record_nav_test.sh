#!/bin/bash
# Record a supervised Nav2 motion test for offline analysis.
#
# Start the robot normally (Sahabat Robot app, Operating mode) and localize it
# first. Run this script, send the test goals from the web console with a
# person beside the emergency stop, then press Ctrl+C to finish the bag.
#
# This script only records. It never publishes or moves the robot.
#
# Usage: record_nav_test.sh [label]     e.g. record_nav_test.sh behind_goal

source /opt/ros/humble/setup.bash
source "$HOME/sahabat_ws/install/setup.bash"

label="${1:-nav_test}"
bag_dir="$HOME/sahabat_ws/bags"
output="$bag_dir/$(date +%Y%m%d_%H%M%S)_${label}"
mkdir -p "$bag_dir"

topics=(
    # Sensors: raw and filtered lidar, wheel/IMU/fused odometry
    /scan_raw /scan /wheel_odom /imu /odom
    # Frames and localization
    /tf /tf_static /amcl_pose /initialpose
    # Planning and control chain, from controller to motors
    /plan /local_costmap/costmap /local_costmap/costmap_updates
    /cmd_vel_nav /cmd_vel_nav_smoothed /cmd_vel
    # Goals, results and safety state
    /goal_pose /navigate_to_pose/_action/status
    /navigate_through_poses/_action/status /emergency_stop
    # Node logs, including controller and AMCL warnings
    /rosout
)

echo "Recording to $output"
echo "Send goals with a person at the emergency stop. Ctrl+C to finish."
exec ros2 bag record -o "$output" "${topics[@]}"
