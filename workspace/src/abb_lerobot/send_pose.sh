#!/bin/bash
# Quick script to send pose to robot
# Usage: ./send_pose.sh <x> <y> <z> <roll_deg> <pitch_deg> <yaw_deg>
# Example: ./send_pose.sh 0.8 0.0 0.6 180 0 -90

cd "$(dirname "$0")/../.."
source install/setup.bash

python3 src/abb_lerobot/abb_lerobot/send_pose.py "$@"
