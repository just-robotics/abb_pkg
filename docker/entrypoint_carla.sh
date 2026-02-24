#!/bin/bash
set -e


bash /workspace/ImportAssets.sh

exec bash CarlaUE4.sh --ros2 -nosound
