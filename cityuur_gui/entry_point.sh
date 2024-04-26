#!/bin/bash

set -e

# setup ros2 environment
source "/opt/ros/humble/setup.bash"
source "/mnt/d/Users/Eric/Desktop/UR_workshop/cityuur_gui_main_ros2_test/cityuur_gui/install/setup.bash"

exec "$@"
