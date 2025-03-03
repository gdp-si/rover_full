#!/bin/bash

# SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
# # shellcheck disable=SC1091
# source "$SCRIPT_DIR"/launch_at_startup.sh
# if [[ -z !"$DOCKER" ]]; then
#     # shellcheck disable=SC1091
#     source "$SCRIPT_DIR"/velodyne_auto_ip.sh
# fi

# # shellcheck disable=SC1091
# source /opt/ros/humble/setup.bash

# if [[ -z !"$DOCKER" ]]; then
#     # shellcheck disable=SC1091
#     source "$HOME"/ajai/project-roast/ros_ws/install/setup.bash
# else
#     # shellcheck disable=SC1091
#     source /workspaces/isaac_ros-dev/ros_ws/install/setup.bash
# fi

# # export PYTHONPATH=$PYTHONPATH:$HOME/ajai/roast/:$HOME/.local/lib/python3.8/site-packages

# # ros2 launch roast_bringup robot_bringup.launch.py
