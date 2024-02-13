#!/bin/bash

set -e

source "/usr/share/gazebo/setup.sh"
source "/opt/ros/foxy/setup.bash"
source "/mros_reasoner/install/setup.bash"

exec "$@"
