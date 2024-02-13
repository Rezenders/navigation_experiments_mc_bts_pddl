#!/bin/bash

set -e

source "/opt/ros/foxy/setup.bash"
source "/mros_reasoner/install/setup.bash"

exec "$@"
