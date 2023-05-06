#!/bin/bash
set -e

# setup ros environment
source "/app/install/setup.bash"

ros2 launch skuid_description skuid_description.launch.py autostart:=true use_rviz:=True

# takes any command given as argument and executes it, namely the "command on the docker compose file of the service"
exec "$@"