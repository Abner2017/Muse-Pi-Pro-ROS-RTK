#!/bin/bash
set -euo pipefail
# start_fbrtk_driver.sh
# - waits briefly for the device (if needed)
# - sources ROS/workspace setup
# - execs roslaunch so PID 1 of the service is roslaunch (via su in systemd)

DEVICE="/dev/ttyACM2"
WORKSPACE="/home/cat/catkin_ws"

cd "$WORKSPACE"

# wait up to 60s for device to appear (non-blocking if already present)
for i in $(seq 1 60); do
  if [ -e "$DEVICE" ]; then
    break
  fi
  sleep 1
done

if [ ! -e "$DEVICE" ]; then
  echo "[start_fbrtk_driver] WARNING: device $DEVICE not found after wait" >&2
  # proceed anyway; systemd ExecStartPre will have attempted chmod earlier
fi

# source the workspace setup if available (the workspace setup usually sources /opt/ros/...)
if [ -f /opt/ros/noetic/setup.bash ]; then
  # ensure minimal ROS env vars are set for profile.d scripts (these scripts can
  # assume variables exist); adapt these values if your ROS master is remote.
  export ROS_DISTRO=noetic
  export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
  # shellcheck disable=SC1090
  source /opt/ros/noetic/setup.bash
fi

# source the workspace setup if available (the workspace setup usually sources /opt/ros/...)
if [ -f "$WORKSPACE/devel/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$WORKSPACE/devel/setup.bash"
else
  echo "[start_fbrtk_driver] ERROR: workspace setup.bash not found at $WORKSPACE/devel/setup.bash" >&2
  exit 1
fi

echo "[start_fbrtk_driver] launching roslaunch as user $(whoami)"

# use exec so this process is replaced by roslaunch (good for systemd supervision)
exec roslaunch fbrtkros fbrtk_driver.launch
