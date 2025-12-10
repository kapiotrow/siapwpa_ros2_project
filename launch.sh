#!/bin/bash

tmux new-session \; \
  send-keys "ros2 launch camera_package/launch/bridge.launch.py" C-m \; \
  split-window -h \; \
  send-keys ". ./sourceme.sh; gz sim racetrack.sdf" C-m \; \
  split-window -v \; \
  send-keys "python3 camera_package/camera_package/camera_pubsub.py" C-m \; \
  select-layout tiled

