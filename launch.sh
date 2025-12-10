#!/bin/bash

tmux new-session \; \
  send-keys ". ./sourceme.sh $1; ros2 launch camera_package/launch/bridge.launch.py" C-m \; \
  split-window -h \; \
  send-keys "sleep 1;. ./sourceme.sh $1; gz sim racetrack.sdf" C-m \; \
  split-window -v \; \
  send-keys "sleep 3; . ./sourceme.sh $1; python3 camera_package/camera_package/camera_pubsub.py" C-m \; \
  select-layout tiled

