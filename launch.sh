#!/usr/bin/env bash
set -e

SESSION="racetrack"
ENV="$1"

tmux new-session -d -s "$SESSION"

tmux send-keys -t "$SESSION" ". ./sourceme.sh $ENV; ros2 launch camera_package/launch/bridge.launch.py" C-m
tmux split-window -h -t "$SESSION"
tmux send-keys -t "$SESSION" "sleep 1; . ./sourceme.sh $ENV; gz sim racetrack.sdf" C-m
tmux split-window -v -t "$SESSION"
tmux send-keys -t "$SESSION" "sleep 3; . ./sourceme.sh $ENV; python3 camera_package/camera_package/camera_pubsub.py" C-m
tmux split-window -h -t "$SESSION"
tmux send-keys -t "$SESSION" "sleep 1; . ./sourceme.sh $ENV; python3 racetrack_evaluator/position_subscriber.py" C-m
tmux split-window -h -t "$SESSION"
tmux send-keys -t "$SESSION" "sleep 1; . ./sourceme.sh $ENV; python3 lidar_scripts/lidarNavigation.py" C-m

tmux select-layout -t "$SESSION" tiled

# ---------- EXIT KEY BINDINGS ----------
tmux bind-key -n C-c run-shell "
  tmux list-panes -t $SESSION -F '#{pane_id}' | while read p; do
    tmux send-keys -t \$p C-c
  done;
  sleep 1;
  pkill -f gz || true;
  pkill -f ros2 || true;
  tmux kill-session -t $SESSION
"

tmux bind-key q run-shell "
  tmux list-panes -t $SESSION -F '#{pane_id}' | while read p; do
    tmux send-keys -t \$p C-c
  done;
  sleep 1;
  tmux kill-session -t $SESSION
"

tmux attach -t "$SESSION"

