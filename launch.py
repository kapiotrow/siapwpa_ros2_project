#!/usr/bin/env python3
import subprocess
import signal
import sys
import os

# Commands to run in parallel
commands = [
    {
        "name": "ros_bridge",
        "cmd": ["ros2", "launch", "camera_package/launch/bridge.launch.py"],
        "log": "ros_bridge.log",
    },
    {
        "name": "gz_sim",
        "cmd": ["gz", "sim", "racetrack.sdf"],
        "log": "gz_sim.log",
    },
    {
        "name": "line_following",
        "cmd": ["python3", "./camera_package/camera_package/camera_pubsub.py"],
        "log": "line_following.log",
    },
    # {
    #     "name": "gt_check",
    #     "cmd": ["python3", "./racetrack_evaluator/position_subscriber.py"],
    #     "log": "gt_check.log",
    # },
]

procs = []

def start_process(entry):
    log_file = open(entry["log"], "w")
    print(f"Starting {entry['name']}... logging to {entry['log']}")
    p = subprocess.Popen(
        entry["cmd"],
        # stdout=log_file,
        stderr=subprocess.STDOUT,
        text=True
    )
    return p, log_file


def signal_handler(sig, frame):
    print("\nStopping all processes...")
    for p, _ in procs:
        p.terminate()
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    for entry in commands:
        p, log = start_process(entry)
        procs.append((p, log))

    print("All processes running. Press Ctrl+C to stop.")

    # Keep script alive
    for p, _ in procs:
        p.wait()
