# Hanka Mecanum Line-Following Robot

## Requirements
### For Gazebo/ROS2 simulation
* Ubuntu OS (tested only on 22.04 LTS)
* Nvidia RTX graphics

### For hardware
* Hanka :)
* RPi 5 with Debian Bookworm
* Intel Realsense 2 camera

## Usage
**Use main branch for simulation and raspberry-pi for hardware!**
### Simulation
After cloning the repo and making sure you're on `main` branch, **reopen the foder in container** (e.g. with VS Code's Remote Explorer extension). Open a new terminal and build the ROS package.
```
colcon build
```

Then source `sourceme.sh` with a parameter (ROS ID, choose one that noone else is using in your network).
```
. ./sourceme.sh 121
```
Finally, launch the simulation.
```
ros2 launch camera_package bridge.launch.py
```
After a couple of second you should see a new window appear with Gazebo. Play the simulation using the "play" button in the left bottom corner.

### Hardware
Connect Hanka's USB and Realsense's USB into the RPi. Power up the board. On your PC connect to the RPi using SSH. Clone the repository and `git checkout raspberry-pi` into the correct branch. Navigate to the project's folder and reopen it in container.
Power up Hanka, make sure the camera can see the line (it should be **red**!).
Open a new terminal and run the control script.
```
python3 mecanum/control.py
```
The script can take a couple of seconds to start.
