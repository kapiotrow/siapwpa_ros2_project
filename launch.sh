ros2 launch camera_package/launch/bridge.launch.py    #odpala brdige na potrzebne topici
gz sim racetrack.sdf                                  #odpala swiat

#podązanie za linią
python3 ./camera_package/camera_package/camera_pubsub.py

#potencjalnie do sprawdzenia GT
python3 ./racetrack_evaluator/position_subscriber.py
