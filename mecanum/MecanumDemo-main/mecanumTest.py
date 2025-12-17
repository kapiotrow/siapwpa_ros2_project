# Python example using mecanum_pb2.py
import mecanum_pb2
import serial
import time
import math

# Open serial connection
ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)  # Adjust port as needed
time.sleep(2)

request = mecanum_pb2.ControlRequest()
request.speed_mmps = 100    # Move forward at 0-500 mm/s
request.rad = 0.5 * math.pi     # Straight line 0-2*PI, pi/2 is straight, 0 is right
request.omega = -2.0         # Rotation speed in rad/s, positive is CCW, range 0-2 rad/s

# Serialize the message
serialized_data = request.SerializeToString()

# Send to Arduino
ser.write(serialized_data)
time.sleep(2)

ser.close()