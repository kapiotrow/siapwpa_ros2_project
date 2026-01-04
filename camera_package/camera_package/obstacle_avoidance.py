from sensor_msgs.msg import LaserScan
import numpy as np
import math
from enum import Enum
import time


class AvoidanceState(Enum):
    IDLE = 0
    TURNING = 1
    BYPASSING = 2
    RETURNING = 3


class ObstacleAvoidance():
    def __init__(self):
        self.state = AvoidanceState.IDLE
        self.turn_direction = 0          # +1 left, -1 right
        self.accumulated_turn = 0.0      # radians
        self.last_time = time.time()

        # ------------------
        # Parameters
        # ------------------
        self.obstacle_dist = 4.0

    
    def visualize(self, msg):
        pass


    def update(self, msg):
        now = time.time()
        dt = (now - self.last_time)
        self.last_time = now

        offset = 0.0
        forward = 0.0

        # Sector definitions
        front = self._get_sector_min(msg, -0.35, 0.35)
        left = self._get_sector_min(msg, 0.35, 1.7)
        right = self._get_sector_min(msg, -1.7, -0.35)


        # ------------------
        # FSM
        # ------------------
        if self.state == AvoidanceState.IDLE:
            if front < self.obstacle_dist:
                self.turn_direction = 1 if left > right else -1
                self.state = AvoidanceState.TURNING
                self.accumulated_turn = 0.0
            else:
                return  [0.0, 0.0, 0.0]# line follower

        elif self.state == AvoidanceState.TURNING:
            if front < self.obstacle_dist:
                forward = 0.7
                offset = self.turn_direction
                self.accumulated_turn += abs(offset) * dt
            else:
                self.state = AvoidanceState.BYPASSING

        elif self.state == AvoidanceState.BYPASSING:
            forward = 1.0
            if ((self.turn_direction == 1 and left > self.obstacle_dist) or
                (self.turn_direction == -1 and right > self.obstacle_dist)):
                self.state = AvoidanceState.RETURNING

        elif self.state == AvoidanceState.RETURNING:
            if self.accumulated_turn > 0.0:
                forward = 0.7
                offset = -self.turn_direction
                self.accumulated_turn -= abs(offset) * dt
            else:
                self.state = AvoidanceState.IDLE

        return [forward, offset, 1.0]
    

    def _get_sector_min(self, msg, angle_min, angle_max):
        """Return minimum range in a given angular sector."""
        ranges = []

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if angle_min <= angle <= angle_max and not math.isinf(r):
                ranges.append(r)

        return min(ranges) if ranges else float('inf')