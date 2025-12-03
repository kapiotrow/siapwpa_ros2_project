from abc import ABC, abstractmethod
import numpy as np
from collections import deque
from camera_package.FrameProcessors import FrameProcessor

class Controller(ABC):
    """
    Abstract controller interface. All controllers compute steering using a frame.
    """

    @abstractmethod
    def compute(self, frame):
        """
        Must return [visibility, steering_value, 0.0]
        """
        pass

class PIDController:
    def __init__(self, P=0.3, I=0.0, D=0.12, history_length=10):
        self.P = P
        self.I = I
        self.D = D

        self.I_buffer = []
        self.D_prev_value = 0.0

        self.last_visibility = 0.0
        self.last_offset = 0.0

        self.processor = None  # injected later

    def attach_processor(self, processor: FrameProcessor):
        self.processor = processor

    def compute(self, frame):
        """
        1. Processor extracts visibility & offset from frame
        2. PID uses offset to compute yaw
        """
        if self.processor is None:
            raise RuntimeError("PIDController has no FrameProcessor attached")

        visibility, offset, _ = self.processor.process(frame)

        self.last_visibility = visibility
        self.last_offset = offset

        # --- PID CONTROL ---
        error = offset

        # Integral
        self.I_buffer.append(error)
        if len(self.I_buffer) > 10:
            self.I_buffer.pop(0)
        I_term = sum(self.I_buffer)

        # Derivative
        D_term = error - self.D_prev_value
        self.D_prev_value = error

        # PID output (yaw)
        yaw = self.P * error + self.I * I_term + self.D * D_term

        # Control vector: visibility, yaw, forward_speed
        return [visibility, yaw, 0.0]

