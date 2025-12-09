from abc import ABC, abstractmethod
import numpy as np
from collections import deque
from FrameProcessors import FrameProcessor
from scipy.linalg import solve_discrete_are

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
    def __init__(self, P=1, I=0.0, D=0.12, history_length=10):
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

class LQRController(Controller):
    def __init__(self, processor=None, dt=0.05):
        """
        LQR controller based on lateral offset dynamics.

        processor = FrameProcessor child (LineProcessor)
        """
        self.processor = processor
        self.dt = dt

        # Discrete linear lateral model
        self.A = np.array([
            [1.0, dt],
            [0.0, 1.0]
        ])

        self.B = np.array([
            [0.0],
            [1.0]
        ])

        # LQR tuning matrices
        self.Q = np.diag([5.0, 0.1])    # penalty on offset + derivative
        self.R = np.array([[0.4]])      # penalty on steering effort

        # Compute LQR gain matrix K
        self.K = self.solve_lqr(self.A, self.B, self.Q, self.R)

        # Memory for derivative estimate
        self.prev_offset = 0.0

    def attach_processor(self, processor: FrameProcessor):
        self.processor = processor

    def solve_lqr(self, A, B, Q, R):
        """
        Solve discrete LQR using the algebraic Riccati equation.
        Returns gain matrix K.
        """
        # Solve A'PA - P - A'PB(R + B'PB)^-1 B'PA + Q = 0
        P = np.matrix(solve_discrete_are(A, B, Q, R))
        K = np.matrix(np.linalg.inv(B.T @ P @ B + R) @ (B.T @ P @ A))
        return np.asarray(K)

    def compute(self, frame):
        """
        Matching PIDController behavior:
        Receives frame → processor extracts offset → LQR computes yaw
        Must return [visibility, yaw, 0.0]
        """
        visibility, offset, _ = self.processor.process(frame)

        # Estimate offset derivative
        offset_rate = (offset - self.prev_offset) / self.dt
        self.prev_offset = offset

        # State vector
        x = np.array([[offset], [offset_rate]])

        # LQR control law: u = -Kx
        u = float((self.K @ x))

        # Clip steering
        u = np.clip(u, -1.0, 1.0)

        return [visibility, -u, 0.0]

