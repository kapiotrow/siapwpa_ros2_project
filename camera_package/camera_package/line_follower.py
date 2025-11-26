import cv2
import numpy as np
import pytest
from unittest.mock import patch
import time
from collections import deque

import cv2
import numpy as np
from collections import deque

import cv2
import numpy as np
from collections import deque

class LineFollower:
    def __init__(self):
        self.last_vector = [0.5, 0.0, 0.0]  # Default output (searching)
        self.P = 0.8
        self.I = 0.005
        self.D = 0.1
        self.D_prev_value = 0.0

        self.I_window_size = 30
        self.I_buffer = deque(maxlen=self.I_window_size)

        # Pure Pursuit parameters
        self.lookahead_distance = 50  # pixels ahead for PP

    def process_frame(self, frame):
        """
        Process frame and compute curve-based offset.
        Returns [visibility, offset, 0.0]
        """
        output_vector = [0.5, 0.0, 0.0]

        try:
            if frame is None or frame.size == 0:
                print("Warning: Empty frame.")
                return self.last_vector

            # HSV conversion and yellow mask
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_yellow = np.array([15, 80, 80])
            upper_yellow = np.array([45, 255, 255])
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            height, width, _ = frame.shape
            center_x = width // 2

            if contours:
                # Sort contours by bottom-most y
                def bottom_y(c):
                    return max(p[0][1] for p in c)
                contours = sorted(contours, key=bottom_y, reverse=True)
                contours_to_use = contours[:4] if len(contours) > 1 else contours

                # Merge points
                all_points = np.vstack(contours_to_use).squeeze()
                if all_points.ndim == 1:
                    all_points = all_points[np.newaxis, :]
                xs = all_points[:, 0]
                ys = all_points[:, 1]

                if len(xs) >= 3:
                    # Fit quadratic curve
                    poly = np.polyfit(ys, xs, 2)
                    a, b, c = poly

                    # Evaluate curve only on blobs
                    y_min, y_max = ys.min(), ys.max()
                    y_vals = np.linspace(y_min, y_max, num=50)
                    x_vals = a*y_vals**2 + b*y_vals + c

                    # Compute offset at bottom of image (PID)
                    y_eval = height - 50
                    x_eval = a*y_eval**2 + b*y_eval + c
                    offset = (x_eval - center_x) / center_x
                    offset = np.clip(offset, -1.0, 1.0)

                    output_vector = [1.0, offset, 0.0]
                    self.last_vector = output_vector

                    # Draw curve only on blobs
                    for i in range(len(x_vals)-1):
                        cv2.line(frame, (int(x_vals[i]), int(y_vals[i])),
                                 (int(x_vals[i+1]), int(y_vals[i+1])),
                                 (0, 255, 0), 2)

                    # # Draw arrow
                    # cv2.arrowedLine(frame, (center_x, height - 50),
                    #                 (int(center_x + offset*100), height - 120),
                    #                 (0, 255, 0), 4, tipLength=0.4)
                    # cv2.putText(frame, f"Curve Dir: {offset:+.2f}", (30, 40),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                else:
                    print("Not enough points for curve fitting.")
                    output_vector = [0.5, 0.0, 0.0]
            else:
                print("Line lost — waiting for it to reappear...")
                output_vector = [0.5, 0.0, 0.0]

        except Exception as e:
            print(f"sException: {e}")
            output_vector = [0.5, 0.0, 0.0]

        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

        return output_vector

    def pid_control(self, frame):
        """
        Standard PID controller based on curve offset.
        """
        info_vector = self.process_frame(frame)
        error = info_vector[1]

        # Integral
        self.I_buffer.append(error)
        I_term = sum(self.I_buffer)

        # Derivative
        D_term = error - self.D_prev_value
        self.D_prev_value = error

        # PID output
        yaw = self.P * error + self.I * I_term + self.D * D_term
        return [info_vector[0], yaw, 0.0]

    def pure_pursuit_control(self, frame):
        """
        Pure Pursuit controller along the curve.
        Returns [visibility, steering_angle, 0.0]
        """
        try:
            if frame is None or frame.size == 0:
                return self.last_vector

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_yellow = np.array([15, 80, 80])
            upper_yellow = np.array([45, 255, 255])
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            height, width, _ = frame.shape
            center_x = width // 2
            center_y = height

            if contours:
                def bottom_y(c):
                    return max(p[0][1] for p in c)
                contours = sorted(contours, key=bottom_y, reverse=True)
                contours_to_use = contours[1:] if len(contours) > 1 else contours

                all_points = np.vstack(contours_to_use).squeeze()
                if all_points.ndim == 1:
                    all_points = all_points[np.newaxis, :]
                xs = all_points[:, 0]
                ys = all_points[:, 1]

                if len(xs) >= 3:
                    poly = np.polyfit(ys, xs, 2)
                    a, b, c = poly

                    # Find lookahead point along curve
                    y_min, y_max = ys.min(), ys.max()
                    y_lookahead = min(y_max, center_y - self.lookahead_distance)
                    x_lookahead = a*y_lookahead**2 + b*y_lookahead + c

                    # Compute steering angle using Pure Pursuit formula
                    dx = x_lookahead - center_x
                    dy = center_y - y_lookahead
                    steering_angle = np.arctan2(dx, dy)  # radians
                    return [1.0, steering_angle, 0.0]

        except Exception as e:
            print(f"PP Exception: {e}")

        return [0.5, 0.0, 0.0]





def test_line_follower_manual_display():
    """
    Manual visual test — opens OpenCV windows to display frames and prints output vectors.
    Press 'q' to quit.
    """
    lf = LineFollower()
    path = "/home/developer/ros2_ws/src/camera_package/camera_package/test_video.avi"
    cap = cv2.VideoCapture(path)
    assert cap.isOpened(), "Failed to open test video"

    print("\nManual Line Follower Test Started.")
    print("Press 'q' in the video window to stop.\n")

    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("End of video reached.")
            break

        # Process frame and get output vector
        output_vector = lf.process_frame(frame)

        # Print output for debugging
        print(f"Frame {frame_count:04d} — Output: {output_vector}")

        frame_count += 1

        # Wait for 'q' key to quit early
        key = cv2.waitKey(10) & 0xFF
        if key == ord('q'):
            print("Manual stop requested.")
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Test completed successfully.\n")
