from abc import ABC, abstractmethod
import cv2
import numpy as np

class FrameProcessor(ABC):
    @abstractmethod
    def process(self, frame):
        """
        Process an input frame and return:
        [visibility, offset, extra_data]

        Should also update internal state for PIDController.
        """
        pass

class BaseLineProcessor(FrameProcessor):
    def __init__(self):
        self.last_vector = [0.5, 0.0, 0.0]

    def visualize_input(self, frame, offset):
        """
        Draw steering visualization.
        """
        h, w, _ = frame.shape
        cx = w // 2
        steer_x = int(cx + offset * cx)
        cv2.line(frame, (cx, h - 20), (steer_x, h - 20), (0, 0, 255), 3)
        cv2.circle(frame, (steer_x, h - 20), 6, (0, 0, 255), -1)

    def process(self, frame):
        """
        Computes visibility + offset.
        Returns [visibility, offset, 0.0]
        """
        output = [0.5, 0.0, 0.0]

        try:
            if frame is None or frame.size == 0:
                print("Warning: Empty frame")
                return self.last_vector

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv,
                               np.array([15, 80, 80]),
                               np.array([45, 255, 255]))

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_NONE)

            h, w, _ = frame.shape
            cx = w // 2

            if contours:
                contours = sorted(contours, key=lambda c: max(p[0][1] for p in c), reverse=True)
                use = contours[:4] if len(contours) > 1 else contours

                pts = np.vstack(use).squeeze()
                if pts.ndim == 1:
                    pts = pts[np.newaxis, :]

                xs = pts[:, 0]
                ys = pts[:, 1]

                if len(xs) >= 3:
                    a, b, c = np.polyfit(ys, xs, 2)
                    y_eval = h - 50
                    x_eval = a*y_eval**2 + b*y_eval + c

                    offset = (x_eval - cx) / cx
                    offset = np.clip(offset, -1.0, 1.0)

                    output = [1.0, offset, 0.0]
                    self.last_vector = output
            else:
                print("Line lost - using fallback")

        except Exception as e:
            print("Process error:", e)

        # visualization
        visibility, offset, _ = output
        self.visualize_input(frame, offset)

        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

        return output
