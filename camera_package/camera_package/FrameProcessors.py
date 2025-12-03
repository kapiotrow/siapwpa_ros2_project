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

    def visualize_input(self, frame, offset, control_vector=None):
        """
        Draw steering visualization and optionally all 3 control values.

        control_vector = [visibility, steering, forward_speed]
        """
        h, w, _ = frame.shape
        cx = w // 2

        # --- Draw lateral offset arrow (steering) ---
        steer_x = int(cx + offset * cx)
        cv2.line(frame, (cx, h - 20), (steer_x, h - 20), (0, 0, 255), 3)
        cv2.circle(frame, (steer_x, h - 20), 6, (0, 0, 255), -1)
        cv2.putText(frame, f"Offset: {offset:+.2f}", (30, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        if control_vector is not None:
            visibility, steering, fwd_speed = control_vector

            # --- Steering / yaw arrow ---
            arrow_length = int(steering * 100)
            cv2.arrowedLine(frame, (cx, h - 60), (cx + arrow_length, h - 120),
                            (255, 0, 0), 3, tipLength=0.4)
            cv2.putText(frame, f"Steering: {steering:+.2f}", (30, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

            # --- Forward speed visualization ---
            speed_bar_length = int(fwd_speed * 100)
            cv2.rectangle(frame, (30, h - 100), (30 + speed_bar_length, h - 90),
                          (0, 255, 0), -1)
            cv2.putText(frame, f"Speed: {fwd_speed:+.2f}", (30, h - 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)


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
        self.visualize_input(frame, offset, output)

        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

        return output
