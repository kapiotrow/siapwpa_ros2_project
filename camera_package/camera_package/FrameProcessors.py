from abc import ABC, abstractmethod
import cv2
import numpy as np
import math

import pygame

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

            lower_red1 = np.array([0, 80, 80])
            upper_red1 = np.array([10, 255, 255])

            # górny zakres czerwieni
            lower_red2 = np.array([170, 80, 80])
            upper_red2 = np.array([179, 255, 255])

            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

            mask = cv2.bitwise_or(mask1, mask2)
            # mask = cv2.inRange(hsv,
            #                    np.array([15, 80, 80]),
            #                    np.array([45, 255, 255]))

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
        # self.visualize_input(frame, offset, output)

        # cv2.imshow("Frame", frame)
        # cv2.waitKey(1)
        


        return frame

class LineKalmanProcessor(FrameProcessor):
    def __init__(self, alpha=0.7, max_missing=5, dt=0.05):
        self.tracks = {}
        self.track_id_counter = 0
        self.dt = dt
        self.alpha = alpha
        self.process_noise = 1e-2
        self.measure_noise = 1.0
        self.max_missing = max_missing

    def _init_track(self, centroid):
        return {
            "x": np.array([[centroid[0]], [centroid[1]], [0.0], [0.0]]),
            "P": np.eye(4),
            "missing": 0
        }

    def _predict(self, track):
        F = np.array([[1, 0, self.dt, 0],
                      [0, 1, 0, self.dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        Q = self.process_noise * np.eye(4)
        track["x"] = F @ track["x"]
        track["P"] = F @ track["P"] @ F.T + Q
        track["missing"] += 1

    def _update(self, track, measurement):
        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]])
        z = np.array([[measurement[0]], [measurement[1]]])
        pred = H @ track["x"]
        track["x"][:2] = self.alpha * z + (1 - self.alpha) * pred
        track["missing"] = 0

    def process(self, frame):
        output = [0.5, 0.0, 0.0]
        h, w, _ = frame.shape
        cx = w // 2

        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        lower_red1 = np.array([0, 150, 100])    # H od 0 do 5, S ≥ 150, V ≥ 100
        upper_red1 = np.array([5, 255, 255])

        lower_red2 = np.array([175, 150, 100])  # H od 175 do 179, S ≥ 150, V ≥ 100
        upper_red2 = np.array([179, 255, 255])


        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        mask = cv2.bitwise_or(mask1, mask2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        centroids = []

        for c in contours:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx_c = int(M["m10"] / M["m00"])
                cy_c = int(M["m01"] / M["m00"])
                centroids.append((cx_c, cy_c))

        for track in self.tracks.values():
            self._predict(track)

        assigned = set()
        for centroid in centroids:
            if self.tracks:
                dists = {tid: np.linalg.norm(track["x"][:2,0] - np.array(centroid))
                         for tid, track in self.tracks.items()}
                tid = min(dists, key=dists.get)
                if dists[tid] < 50:
                    self._update(self.tracks[tid], centroid)
                    assigned.add(tid)
                    continue
            self.tracks[self.track_id_counter] = self._init_track(centroid)
            self.track_id_counter += 1

        to_delete = [tid for tid, t in self.tracks.items() if t["missing"] > self.max_missing]
        for tid in to_delete:
            del self.tracks[tid]

        tracked_pts = [track["x"][:2,0].astype(int) for track in self.tracks.values()]

        # Connect each point to its closest neighbor
        if tracked_pts:
            # sort points by x-distance to image center
            tracked_pts.sort(key=lambda pt: abs(h-pt[1]))
            
            # Gaussian weights centered on middle index
            N = len(tracked_pts)
            center_idx = 3
            sigma = float(N)/6.0  # controls spread
            weights = [math.exp(-0.5*((i - center_idx)/sigma)**2) for i in range(N)]
            weights = np.array(weights)
            weights /= weights.sum()  # normalize

            # Weighted x-coordinate
            x_vals = np.array([pt[0] for pt in tracked_pts])
            avg_x = np.sum(x_vals * weights)

            # Compute offset
            offset = (avg_x - cx) / cx
            offset = np.clip(offset, -1.0, 1.0)
            output = [1.0, offset, 0.0]

            # Draw line connecting closest neighbors
            connected = set()
            for i, pt in enumerate(tracked_pts[:5]):
                min_dist = float("inf")
                closest_j = None
                for j, other in enumerate(tracked_pts):
                    if i == j or (i,j) in connected or (j,i) in connected:
                        continue
                    d = np.linalg.norm(pt - other)
                    if d < min_dist:
                        min_dist = d
                        closest_j = j
                if closest_j is not None:
                    cv2.line(frame, tuple(pt), tuple(tracked_pts[closest_j]), (0,255,0), 2)
                    connected.add((i, closest_j))


        # self.visualize_tracks(frame, tracked_pts, output)
        # cv2.imshow("KalmanFrame", frame)
        # cv2.waitKey(1)
        return frame

    def visualize_tracks(self, frame, pts, control_vector=None):
        for pt in pts:
            cv2.circle(frame, tuple(pt), 5, (0,0,255), -1)
        if control_vector is not None:
            visibility, steering, fwd_speed = control_vector
            h, w, _ = frame.shape
            cx = w//2
            arrow_length = int(steering * 100)
            cv2.arrowedLine(frame, (cx, h-60), (cx + arrow_length, h-120),
                            (255,0,0), 3, tipLength=0.4)
            cv2.putText(frame, f"Offset: {steering:+.2f}", (30,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,0,0), 2)
            