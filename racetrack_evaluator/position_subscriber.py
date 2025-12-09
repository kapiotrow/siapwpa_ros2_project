#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose

import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import LineString

# ---------------- TRACK GENERATION ---------------------

def parametric_circle(t, xc, yc, R):
    x = xc + R * np.cos(t)
    y = yc + R * np.sin(t)
    return x, y

def inv_parametric_circle(x, xc, R):
    t = np.arccos(min((x - xc) / R, 1.0))
    return t

def create_circle(c, start, end, R, N):
    start_t = inv_parametric_circle(start[0], c[0], R)
    if start[1] < c[1]:
        start_t += np.pi
    end_t = inv_parametric_circle(end[0], c[0], R)
    if end[1] < c[1]:
        end_t += np.pi
    if start_t > end_t:
        start_t -= 2 * np.pi
    arc_T = np.linspace(start_t, end_t, N)
    return np.vstack(parametric_circle(arc_T, c[0], c[1], R)).T

def create_circle_angle(c, start, angle, R, N):
    start_t = inv_parametric_circle(start[0], c[0], R)
    if start[1] < c[1]:
        start_t += np.pi
    end_t = start_t + angle
    arc_T = np.linspace(start_t, end_t, N)
    return np.vstack(parametric_circle(arc_T, c[0], c[1], R)).T

def print_border(ax, waypoints):
    line = LineString(waypoints)
    xs, ys = line.xy
    ax.plot(xs, ys, color="white", linewidth=2)

# ---------------- ROS2 CONFIG --------------------------

POSE_TOPIC = '/world/mecanum_drive/pose/info'
ROBOT_INDEX = 2   # ← ten indeks zmień jeśli robot będzie w innym miejscu tablicy

# ---------------- SUBSCRIBER NODE ----------------------

class PoseInfoSubscriber(Node):

    def __init__(self):
        super().__init__('pose_info_subscriber')

        self.subscription = self.create_subscription(
            PoseArray,
            POSE_TOPIC,
            self.listener_callback,
            10
        )

        # trajectory
        self.traj_x = []
        self.traj_y = []

        # -------------- prepare track once ------------------

        points = np.array([
            [0.7, 0.0],
            [3.8, 0.0],
            [4.5, 0.7],
            [4.5, 1.0],
            [3.8, 2.2],
            [0.7, 3.0],
            [0.0, 2.7],
            [0.0, 0.7]
        ])

        R_1 = 0.7
        N_1 = 15

        curve_1 = create_circle((points[0, 0], points[7, 1]), points[7], points[0], R_1, N_1)
        curve_2 = create_circle_angle((points[1, 0], points[2, 1]), points[1], np.pi/2, R_1, N_1)
        curve_3 = create_circle_angle((3.8, 1.0), points[3], np.pi/2 - 0.55, R_1, N_1)
        curve_4 = np.flipud(create_circle_angle((points[5, 0], points[6, 1]), points[6], -np.pi/2 - 0.55, R_1, N_1))

        center_line = np.vstack([curve_1, curve_2, curve_3, curve_4])
        center_line = np.array([line*6 + [-7.5, -7.5] for line in center_line], dtype=np.float32)

        self.track = center_line

        # ------------------- PLOT SETUP ---------------------

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_title("Trajektoria robota na mapie")
        self.ax.set_facecolor("black")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.grid(True, color="gray")

        # draw static map
        print_border(self.ax, self.track)

        # trajectory line
        self.line, = self.ax.plot([], [], 'r-', linewidth=2)

    # ---------------- CALLBACK -----------------------------

    def listener_callback(self, msg):

        if len(msg.poses) <= ROBOT_INDEX:
            return

        pose: Pose = msg.poses[ROBOT_INDEX]

        x = pose.position.x
        y = pose.position.y

        self.traj_x.append(x)
        self.traj_y.append(y)

        self.get_logger().info(f"Robot: X={x:.2f}, Y={y:.2f}")

        # update trajectory line
        self.line.set_xdata(self.traj_x)
        self.line.set_ydata(self.traj_y)

        self.ax.relim()
        self.ax.autoscale_view()

        plt.draw()
        plt.pause(0.001)


# ---------------- MAIN EXECUTION ------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PoseInfoSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
