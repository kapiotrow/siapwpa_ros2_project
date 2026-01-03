#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose

import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import LineString
from copy import deepcopy

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
    ax.plot(xs, ys, color="white", linewidth=2, marker='o', markersize=2, linestyle='-')

# ---------------- ROS2 CONFIG --------------------------

POSE_TOPIC = '/world/mecanum_drive/pose/info'
ROBOT_INDEX = 2   # ← ten indeks zmień jeśli robot będzie w innym miejscu tablicy

def point_to_segment_distance(px, py, ax, ay, bx, by):
    """Zwrot: odległość punktu P od odcinka AB"""
    A = np.array([ax, ay], dtype=float)
    B = np.array([bx, by], dtype=float)
    P = np.array([px, py], dtype=float)

    AB = B - A
    AP = P - A

    # rzutowanie AP na AB — parametr t
    t = np.dot(AP, AB) / np.dot(AB, AB)
    t = max(0.0, min(1.0, t))  # ograniczenie do [0,1]

    closest = A + t * AB
    return np.linalg.norm(P - closest)


# ---------------- SUBSCRIBER NODE ----------------------




class PoseInfoSubscriber(Node):

    

    def __init__(self):
        super().__init__('pose_info_subscriber')

        self.subscription = self.create_subscription(
            PoseArray,
            POSE_TOPIC,
            self.listener_callback,
            100
        )

        self.current_pos = []

        # trajectory
        self.traj_x = []
        self.traj_y = []

        self.loss = 0
        self.start_point = 0
        self.next_start_point = None
        self.ended_lap = False
        self.near_end = False

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

        

        self.center_line = np.vstack([[0, 2], curve_4[::-1], curve_3[::-1], curve_2[::-1], curve_1[::-1], [0, 2]])

        self.center_line = np.array([line*5.78 + [-7.59, -7.64] for line in self.center_line], dtype=np.float32)



        self.track = self.center_line
        self.prev_pos = None

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
        self.line, = self.ax.plot([], [], 'ro-', linewidth=1, markersize=1)

    # ---------------- CALLBACK -----------------------------

    def project_point_to_segment(self, P, A, B):
        AP = P - A
        AB = B - A

        t = np.dot(AP, AB) / np.dot(AB, AB)
        t_clamped = np.clip(t, 0.0, 1.0)

        projection = A + t_clamped * AB
        return projection, t_clamped


    def estiamte_position_points(self):
        P = np.array(self.current_pos)

        min_dist = np.inf
        best_idx = None
        best_proj = None

        

        for i in range(len(self.center_line) - 1):
            A = np.array(self.center_line[i])
            B = np.array(self.center_line[i + 1])

            proj, t = self.project_point_to_segment(P, A, B)

            if 0.0 <= t <= 1.0:
                dist = np.linalg.norm(P - proj)

                if dist < min_dist:
                    min_dist = dist
                    best_idx = i
                    best_proj = proj

        if best_idx is None:
            return  
        if self.prev_pos is not None and self.prev_pos != self.current_pos:
            self.loss += min_dist 

        self.prev_pos = self.current_pos
        
        if self.next_start_point is None:
            #incijalizacja lapu
            if best_idx != 0:
                self.near_end = False
                self.next_start_point = best_idx
                self.ended_lap = False
        else:
            self.next_start_point = best_idx

        if self.next_start_point == 0:
            self.near_end = True
        
        if self.next_start_point is not None:
            if self.next_start_point == self.start_point and self.near_end and self.start_point is not None:
                if self.ended_lap == False:
                    # print on first pass
                    self.get_logger().info(f"Lap ended, final loss: {self.loss}")
                self.next_start_point = None
                self.ended_lap = True
                self.loss = 0
        

        # self.get_logger().info(f"Loss: {self.loss}")
       
    
    def listener_callback(self, msg):

        if len(msg.poses) <= ROBOT_INDEX:
            return

        pose: Pose = msg.poses[ROBOT_INDEX]

        x = pose.position.x
        y = pose.position.y

        self.current_pos = [x, y]

        self.traj_x.append(x)
        self.traj_y.append(y)

        # self.get_logger().info(f"Robot: X={x:.2f}, Y={y:.2f}")

        # update trajectory line
        self.line.set_xdata(self.traj_x)
        self.line.set_ydata(self.traj_y)

        self.ax.relim()
        self.ax.autoscale_view()

        
        self.estiamte_position_points()

        # plt.plot(self.center_line[points, 0], self.center_line[points, 1], 'go')

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
