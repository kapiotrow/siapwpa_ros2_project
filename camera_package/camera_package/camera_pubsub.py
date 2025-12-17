import rclpy
from rclpy.node import Node
# from camera_subscriber import RGBCameraSubscriber
import cv2
from cv_bridge import CvBridge
from .line_follower import LineFollower
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from pynput import keyboard as kb
import numpy as np
import math

import time 
from enum import Enum

TOPIC_LIDAR = '/world/mecanum_drive/model/samochod/link/base_footprint/sensor/velodyne_lidar/scan'
VEL_TOPIC = '/model/samochod/cmd_vel'
TOPIC_RGB = '/world/mecanum_drive/model/samochod/link/base_footprint/sensor/realsense_rgbd/image'
TOPIC_DEPTH = '/world/mecanum_drive/model/samochod/link/base_footprint/sensor/realsense_depth/depth_image'
bridge = CvBridge()
START = False

class AvoidState(Enum):
    LINE = 0
    TURN_AWAY = 1
    PASS = 2
    TURN_BACK = 3


class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('velocity_publisher')

        self.publisher_ = self.create_publisher(Twist, VEL_TOPIC, 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.dt = 0.5

        # angles
        self.front_angle = 0.26  # ~15°
        self.dist_thresh = 1.0

        self.state = AvoidState.LINE
        self.turn_dir = 0        # +1 lewo, -1 prawo
        self.turn_memory = 0.0
        self.turn_speed = 0.8    # rad/s

        # dist
        self.min_front = 10.0
        self.min_left = 10.0
        self.min_right = 10.

        # SUBSKRYBER W TYM SAMYM NODE
        self.subscription = self.create_subscription(
            Image,
            TOPIC_RGB,
            self.camera_callback,
            10
        )

        # lidar subscriber
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            TOPIC_LIDAR,
            self.lidar_callback,
            10
        )

        self.obstacle_detected = False

        self.line_follower = LineFollower()
        self.directions_vector = [0.0, 0.0]
        self.manual = False
        self.keys = set()

        self.listener = kb.Listener(
            on_press=self.on_press,
            on_release=self.on_release
            )
        self.listener.start()

    def on_press(self, key):
        try:
            if key.char == 'm':    # toggle manual mode
                self.manual = not self.manual
                mode = "MANUAL" if self.manual else "AUTOMATIC"
                self.get_logger().info(f"Switched to {mode}")
            else:
                self.keys.add(key.char)
        except AttributeError:
            # arrow keys & specials
            self.keys.add(key)

    def on_release(self, key):
        try:
            self.keys.discard(key.char)
        except AttributeError:
            self.keys.discard(key)



    def camera_callback(self, msg):
        cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.directions_vector = self.line_follower.update(cv_image)


    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)

        # zabezpieczenie przed inf / nan
        ranges = np.where(np.isinf(ranges), msg.range_max, ranges)
        ranges = np.where(np.isnan(ranges), msg.range_max, ranges)

        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # sektory kątowe
        front_mask = np.abs(angles) < math.radians(15)
        left_mask  = (angles > math.radians(15)) & (angles < math.radians(90))
        right_mask = (angles < -math.radians(15)) & (angles > -math.radians(90))

        self.min_front = np.min(ranges[front_mask]) if np.any(front_mask) else msg.range_max
        self.min_left  = np.min(ranges[left_mask])  if np.any(left_mask)  else msg.range_max
        self.min_right = np.min(ranges[right_mask]) if np.any(right_mask) else msg.range_max

        self.get_logger().info(f"Received LIDAR data:")
        self.get_logger().info(f"Angle min: {msg.angle_min}")
        self.get_logger().info(f"Angle max: {msg.angle_max}")
        self.get_logger().info(f"Number of ranges: {len(msg.ranges)}")
        self.get_logger().info(f"Ranges (first 10 points): {msg.ranges[:10]}")


    def timer_callback(self):
        START = True

        visibility = self.directions_vector[0]
        offset = self.directions_vector[1]

        max_speed_forward = 1.0  # m/s
        max_speed_sideways = 4.0 # m/s


        if self.manual:
            return

        msg = Twist()

        # ==========================
        # STATE: LINE FOLLOWING
        # ==========================
        if self.state == AvoidState.LINE:
            if self.min_front < self.dist_thresh:
                # decide turn direction
                self.turn_dir = +1 if self.left_dist > self.right_dist else -1
                self.turn_memory = 0.0
                self.avoid = AvoidState.TURN_AWAY
                return

            # normal line following
            visibility, offset = self.directions_vector[:2]
            msg.linear.x = 4.0 * offset
            msg.angular.z = -1.0 * visibility
            self.publisher_.publish(msg)
            return

        # -----------------------------
        # TURN_AWAY – skręcaj aż przód wolny
        # -----------------------------
        if self.state == AvoidState.TURN_AWAY:
            if self.min_front < self.dist_thresh:
                msg.angular.z = self.turn_dir * self.turn_speed
                self.turn_memory += self.turn_speed * self.dt
                self.publisher_.publish(msg)
                return
            else:
                self.state = AvoidState.PASS
                return

        # ==========================
        # STATE: PASS ALONGSIDE
        # ==========================
        if self.state == AvoidState.PASS:
            side_dist = self.min_left if self.turn_dir == 1 else self.min_right

            if side_dist < self.dist_thresh:
                msg.linear.y = 0.5
                self.publisher_.publish(msg)
                return
            else:
                self.state = AvoidState.TURN_BACK
                return

        # ==========================
        # STATE: TURN BACK
        # ==========================
        if self.state == AvoidState.TURN_BACK:
            if self.turn_memory > 0.0:
                msg.angular.z = -self.turn_dir * self.turn_speed
                self.turn_memory -= self.turn_speed * self.dt
                self.publisher_.publish(msg)
                return
            else:
                self.state = AvoidState.LINE
                return



def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

  
if __name__ == '__main__':
    main()
